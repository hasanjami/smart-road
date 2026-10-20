//! Fixed approach lanes and the twelve intersection movements.
//!
//! Traffic keeps to the right. Each incoming direction has one right lane,
//! one straight lane, and one left lane. A movement is a polyline of straight
//! segments and a circular arc, stored by arc length so a vehicle can follow
//! it at a chosen speed and take its heading from the tangent.

use std::sync::OnceLock;

use crate::collision::Pose;
use crate::config::{
    CELL_SIZE, CENTER, CLEAR_BUFFER, EXIT_MARGIN, GRID_SIZE, LANE_WIDTH, LEFT_RADIUS, RIGHT_RADIUS,
    ROAD_MAX, ROAD_MIN, SPAWN_INSET, STOP_BUFFER, VEHICLE_LENGTH, VEHICLE_WIDTH, WINDOW,
};
use crate::vec2::Vec2;

/// Side of the intersection a vehicle comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Direction {
    North,
    South,
    East,
    West,
}

impl Direction {
    pub const ALL: [Self; 4] = [Self::North, Self::South, Self::East, Self::West];

    pub fn index(self) -> usize {
        match self {
            Self::North => 0,
            Self::South => 1,
            Self::East => 2,
            Self::West => 3,
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::North => "North",
            Self::South => "South",
            Self::East => "East",
            Self::West => "West",
        }
    }
}

/// The only legal movement out of a lane. It never changes after spawn.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Turn {
    Right,
    Straight,
    Left,
}

impl Turn {
    pub const ALL: [Self; 3] = [Self::Right, Self::Straight, Self::Left];

    pub fn index(self) -> usize {
        match self {
            Self::Right => 0,
            Self::Straight => 1,
            Self::Left => 2,
        }
    }

    pub fn letter(self) -> &'static str {
        match self {
            Self::Right => "R",
            Self::Straight => "S",
            Self::Left => "L",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::Right => "Right",
            Self::Straight => "Straight",
            Self::Left => "Left",
        }
    }
}

pub fn route_index(direction: Direction, turn: Turn) -> usize {
    direction.index() * 3 + turn.index()
}

/// Arc-length polyline. Samples are about two pixels apart.
#[derive(Clone, Debug)]
pub struct Path {
    samples: Vec<Vec2>,
    cumulative: Vec<f32>,
    length: f32,
}

impl Path {
    pub fn length(&self) -> f32 {
        self.length
    }

    /// Position and unit tangent at arc length `s`.
    pub fn sample(&self, s: f32) -> (Vec2, Vec2) {
        if self.samples.len() < 2 {
            return (
                self.samples.first().copied().unwrap_or(Vec2::ZERO),
                Vec2::new(1.0, 0.0),
            );
        }
        let s = s.clamp(0.0, self.length);
        let mut i = self.cumulative.partition_point(|&c| c < s);
        if i == 0 {
            i = 1;
        }
        if i >= self.samples.len() {
            i = self.samples.len() - 1;
        }
        let i0 = i - 1;
        let span = (self.cumulative[i] - self.cumulative[i0]).max(1e-4);
        let t = ((s - self.cumulative[i0]) / span).clamp(0.0, 1.0);
        let pos = self.samples[i0].lerp(self.samples[i], t);
        let tangent = (self.samples[i] - self.samples[i0]).normalized();
        (pos, tangent)
    }
}

/// One of the twelve legal movements.
#[derive(Clone, Debug)]
pub struct Route {
    pub direction: Direction,
    pub turn: Turn,
    pub path: Path,
    /// First arc length at which the body touches the conflict square.
    pub s_enter: f32,
    /// Arc length at which the body has fully left that square.
    pub s_clear: f32,
    /// Center arc length where a vehicle without a reservation must stop.
    pub s_stop: f32,
    /// Conflict cells occupied while crossing. Bit `row * 6 + col`.
    pub mask: u64,
}

impl Route {
    pub fn index(&self) -> usize {
        route_index(self.direction, self.turn)
    }

    pub fn name(&self) -> String {
        format!("{} {}", self.direction.label(), self.turn.label())
    }
}

pub fn routes() -> &'static [Route] {
    static ROUTES: OnceLock<Vec<Route>> = OnceLock::new();
    ROUTES.get_or_init(build_routes)
}

pub fn route(direction: Direction, turn: Turn) -> &'static Route {
    &routes()[route_index(direction, turn)]
}

pub fn masks_conflict(a: usize, b: usize) -> bool {
    let routes = routes();
    routes[a].mask & routes[b].mask != 0
}

/// Smallest edge gap between bodies sampled along both paths inside the square.
pub fn intersection_separation(a: usize, b: usize) -> f32 {
    let routes = routes();
    let left = &routes[a];
    let right = &routes[b];
    let mut best = f32::MAX;
    let mut s = left.s_enter;
    while s <= left.s_clear {
        let pose_a = pose_on(left, s);
        let mut t = right.s_enter;
        while t <= right.s_clear {
            let pose_b = pose_on(right, t);
            best = best.min(crate::collision::edge_distance(pose_a, pose_b));
            t += 8.0;
        }
        s += 8.0;
    }
    best
}

pub fn pose_on(route: &Route, s: f32) -> Pose {
    let (pos, tangent) = route.path.sample(s);
    Pose::from_tangent(pos, tangent)
}

fn build_routes() -> Vec<Route> {
    let mut built = Vec::with_capacity(12);
    for direction in Direction::ALL {
        for turn in Turn::ALL {
            let points = keypoints(direction, turn);
            let path = densify(&points);
            let (s_enter, s_clear) = intersection_range(&path);
            let s_stop = (s_enter - STOP_BUFFER).max(0.0);
            let mask = rasterize(&path, s_enter, s_clear);
            built.push(Route {
                direction,
                turn,
                path,
                s_enter,
                s_clear,
                s_stop,
                mask,
            });
        }
    }
    built
}

fn keypoints(direction: Direction, turn: Turn) -> Vec<Vec2> {
    match (direction, turn) {
        (Direction::South, Turn::Straight) => line(
            lane_x(Direction::South, Turn::Straight),
            spawn_y(Direction::South),
            lane_x(Direction::South, Turn::Straight),
            -EXIT_MARGIN,
        ),
        (Direction::North, Turn::Straight) => line(
            lane_x(Direction::North, Turn::Straight),
            spawn_y(Direction::North),
            lane_x(Direction::North, Turn::Straight),
            WINDOW + EXIT_MARGIN,
        ),
        (Direction::West, Turn::Straight) => line(
            spawn_x(Direction::West),
            lane_y(Direction::West, Turn::Straight),
            WINDOW + EXIT_MARGIN,
            lane_y(Direction::West, Turn::Straight),
        ),
        (Direction::East, Turn::Straight) => line(
            spawn_x(Direction::East),
            lane_y(Direction::East, Turn::Straight),
            -EXIT_MARGIN,
            lane_y(Direction::East, Turn::Straight),
        ),
        (Direction::South, Turn::Right) => right_south(),
        (Direction::North, Turn::Right) => right_north(),
        (Direction::West, Turn::Right) => right_west(),
        (Direction::East, Turn::Right) => right_east(),
        (Direction::South, Turn::Left) => left_south(),
        (Direction::North, Turn::Left) => left_north(),
        (Direction::West, Turn::Left) => left_west(),
        (Direction::East, Turn::Left) => left_east(),
    }
}

/// Northbound lanes sit on the east side of the vertical road.
fn lane_x(direction: Direction, turn: Turn) -> f32 {
    let slot = match turn {
        Turn::Left => 0.5,
        Turn::Straight => 1.5,
        Turn::Right => 2.5,
    };
    match direction {
        Direction::South => CENTER + slot * LANE_WIDTH,
        Direction::North => CENTER - slot * LANE_WIDTH,
        _ => CENTER,
    }
}

/// Eastbound lanes sit on the south side of the horizontal road.
fn lane_y(direction: Direction, turn: Turn) -> f32 {
    let slot = match turn {
        Turn::Left => 0.5,
        Turn::Straight => 1.5,
        Turn::Right => 2.5,
    };
    match direction {
        Direction::West => CENTER + slot * LANE_WIDTH,
        Direction::East => CENTER - slot * LANE_WIDTH,
        _ => CENTER,
    }
}

fn spawn_y(direction: Direction) -> f32 {
    match direction {
        Direction::South => WINDOW - SPAWN_INSET,
        Direction::North => SPAWN_INSET,
        _ => CENTER,
    }
}

fn spawn_x(direction: Direction) -> f32 {
    match direction {
        Direction::West => SPAWN_INSET,
        Direction::East => WINDOW - SPAWN_INSET,
        _ => CENTER,
    }
}

fn line(x0: f32, y0: f32, x1: f32, y1: f32) -> Vec<Vec2> {
    vec![Vec2::new(x0, y0), Vec2::new(x1, y1)]
}

/// Screen-space circle. Theta 0 points east, and positive theta turns toward north.
fn arc(center: Vec2, radius: f32, theta0: f32, theta1: f32) -> Vec<Vec2> {
    let sweep = theta1 - theta0;
    let steps = ((sweep.abs() / 0.04).ceil() as usize).max(4);
    (0..=steps)
        .map(|i| {
            let t = theta0 + sweep * (i as f32) / steps as f32;
            Vec2::new(center.x + radius * t.cos(), center.y - radius * t.sin())
        })
        .collect()
}

fn join(parts: &[Vec<Vec2>]) -> Vec<Vec2> {
    let mut out = Vec::new();
    for part in parts {
        for p in part {
            if out.last().is_none_or(|q: &Vec2| q.distance(*p) > 0.4) {
                out.push(*p);
            }
        }
    }
    out
}

fn right_south() -> Vec<Vec2> {
    let x = lane_x(Direction::South, Turn::Right);
    let y = lane_y(Direction::West, Turn::Right);
    let r = RIGHT_RADIUS;
    let center = Vec2::new(x + r, y + r);
    join(&[
        line(x, spawn_y(Direction::South), x, center.y),
        arc(center, r, std::f32::consts::PI, std::f32::consts::FRAC_PI_2),
        line(center.x, y, WINDOW + EXIT_MARGIN, y),
    ])
}

fn right_north() -> Vec<Vec2> {
    let x = lane_x(Direction::North, Turn::Right);
    let y = lane_y(Direction::East, Turn::Right);
    let r = RIGHT_RADIUS;
    let center = Vec2::new(x - r, y - r);
    join(&[
        line(x, spawn_y(Direction::North), x, center.y),
        arc(center, r, 0.0, -std::f32::consts::FRAC_PI_2),
        line(center.x, y, -EXIT_MARGIN, y),
    ])
}

fn right_west() -> Vec<Vec2> {
    let y = lane_y(Direction::West, Turn::Right);
    let x = lane_x(Direction::North, Turn::Right);
    let r = RIGHT_RADIUS;
    let center = Vec2::new(x - r, y + r);
    join(&[
        line(spawn_x(Direction::West), y, center.x, y),
        arc(center, r, std::f32::consts::FRAC_PI_2, 0.0),
        line(x, center.y, x, WINDOW + EXIT_MARGIN),
    ])
}

fn right_east() -> Vec<Vec2> {
    let y = lane_y(Direction::East, Turn::Right);
    let x = lane_x(Direction::South, Turn::Right);
    let r = RIGHT_RADIUS;
    let center = Vec2::new(x + r, y - r);
    join(&[
        line(spawn_x(Direction::East), y, center.x, y),
        arc(
            center,
            r,
            -std::f32::consts::FRAC_PI_2,
            -std::f32::consts::PI,
        ),
        line(x, center.y, x, -EXIT_MARGIN),
    ])
}

fn left_south() -> Vec<Vec2> {
    let x = lane_x(Direction::South, Turn::Left);
    let y = lane_y(Direction::East, Turn::Left);
    let r = LEFT_RADIUS;
    let center = Vec2::new(x - r, y + r);
    join(&[
        line(x, spawn_y(Direction::South), x, center.y),
        arc(center, r, 0.0, std::f32::consts::FRAC_PI_2),
        line(center.x, y, -EXIT_MARGIN, y),
    ])
}

fn left_north() -> Vec<Vec2> {
    let x = lane_x(Direction::North, Turn::Left);
    let y = lane_y(Direction::West, Turn::Left);
    let r = LEFT_RADIUS;
    let center = Vec2::new(x + r, y - r);
    join(&[
        line(x, spawn_y(Direction::North), x, center.y),
        arc(
            center,
            r,
            std::f32::consts::PI,
            std::f32::consts::PI + std::f32::consts::FRAC_PI_2,
        ),
        line(center.x, y, WINDOW + EXIT_MARGIN, y),
    ])
}

fn left_west() -> Vec<Vec2> {
    let y = lane_y(Direction::West, Turn::Left);
    let x = lane_x(Direction::South, Turn::Left);
    let r = LEFT_RADIUS;
    let center = Vec2::new(x - r, y - r);
    join(&[
        line(spawn_x(Direction::West), y, center.x, y),
        arc(center, r, -std::f32::consts::FRAC_PI_2, 0.0),
        line(x, center.y, x, -EXIT_MARGIN),
    ])
}

fn left_east() -> Vec<Vec2> {
    let y = lane_y(Direction::East, Turn::Left);
    let x = lane_x(Direction::North, Turn::Left);
    let r = LEFT_RADIUS;
    let center = Vec2::new(x + r, y + r);
    join(&[
        line(spawn_x(Direction::East), y, center.x, y),
        arc(center, r, std::f32::consts::FRAC_PI_2, std::f32::consts::PI),
        line(x, center.y, x, WINDOW + EXIT_MARGIN),
    ])
}

fn densify(points: &[Vec2]) -> Path {
    let mut samples = Vec::new();
    if let Some(first) = points.first() {
        samples.push(*first);
    }
    for pair in points.windows(2) {
        let a = pair[0];
        let b = pair[1];
        let dist = a.distance(b);
        if dist < 0.5 {
            continue;
        }
        let n = (dist / 2.0).ceil() as usize;
        for i in 1..=n {
            samples.push(a.lerp(b, i as f32 / n as f32));
        }
    }
    let mut cumulative = Vec::with_capacity(samples.len());
    cumulative.push(0.0);
    for pair in samples.windows(2) {
        let next = cumulative.last().copied().unwrap_or(0.0) + pair[0].distance(pair[1]);
        cumulative.push(next);
    }
    let length = cumulative.last().copied().unwrap_or(0.0);
    Path {
        samples,
        cumulative,
        length,
    }
}

fn body_hits_square(path: &Path, s: f32) -> bool {
    let (pos, tangent) = path.sample(s);
    let pose = Pose::from_tangent(pos, tangent);
    rect_hits_aabb(pose, 0.0, ROAD_MIN, ROAD_MIN, ROAD_MAX, ROAD_MAX)
}

fn rect_hits_aabb(
    pose: Pose,
    inflate: f32,
    min_x: f32,
    min_y: f32,
    max_x: f32,
    max_y: f32,
) -> bool {
    let corners = pose.corners(inflate);
    if corners
        .iter()
        .any(|p| p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y)
    {
        return true;
    }
    let xs = [min_x, max_x, max_x, min_x];
    let ys = [min_y, min_y, max_y, max_y];
    for i in 0..4 {
        if point_in_pose(Vec2::new(xs[i], ys[i]), pose, inflate) {
            return true;
        }
    }
    false
}

fn point_in_pose(point: Vec2, pose: Pose, inflate: f32) -> bool {
    let d = point - pose.center;
    d.dot(pose.forward).abs() <= VEHICLE_LENGTH * 0.5 + inflate
        && d.dot(pose.right).abs() <= VEHICLE_WIDTH * 0.5 + inflate
}

fn intersection_range(path: &Path) -> (f32, f32) {
    let mut first = None;
    let mut last = None;
    let mut s = 0.0;
    while s <= path.length {
        if body_hits_square(path, s) {
            if first.is_none() {
                first = Some(s);
            }
            last = Some(s);
        }
        s += 2.0;
    }
    let enter = first.unwrap_or(path.length * 0.45);
    let clear = (last.unwrap_or(enter) + 2.0 + CLEAR_BUFFER).min(path.length);
    (enter, clear.max(enter))
}

fn rasterize(path: &Path, enter: f32, clear: f32) -> u64 {
    let mut mask = 0u64;
    let inflate = 14.0;
    let mut s = enter;
    while s <= clear {
        let (pos, tangent) = path.sample(s);
        let pose = Pose::from_tangent(pos, tangent);
        mark_pose(&mut mask, pose, inflate);
        s += 4.0;
    }
    mask
}

fn mark_pose(mask: &mut u64, pose: Pose, inflate: f32) {
    let steps = 4;
    let half_l = VEHICLE_LENGTH * 0.5 + inflate;
    let half_w = VEHICLE_WIDTH * 0.5 + inflate;
    for i in 0..=steps {
        for j in 0..=steps {
            let u = -1.0 + 2.0 * i as f32 / steps as f32;
            let v = -1.0 + 2.0 * j as f32 / steps as f32;
            let p = pose.center + pose.forward * (u * half_l) + pose.right * (v * half_w);
            if let Some(bit) = cell_bit(p.x, p.y) {
                *mask |= 1u64 << bit;
            }
        }
    }
}

fn cell_bit(x: f32, y: f32) -> Option<u8> {
    if !(ROAD_MIN..ROAD_MAX).contains(&x) || !(ROAD_MIN..ROAD_MAX).contains(&y) {
        return None;
    }
    let col = ((x - ROAD_MIN) / CELL_SIZE).floor() as i32;
    let row = ((y - ROAD_MIN) / CELL_SIZE).floor() as i32;
    if (0..GRID_SIZE).contains(&col) && (0..GRID_SIZE).contains(&row) {
        Some((row * GRID_SIZE + col) as u8)
    } else {
        None
    }
}
