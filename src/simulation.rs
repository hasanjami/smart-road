//! Time-based intersection simulation: spawning, following, and reservations.

use std::collections::HashSet;

use rand::rngs::SmallRng;
use rand::{Rng, SeedableRng};

use crate::collision::{too_close, Pose};
use crate::config::{
    ACCELERATION, AGE_LIMIT, BRAKING, COLLISION_DISTANCE, CONTROL_ZONE, CROSSING_MIN_SPEED,
    DEFER_LIMIT, EMERGENCY_BRAKING, FOLLOW_CONTROL_BUFFER, MAX_SUBSTEP, MIN_FOLLOWING_GAP,
    RANDOM_PERIOD, SATURATED_QUEUE, TIME_MARGIN, VEHICLE_LENGTH, WARNING_DISTANCE,
};
use crate::intersection::IntersectionManager;
use crate::routes::{masks_conflict, pose_on, route_index, routes, Direction, Turn};
use crate::statistics::Statistics;
use crate::vehicle::{CompletedTrip, SpeedLevel, Vehicle, VehicleState};

/// A spawn was refused because the lane entrance was not clear, or the run is frozen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SpawnReject;

#[derive(Clone, Copy, Debug)]
struct DeferredSpawn {
    direction: Direction,
    turn: Turn,
    speed: SpeedLevel,
    variant: u8,
    queued_at: f32,
}

/// Full simulation state. Own one of these per run.
pub struct Simulation {
    time: f32,
    next_id: u64,
    vehicles: Vec<Vehicle>,
    completed: Vec<CompletedTrip>,
    manager: IntersectionManager,
    stats: Statistics,
    rng: SmallRng,
    random_enabled: bool,
    random_timer: f32,
    deferred: [Option<DeferredSpawn>; 12],
    frozen: bool,
    close_pairs: HashSet<(u64, u64)>,
    collision_pairs: HashSet<(u64, u64)>,
    yielding: HashSet<u64>,
}

impl Simulation {
    pub fn new(seed: u64) -> Self {
        Self {
            time: 0.0,
            next_id: 1,
            vehicles: Vec::new(),
            completed: Vec::new(),
            manager: IntersectionManager::new(),
            stats: Statistics::default(),
            rng: SmallRng::seed_from_u64(seed),
            random_enabled: false,
            random_timer: RANDOM_PERIOD,
            deferred: [None; 12],
            frozen: false,
            close_pairs: HashSet::new(),
            collision_pairs: HashSet::new(),
            yielding: HashSet::new(),
        }
    }

    pub fn time(&self) -> f32 {
        self.time
    }

    pub fn statistics(&self) -> &Statistics {
        &self.stats
    }

    pub fn vehicles(&self) -> &[Vehicle] {
        &self.vehicles
    }

    pub fn completed_trips(&self) -> &[CompletedTrip] {
        &self.completed
    }

    pub fn active_count(&self) -> u32 {
        self.vehicles.len() as u32
    }

    pub fn random_enabled(&self) -> bool {
        self.random_enabled
    }

    pub fn is_frozen(&self) -> bool {
        self.frozen
    }

    pub fn toggle_random(&mut self) -> bool {
        self.random_enabled = !self.random_enabled;
        self.random_enabled
    }

    pub fn set_random_enabled(&mut self, enabled: bool) {
        self.random_enabled = enabled;
    }

    pub fn freeze(&mut self) {
        self.frozen = true;
        self.random_enabled = false;
    }

    pub fn set_desired_speed(&mut self, id: u64, speed: f32) -> bool {
        if let Some(vehicle) = self.vehicles.iter_mut().find(|v| v.id == id) {
            vehicle.desired_speed = speed.max(0.0);
            true
        } else {
            false
        }
    }

    pub fn run_for(&mut self, seconds: f32, dt: f32) {
        if dt <= 0.0 {
            return;
        }
        let steps = (seconds / dt).ceil() as u32;
        for _ in 0..steps {
            self.update(dt);
        }
    }

    /// Spawn a specific movement when the entrance gap is free.
    pub fn try_spawn(
        &mut self,
        direction: Direction,
        turn: Turn,
        speed: SpeedLevel,
        variant: u8,
    ) -> Result<u64, SpawnReject> {
        if self.frozen {
            return Err(SpawnReject);
        }
        let index = route_index(direction, turn);
        if !self.lane_clear(index) {
            return Err(SpawnReject);
        }
        Ok(self.insert_vehicle(direction, turn, speed, variant))
    }

    /// Pick a random legal route and speed for an arrow-key spawn.
    pub fn spawn_from_direction(&mut self, direction: Direction) -> Result<u64, SpawnReject> {
        let turn = Turn::ALL[self.rng.gen_range(0..3)];
        let speed = SpeedLevel::from_index(self.rng.gen_range(0..3));
        let variant = self.rng.gen_range(0..3);
        self.try_spawn(direction, turn, speed, variant)
    }

    /// Arrow-key entry point. A blocked lane keeps at most one short-lived request.
    pub fn keyboard_spawn(&mut self, direction: Direction) {
        let turn = Turn::ALL[self.rng.gen_range(0..3)];
        let speed = SpeedLevel::from_index(self.rng.gen_range(0..3));
        let variant = self.rng.gen_range(0..3);
        self.request_spawn(direction, turn, speed, variant);
    }

    pub fn request_spawn(
        &mut self,
        direction: Direction,
        turn: Turn,
        speed: SpeedLevel,
        variant: u8,
    ) {
        if self.try_spawn(direction, turn, speed, variant).is_ok() {
            return;
        }
        let index = route_index(direction, turn);
        if self.deferred[index].is_some() {
            self.stats.rejected_spawns += 1;
        } else {
            self.deferred[index] = Some(DeferredSpawn {
                direction,
                turn,
                speed,
                variant,
                queued_at: self.time,
            });
            self.stats.deferred_spawns += 1;
        }
    }

    pub fn update(&mut self, dt: f32) {
        if self.frozen || !dt.is_finite() || dt <= 0.0 {
            return;
        }
        let dt = dt.min(0.25);
        self.time += dt;
        self.stats.simulation_time = self.time;
        self.flush_deferred();
        self.tick_random(dt);
        let mut left = dt;
        while left > 1e-6 {
            let step = left.min(MAX_SUBSTEP);
            self.integrate(step);
            left -= step;
        }
    }

    pub fn debug_dump(&self) -> String {
        let mut lines = vec![format!(
            "t={:.2}s active={} completed={} collisions={} close={} emergency={} queues={}",
            self.time,
            self.vehicles.len(),
            self.stats.completed,
            self.stats.collisions,
            self.stats.close_calls,
            self.stats.emergency_stops,
            self.stats.max_queue_overall
        )];
        for v in &self.vehicles {
            lines.push(format!(
                "  #{:<3} {} {:<8} {:<11} s={:6.1} v={:5.1} desired={:5.1} hold={}",
                v.id,
                v.direction.label(),
                v.turn.label(),
                v.state.label(),
                v.progress,
                v.velocity,
                v.desired_speed,
                self.manager.holds(v.id)
            ));
        }
        lines.join("\n")
    }

    fn insert_vehicle(
        &mut self,
        direction: Direction,
        turn: Turn,
        speed: SpeedLevel,
        variant: u8,
    ) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        let index = route_index(direction, turn);
        let vehicle = Vehicle::new(id, direction, turn, speed, variant, self.time);
        self.vehicles.push(vehicle);
        self.manager.ensure_ticket(id, index, self.time);
        self.stats.spawned += 1;
        self.stats.observe_velocity(0.0, 0.0);
        id
    }

    fn lane_clear(&self, route: usize) -> bool {
        let need = VEHICLE_LENGTH + MIN_FOLLOWING_GAP;
        self.vehicles
            .iter()
            .filter(|v| v.route == route)
            .all(|v| v.progress >= need)
    }

    fn queue_len(&self, route: usize) -> u32 {
        let enter = routes()[route].s_enter;
        self.vehicles
            .iter()
            .filter(|v| v.route == route && v.progress < enter)
            .count() as u32
    }

    fn flush_deferred(&mut self) {
        for index in 0..12 {
            let Some(request) = self.deferred[index] else {
                continue;
            };
            if self.lane_clear(index) {
                self.deferred[index] = None;
                let _ = self.try_spawn(
                    request.direction,
                    request.turn,
                    request.speed,
                    request.variant,
                );
            } else if self.time - request.queued_at > DEFER_LIMIT {
                self.deferred[index] = None;
                self.stats.rejected_spawns += 1;
            }
        }
    }

    fn tick_random(&mut self, dt: f32) {
        if !self.random_enabled {
            return;
        }
        self.random_timer -= dt;
        let mut spins = 0;
        while self.random_timer <= 0.0 && spins < 4 {
            self.random_timer += RANDOM_PERIOD;
            self.spawn_random();
            spins += 1;
        }
    }

    fn spawn_random(&mut self) {
        for _ in 0..6 {
            let direction = Direction::ALL[self.rng.gen_range(0..4)];
            let turn = Turn::ALL[self.rng.gen_range(0..3)];
            let index = route_index(direction, turn);
            if self.queue_len(index) >= SATURATED_QUEUE {
                continue;
            }
            let speed = SpeedLevel::from_index(self.rng.gen_range(0..3));
            let variant = self.rng.gen_range(0..3);
            if self.try_spawn(direction, turn, speed, variant).is_ok() {
                return;
            }
        }
        self.stats.rejected_spawns += 1;
    }

    fn integrate(&mut self, dt: f32) {
        self.note_zone_entry();
        self.sync_reservations();
        let count = self.vehicles.len();
        let mut targets = vec![0.0; count];
        let mut leaders = vec![None; count];
        for i in 0..count {
            leaders[i] = self.leader_index(i);
            targets[i] = self.compute_target(i, leaders[i]);
        }
        let mut order: Vec<usize> = (0..count).collect();
        order.sort_by(|&a, &b| {
            let pa = move_priority(&self.vehicles[a]);
            let pb = move_priority(&self.vehicles[b]);
            pb.cmp(&pa).then_with(|| {
                self.vehicles[b]
                    .progress
                    .total_cmp(&self.vehicles[a].progress)
            })
        });
        for idx in order {
            self.advance(idx, targets[idx], leaders[idx], dt);
            self.refresh_state(idx);
        }
        self.measure_pairs();
        self.record_queues();
        self.retire_finished();
    }

    fn note_zone_entry(&mut self) {
        let now = self.time;
        for vehicle in &mut self.vehicles {
            let geom = &routes()[vehicle.route];
            let dist = geom.s_stop - vehicle.progress;
            if vehicle.zone_enter.is_none()
                && dist <= CONTROL_ZONE
                && vehicle.progress < geom.s_enter
            {
                vehicle.zone_enter = Some(now);
            }
        }
    }

    fn sync_reservations(&mut self) {
        self.manager.prune(self.time);
        let mut release = Vec::new();
        for vehicle in &self.vehicles {
            if self.manager.holds(vehicle.id) {
                let clear = routes()[vehicle.route].s_clear;
                if vehicle.progress >= clear {
                    release.push(vehicle.id);
                }
            } else if vehicle.progress < routes()[vehicle.route].s_clear {
                self.manager
                    .ensure_ticket(vehicle.id, vehicle.route, vehicle.spawn_time);
            }
        }
        for id in release {
            self.manager.release(id, self.time);
        }

        let mut waiting: Vec<u64> = self
            .vehicles
            .iter()
            .filter(|v| !self.manager.holds(v.id) && self.ready_to_commit(v))
            .map(|v| v.id)
            .collect();
        waiting.sort_by(|&a, &b| {
            self.ticket_key(a)
                .total_cmp(&self.ticket_key(b))
                .then(a.cmp(&b))
        });
        for id in waiting {
            let Some(index) = self.vehicles.iter().position(|v| v.id == id) else {
                continue;
            };
            let route = self.vehicles[index].route;
            if self.manager.blocks(route, self.time) || self.elder_blocks(id, route) {
                continue;
            }
            if self.leader_blocks_entry(index) {
                continue;
            }
            let _ = self.manager.try_grant(id, route, self.time);
        }
    }

    fn ticket_key(&self, id: u64) -> f32 {
        self.manager
            .ticket_time(id)
            .or_else(|| {
                self.vehicles
                    .iter()
                    .find(|v| v.id == id)
                    .map(|v| v.spawn_time)
            })
            .unwrap_or(self.time)
    }

    fn ready_to_commit(&self, vehicle: &Vehicle) -> bool {
        let geom = &routes()[vehicle.route];
        let dist = geom.s_stop - vehicle.progress;
        if vehicle.progress >= geom.s_enter || dist > CONTROL_ZONE {
            return false;
        }
        let waited = vehicle.zone_enter.map(|t| self.time - t).unwrap_or(0.0);
        dist <= 140.0 || waited >= AGE_LIMIT
    }

    fn elder_blocks(&self, id: u64, route: usize) -> bool {
        let mine = self.ticket_key(id);
        for (other_id, other_route, other_time) in self.manager.waiting_tickets() {
            if other_id == id || !masks_conflict(route, other_route) {
                continue;
            }
            let older = other_time < mine || (other_time == mine && other_id < id);
            if !older {
                continue;
            }
            let Some(other) = self.vehicles.iter().find(|v| v.id == other_id) else {
                continue;
            };
            if self.ready_to_commit(other) {
                return true;
            }
        }
        false
    }

    fn leader_blocks_entry(&self, index: usize) -> bool {
        let Some(leader) = self.leader_index(index) else {
            return false;
        };
        let lead = &self.vehicles[leader];
        let clear = routes()[lead.route].s_clear;
        lead.progress < clear
    }

    fn leader_index(&self, index: usize) -> Option<usize> {
        let me = &self.vehicles[index];
        let mut best: Option<(f32, usize)> = None;
        for (j, other) in self.vehicles.iter().enumerate() {
            if j == index || other.route != me.route {
                continue;
            }
            let ahead = other.progress - me.progress;
            if ahead > 0.5 && best.map(|(dist, _)| ahead < dist).unwrap_or(true) {
                best = Some((ahead, j));
            }
        }
        best.map(|(_, j)| j)
    }

    fn compute_target(&self, index: usize, leader: Option<usize>) -> f32 {
        let vehicle = &self.vehicles[index];
        let geom = &routes()[vehicle.route];
        let mut target = vehicle.desired_speed;
        if let Some(leader_index) = leader {
            let lead = &self.vehicles[leader_index];
            let gap = lead.progress - vehicle.progress - VEHICLE_LENGTH;
            let available = gap - MIN_FOLLOWING_GAP - FOLLOW_CONTROL_BUFFER;
            if available <= 0.0 {
                target = 0.0;
            } else {
                target = target.min((2.0 * BRAKING * available).sqrt());
                if gap < MIN_FOLLOWING_GAP + FOLLOW_CONTROL_BUFFER + 18.0 {
                    target = target.min(lead.velocity);
                }
            }
        }

        let granted = self.manager.holds(vehicle.id);
        let inside = vehicle.progress >= geom.s_enter && vehicle.progress < geom.s_clear;
        if granted && inside {
            let gap_ok = match leader {
                None => true,
                Some(leader_index) => {
                    let gap =
                        self.vehicles[leader_index].progress - vehicle.progress - VEHICLE_LENGTH;
                    gap > MIN_FOLLOWING_GAP + 48.0
                }
            };
            if gap_ok {
                target = target.max(vehicle.desired_speed.min(CROSSING_MIN_SPEED));
            }
        } else if !granted && vehicle.progress < geom.s_enter {
            let dist = (geom.s_stop - vehicle.progress).max(0.0);
            if let Some(eta) = self.arrival_delay(vehicle) {
                // Schedule only on the final approach. Farther back, cruise and
                // use the stopping curve so a long queue does not crawl from the spawn.
                if eta > 0.05 && (1.0..180.0).contains(&dist) {
                    target = target.min((dist / eta).max(16.0));
                }
            }
            let stop_v = if dist <= 0.5 {
                0.0
            } else {
                (2.0 * BRAKING * dist).sqrt()
            };
            target = target.min(stop_v);
        }
        target.max(0.0)
    }

    fn arrival_delay(&self, vehicle: &Vehicle) -> Option<f32> {
        let mut best: Option<f32> = None;
        let mut push = |t: f32| {
            if t > 0.0 {
                best = Some(best.map(|b: f32| b.max(t)).unwrap_or(t));
            }
        };
        for (id, route) in self.manager.active_routes() {
            if id == vehicle.id || !masks_conflict(vehicle.route, route) {
                continue;
            }
            if let Some(holder) = self.vehicles.iter().find(|v| v.id == id) {
                let remaining = (routes()[holder.route].s_clear - holder.progress).max(0.0);
                let speed = holder.velocity.max(CROSSING_MIN_SPEED);
                push(remaining / speed + TIME_MARGIN);
            }
        }
        if let Some(cool) = self.manager.cooldown_remaining(vehicle.route, self.time) {
            push(cool);
        }
        if let Some(extra) = self.elder_eta(vehicle) {
            push(extra);
        }
        best
    }

    fn elder_eta(&self, vehicle: &Vehicle) -> Option<f32> {
        let mine = self.ticket_key(vehicle.id);
        let mut best: Option<f32> = None;
        for (other_id, other_route, other_time) in self.manager.waiting_tickets() {
            if other_id == vehicle.id || !masks_conflict(vehicle.route, other_route) {
                continue;
            }
            let older = other_time < mine || (other_time == mine && other_id < vehicle.id);
            if !older {
                continue;
            }
            let Some(other) = self.vehicles.iter().find(|v| v.id == other_id) else {
                continue;
            };
            if !self.ready_to_commit(other) && other.zone_enter.is_none() {
                continue;
            }
            let geom = &routes()[other.route];
            let dist = (geom.s_stop - other.progress).max(0.0);
            let speed = other.velocity.max(40.0);
            let cross = (geom.s_clear - geom.s_enter).max(0.0) / other.desired_speed.max(40.0);
            let eta = dist / speed + cross + TIME_MARGIN;
            best = Some(best.map(|b| b.max(eta)).unwrap_or(eta));
        }
        best
    }

    fn advance(&mut self, index: usize, target: f32, leader: Option<usize>, dt: f32) {
        let route_index = self.vehicles[index].route;
        let granted = self.manager.holds(self.vehicles[index].id);
        let s_stop = routes()[route_index].s_stop;
        let s_enter = routes()[route_index].s_enter;
        let before = self.vehicles[index].velocity;
        let mut velocity = before;
        if target > velocity {
            velocity = (velocity + ACCELERATION * dt).min(target);
        } else if target < velocity {
            velocity = (velocity - BRAKING * dt).max(target).max(0.0);
        }
        let s_old = self.vehicles[index].progress;
        let mut s_new = s_old + velocity * dt;
        if !granted && s_old < s_enter {
            if s_old <= s_stop {
                s_new = s_new.min(s_stop);
            } else {
                s_new = s_old;
            }
        }
        if let Some(leader_index) = leader {
            let max_s = self.vehicles[leader_index].progress - VEHICLE_LENGTH - MIN_FOLLOWING_GAP;
            if s_new > max_s {
                s_new = max_s.max(s_old);
            }
        }
        let pose_new = pose_on(&routes()[route_index], s_new);
        let blocked = self.vehicles.iter().enumerate().any(|(j, other)| {
            j != index
                && other.position.distance(pose_new.center) < 180.0
                && too_close(pose_new, other.pose)
        });
        let id = self.vehicles[index].id;
        if blocked {
            s_new = s_old;
            velocity = (before - EMERGENCY_BRAKING * dt).max(0.0);
            if self.yielding.insert(id) {
                self.stats.emergency_stops += 1;
            }
        } else {
            self.yielding.remove(&id);
        }
        if s_new < s_old {
            s_new = s_old;
        }
        let travelled = s_new - s_old;
        if dt > 0.0 && travelled + 0.05 < velocity * dt {
            velocity = travelled / dt;
        }
        let (position, tangent) = routes()[route_index].path.sample(s_new);
        let vehicle = &mut self.vehicles[index];
        vehicle.acceleration = (velocity - before) / dt.max(1e-4);
        vehicle.velocity = velocity.max(0.0);
        vehicle.progress = s_new;
        vehicle.distance_travelled += travelled;
        vehicle.position = position;
        vehicle.rotation = tangent.y.atan2(tangent.x);
        vehicle.pose = Pose::from_tangent(position, tangent);
        vehicle.min_velocity = vehicle.min_velocity.min(vehicle.velocity);
        vehicle.max_velocity = vehicle.max_velocity.max(vehicle.velocity);
        self.stats.observe_velocity(vehicle.velocity, dt);
    }

    fn refresh_state(&mut self, index: usize) {
        let route_index = self.vehicles[index].route;
        let geom = &routes()[route_index];
        let s_enter = geom.s_enter;
        let s_clear = geom.s_clear;
        let s_stop = geom.s_stop;
        let granted = self.manager.holds(self.vehicles[index].id);
        let vehicle = &mut self.vehicles[index];
        vehicle.state = if vehicle.progress >= s_clear {
            VehicleState::Exiting
        } else if granted && vehicle.progress >= s_enter {
            VehicleState::Crossing
        } else if granted {
            VehicleState::Reserved
        } else if vehicle.progress >= s_stop - 1.5 && vehicle.velocity < 8.0 {
            VehicleState::Waiting
        } else {
            VehicleState::Approaching
        };
        let near_stop = vehicle.progress >= s_stop - 8.0 && vehicle.progress < s_enter;
        if near_stop && vehicle.velocity < 6.0 && !granted {
            if vehicle.stopped_since.is_none() {
                vehicle.stopped_since = Some(self.time);
            }
            if let Some(started) = vehicle.stopped_since {
                let waited = self.time - started;
                self.stats.max_line_wait = self.stats.max_line_wait.max(waited);
            }
        } else {
            vehicle.stopped_since = None;
        }
    }

    fn measure_pairs(&mut self) {
        let mut inside = 0u32;
        for vehicle in &self.vehicles {
            if vehicle.state == VehicleState::Crossing {
                inside += 1;
            }
        }
        self.stats.max_in_intersection = self.stats.max_in_intersection.max(inside);

        let poses: Vec<(u64, Pose)> = self.vehicles.iter().map(|v| (v.id, v.pose)).collect();
        let mut seen_close = HashSet::new();
        let mut seen_hit = HashSet::new();
        for i in 0..poses.len() {
            for j in (i + 1)..poses.len() {
                let dist = poses[i].1.center.distance(poses[j].1.center);
                if dist > 280.0 {
                    continue;
                }
                let gap = crate::collision::edge_distance(poses[i].1, poses[j].1);
                self.stats.observe_separation(gap);
                let key = ordered_pair(poses[i].0, poses[j].0);
                if gap < COLLISION_DISTANCE {
                    seen_hit.insert(key);
                    if self.collision_pairs.insert(key) {
                        self.stats.collisions += 1;
                    }
                } else if gap < WARNING_DISTANCE {
                    seen_close.insert(key);
                    if self.close_pairs.insert(key) {
                        self.stats.close_calls += 1;
                    }
                }
            }
        }
        self.collision_pairs.retain(|key| seen_hit.contains(key));
        self.close_pairs
            .retain(|key| seen_close.contains(key) || seen_hit.contains(key));
    }

    fn record_queues(&mut self) {
        let mut lengths = [0u32; 12];
        for vehicle in &self.vehicles {
            if vehicle.progress < routes()[vehicle.route].s_enter {
                lengths[vehicle.route] += 1;
            }
        }
        for (lane, length) in lengths.iter().enumerate() {
            self.stats.observe_queue(lane, *length);
        }
    }

    fn retire_finished(&mut self) {
        let mut index = 0;
        while index < self.vehicles.len() {
            let route_index = self.vehicles[index].route;
            let length = routes()[route_index].path.length();
            if self.vehicles[index].progress + 0.2 >= length {
                let vehicle = self.vehicles.remove(index);
                self.manager.release(vehicle.id, self.time);
                self.manager.drop_ticket(vehicle.id);
                self.yielding.remove(&vehicle.id);
                let traversal = (self.time - vehicle.spawn_time).max(0.0);
                self.stats.record_completion(traversal);
                self.completed.push(CompletedTrip {
                    id: vehicle.id,
                    direction: vehicle.direction,
                    turn: vehicle.turn,
                    spawn_time: vehicle.spawn_time,
                    completion_time: self.time,
                    min_velocity: vehicle.min_velocity,
                    max_velocity: vehicle.max_velocity,
                    distance_travelled: vehicle.distance_travelled,
                });
            } else {
                index += 1;
            }
        }
    }
}

fn move_priority(vehicle: &Vehicle) -> u8 {
    match vehicle.state {
        VehicleState::Crossing => 3,
        VehicleState::Reserved | VehicleState::Exiting => 2,
        _ => 1,
    }
}

fn ordered_pair(a: u64, b: u64) -> (u64, u64) {
    if a < b {
        (a, b)
    } else {
        (b, a)
    }
}
