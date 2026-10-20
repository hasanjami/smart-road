//! Headless checks for routes, reservations, following distance, and longer runs.

use smart_road::config::{
    ACCELERATION, BRAKING, MIN_FOLLOWING_GAP, SPEED_FAST, SPEED_MEDIUM, SPEED_SLOW, TIME_MARGIN,
    VEHICLE_LENGTH, WINDOW,
};
use smart_road::intersection::IntersectionManager;
use smart_road::routes::{masks_conflict, route, route_index, routes, Direction, Turn};
use smart_road::simulation::Simulation;
use smart_road::statistics::{
    LABEL_CLOSE_CALLS, LABEL_MAX_TIME, LABEL_MAX_VELOCITY, LABEL_MIN_TIME, LABEL_MIN_VELOCITY,
    LABEL_PASSED,
};
use smart_road::vec2::angle_diff;
use smart_road::vehicle::SpeedLevel;

const DT: f32 = 1.0 / 60.0;

fn finish(sim: &mut Simulation, expected: u32, limit: f32) {
    let mut elapsed = 0.0;
    while elapsed < limit {
        sim.update(DT);
        elapsed += DT;
        if sim.statistics().completed >= expected && sim.active_count() == 0 {
            return;
        }
    }
    panic!(
        "timed out at {elapsed:.1}s with {} completed\n{}",
        sim.statistics().completed,
        sim.debug_dump()
    );
}

fn spawn_n(sim: &mut Simulation, direction: Direction, turn: Turn, count: u32, speed: SpeedLevel) {
    let mut spawned = 0;
    for _ in 0..12_000 {
        if spawned == count {
            return;
        }
        if sim.try_spawn(direction, turn, speed, spawned as u8).is_ok() {
            spawned += 1;
        }
        sim.update(DT);
    }
    panic!(
        "spawned {spawned}/{count} {:?}\n{}",
        (direction, turn),
        sim.debug_dump()
    );
}

fn assert_same_lane_gaps(sim: &Simulation) {
    let vehicles = sim.vehicles();
    for i in 0..vehicles.len() {
        for j in 0..vehicles.len() {
            if i == j || vehicles[i].route != vehicles[j].route {
                continue;
            }
            if vehicles[j].progress <= vehicles[i].progress {
                continue;
            }
            let gap = vehicles[j].progress - vehicles[i].progress - VEHICLE_LENGTH;
            assert!(
                gap + 0.05 >= MIN_FOLLOWING_GAP,
                "gap {gap:.2} between {} and {}\n{}",
                vehicles[i].id,
                vehicles[j].id,
                sim.debug_dump()
            );
        }
    }
}

fn assert_sane(text: &str) {
    let lower = text.to_ascii_lowercase();
    assert!(!lower.contains("nan"), "{text}");
    assert!(!lower.contains("inf"), "{text}");
}

fn run_pair(direction_a: Direction, direction_b: Direction, each: u32) {
    let mut sim = Simulation::new(8);
    let plan = [(direction_a, Turn::Straight), (direction_b, Turn::Straight)];
    let mut counts = [0u32; 2];
    for _ in 0..20_000 {
        if counts.iter().all(|count| *count >= each) {
            break;
        }
        for (index, (direction, turn)) in plan.iter().enumerate() {
            if counts[index] >= each {
                continue;
            }
            if sim
                .try_spawn(*direction, *turn, SpeedLevel::Medium, index as u8)
                .is_ok()
            {
                counts[index] += 1;
            }
        }
        sim.update(DT);
        assert_eq!(sim.statistics().collisions, 0, "{}", sim.debug_dump());
    }
    assert_eq!(counts[0], each, "{}", sim.debug_dump());
    assert_eq!(counts[1], each, "{}", sim.debug_dump());
    finish(&mut sim, each * 2, 90.0);
    assert_eq!(sim.statistics().completed, each * 2, "{}", sim.debug_dump());
    assert_eq!(sim.statistics().collisions, 0, "{}", sim.debug_dump());
}

#[test]
fn every_direction_spawns_on_its_approach() {
    let mut sim = Simulation::new(1);
    for direction in Direction::ALL {
        sim.try_spawn(direction, Turn::Straight, SpeedLevel::Medium, 0)
            .unwrap();
    }
    assert_eq!(sim.vehicles().len(), 4);
    for vehicle in sim.vehicles() {
        match vehicle.direction {
            Direction::South => {
                assert!(vehicle.position.y > 900.0);
                assert!(vehicle.position.x > 500.0);
            }
            Direction::North => {
                assert!(vehicle.position.y < 100.0);
                assert!(vehicle.position.x < 500.0);
            }
            Direction::West => {
                assert!(vehicle.position.x < 100.0);
                assert!(vehicle.position.y > 500.0);
            }
            Direction::East => {
                assert!(vehicle.position.x > 900.0);
                assert!(vehicle.position.y < 500.0);
            }
        }
    }
}

#[test]
fn all_twelve_routes_start_and_end_on_the_expected_sides() {
    assert_eq!(routes().len(), 12);
    for route in routes() {
        assert_ne!(route.mask, 0, "{}", route.name());
        assert!(route.s_stop < route.s_enter);
        assert!(route.s_enter < route.s_clear);
        assert!(route.s_clear < route.path.length());
        let (start, _) = route.path.sample(0.0);
        let (end, _) = route.path.sample(route.path.length());
        match route.direction {
            Direction::South => assert!(start.y > 900.0 && start.x > 500.0, "{start:?}"),
            Direction::North => assert!(start.y < 100.0 && start.x < 500.0, "{start:?}"),
            Direction::West => assert!(start.x < 100.0 && start.y > 500.0, "{start:?}"),
            Direction::East => assert!(start.x > 900.0 && start.y < 500.0, "{start:?}"),
        }
        match (route.direction, route.turn) {
            (Direction::South, Turn::Right) => assert!(end.x > WINDOW && end.y > 500.0),
            (Direction::South, Turn::Straight) => assert!(end.y < 0.0 && end.x > 500.0),
            (Direction::South, Turn::Left) => assert!(end.x < 0.0 && end.y < 500.0),
            (Direction::North, Turn::Right) => assert!(end.x < 0.0 && end.y < 500.0),
            (Direction::North, Turn::Straight) => assert!(end.y > WINDOW && end.x < 500.0),
            (Direction::North, Turn::Left) => assert!(end.x > WINDOW && end.y > 500.0),
            (Direction::West, Turn::Right) => assert!(end.y > WINDOW && end.x < 500.0),
            (Direction::West, Turn::Straight) => assert!(end.x > WINDOW && end.y > 500.0),
            (Direction::West, Turn::Left) => assert!(end.y < 0.0 && end.x > 500.0),
            (Direction::East, Turn::Right) => assert!(end.y < 0.0 && end.x > 500.0),
            (Direction::East, Turn::Straight) => assert!(end.x < 0.0 && end.y < 500.0),
            (Direction::East, Turn::Left) => assert!(end.y > WINDOW && end.x < 500.0),
        }
    }
}

#[test]
fn turns_change_heading_along_the_curve() {
    for direction in Direction::ALL {
        for turn in Turn::ALL {
            let route = route(direction, turn);
            let mut previous = None;
            let mut total = 0.0;
            let mut s = 0.0;
            while s <= route.path.length() {
                let (_, tangent) = route.path.sample(s);
                let angle = tangent.y.atan2(tangent.x);
                if let Some(before) = previous {
                    let step = angle_diff(angle, before);
                    assert!(
                        step.abs() < 0.45,
                        "{} heading jumped by {step}",
                        route.name()
                    );
                    total += step;
                }
                previous = Some(angle);
                s += 8.0;
            }
            if turn == Turn::Straight {
                assert!(total.abs() < 0.2, "{} total {total}", route.name());
            } else {
                assert!(
                    (total.abs() - std::f32::consts::FRAC_PI_2).abs() < 0.35,
                    "{} total {total}",
                    route.name()
                );
            }
        }
    }
}

#[test]
fn assigned_route_does_not_change() {
    let mut sim = Simulation::new(3);
    let id = sim
        .try_spawn(Direction::South, Turn::Left, SpeedLevel::Fast, 2)
        .unwrap();
    sim.run_for(3.0, DT);
    let vehicle = sim.vehicles().iter().find(|v| v.id == id).unwrap();
    assert_eq!(vehicle.direction, Direction::South);
    assert_eq!(vehicle.turn, Turn::Left);
    assert_eq!(vehicle.route, route_index(Direction::South, Turn::Left));
}

#[test]
fn conflict_masks_are_symmetric_and_cover_close_paths() {
    for left in 0..12 {
        for right in 0..12 {
            assert_eq!(masks_conflict(left, right), masks_conflict(right, left));
        }
    }
    let south = route_index(Direction::South, Turn::Straight);
    let north = route_index(Direction::North, Turn::Straight);
    let west = route_index(Direction::West, Turn::Straight);
    assert!(!masks_conflict(south, north));
    assert!(masks_conflict(south, west));
    assert!(!masks_conflict(
        route_index(Direction::South, Turn::Right),
        route_index(Direction::North, Turn::Right)
    ));
}

#[test]
fn close_paths_share_a_conflict_mask() {
    for left in 0..12 {
        for right in (left + 1)..12 {
            let gap = smart_road::routes::intersection_separation(left, right);
            if gap < 16.0 {
                assert!(
                    masks_conflict(left, right),
                    "{} and {} pass within {gap:.1} px without a shared cell",
                    routes()[left].name(),
                    routes()[right].name()
                );
            }
        }
    }
}

#[test]
fn conflicting_routes_cannot_hold_overlapping_reservations() {
    let mut manager = IntersectionManager::new();
    let south = route_index(Direction::South, Turn::Straight);
    let west = route_index(Direction::West, Turn::Straight);
    let north = route_index(Direction::North, Turn::Straight);
    assert!(manager.try_grant(1, south, 0.0));
    assert!(manager.blocks(west, 0.0));
    assert!(!manager.try_grant(2, west, 0.0));
    assert!(manager.try_grant(3, north, 0.0));
    manager.release(1, 0.0);
    // North straight crosses west straight, so west stays blocked until that lock ends.
    assert!(manager.blocks(west, 0.0));
    manager.release(3, 0.0);
    assert!(manager.blocks(west, 0.0));
    let open_at = TIME_MARGIN + 0.05;
    assert!(!manager.blocks(west, open_at));
    assert!(manager.try_grant(2, west, open_at));
}

#[test]
fn speed_levels_are_distinct_and_positive() {
    const {
        assert!(SPEED_SLOW > 0.0);
        assert!(SPEED_MEDIUM > SPEED_SLOW);
        assert!(SPEED_FAST > SPEED_MEDIUM);
        assert!(ACCELERATION > 0.0);
        assert!(BRAKING > ACCELERATION);
        assert!(MIN_FOLLOWING_GAP > 0.0);
        assert!(SpeedLevel::Fast.speed() > SpeedLevel::Medium.speed());
        assert!(SpeedLevel::Medium.speed() > SpeedLevel::Slow.speed());
    }
}

#[test]
fn acceleration_and_braking_are_gradual() {
    let mut sim = Simulation::new(1);
    let id = sim
        .try_spawn(Direction::South, Turn::Straight, SpeedLevel::Fast, 0)
        .unwrap();
    sim.update(0.2);
    let velocity = sim.vehicles()[0].velocity;
    assert!(velocity > 8.0 && velocity < SPEED_FAST * 0.5, "{velocity}");
    sim.run_for(0.8, DT);
    let before = sim.vehicles().iter().find(|v| v.id == id).unwrap().velocity;
    assert!(before > 40.0, "{before}");
    assert!(sim.set_desired_speed(id, 0.0));
    sim.update(0.1);
    let after = sim.vehicles().iter().find(|v| v.id == id).unwrap().velocity;
    assert!(after < before - 8.0, "before {before} after {after}");
    assert!(
        after > before - BRAKING * 0.1 - 2.0,
        "before {before} after {after}"
    );
    assert!(after > 0.0, "{after}");
}

#[test]
fn spawn_spam_does_not_overlap_a_lane() {
    let mut sim = Simulation::new(1);
    assert!(sim
        .try_spawn(Direction::East, Turn::Right, SpeedLevel::Slow, 0)
        .is_ok());
    assert!(sim
        .try_spawn(Direction::East, Turn::Right, SpeedLevel::Fast, 1)
        .is_err());
    assert!(sim
        .try_spawn(Direction::West, Turn::Left, SpeedLevel::Medium, 0)
        .is_ok());
    assert_eq!(sim.vehicles().len(), 2);
    let gap = crate::edge_gap(&sim);
    assert!(gap > MIN_FOLLOWING_GAP);
}

fn edge_gap(sim: &Simulation) -> f32 {
    let vehicles = sim.vehicles();
    smart_road::collision::edge_distance(vehicles[0].pose, vehicles[1].pose)
}

#[test]
fn same_lane_followers_keep_the_configured_gap() {
    let mut sim = Simulation::new(4);
    spawn_n(
        &mut sim,
        Direction::South,
        Turn::Straight,
        3,
        SpeedLevel::Fast,
    );
    assert_eq!(sim.statistics().spawned, 3);
    let mut elapsed = 0.0;
    while elapsed < 80.0 {
        sim.update(DT);
        elapsed += DT;
        assert_same_lane_gaps(&sim);
        assert_eq!(sim.statistics().collisions, 0, "{}", sim.debug_dump());
        if sim.statistics().completed == 3 && sim.active_count() == 0 {
            break;
        }
    }
    assert_eq!(sim.statistics().completed, 3, "{}", sim.debug_dump());
    assert_eq!(sim.active_count(), 0, "{}", sim.debug_dump());
}

#[test]
fn follower_stops_behind_a_slowing_leader() {
    let mut sim = Simulation::new(6);
    let leader = sim
        .try_spawn(Direction::North, Turn::Straight, SpeedLevel::Slow, 0)
        .unwrap();
    let mut follower = None;
    for _ in 0..600 {
        sim.update(DT);
        if follower.is_none()
            && sim
                .try_spawn(Direction::North, Turn::Straight, SpeedLevel::Fast, 1)
                .is_ok()
        {
            follower = Some(sim.vehicles().last().unwrap().id);
        }
        if follower.is_some()
            && sim
                .vehicles()
                .iter()
                .find(|v| Some(v.id) == follower)
                .map(|v| v.velocity > 35.0)
                .unwrap_or(false)
        {
            break;
        }
    }
    let follower = follower.expect("follower did not spawn");
    assert!(sim.set_desired_speed(leader, 0.0));
    for _ in 0..360 {
        sim.update(DT);
        assert_same_lane_gaps(&sim);
        assert_eq!(sim.statistics().collisions, 0, "{}", sim.debug_dump());
    }
    let lead = sim.vehicles().iter().find(|v| v.id == leader).unwrap();
    let follow = sim.vehicles().iter().find(|v| v.id == follower).unwrap();
    assert!(lead.velocity < 8.0, "{}", sim.debug_dump());
    assert!(
        follow.velocity < lead.velocity + 15.0,
        "{}",
        sim.debug_dump()
    );
}

#[test]
fn older_conflicting_vehicle_is_not_starved() {
    let mut sim = Simulation::new(5);
    let west = sim
        .try_spawn(Direction::West, Turn::Straight, SpeedLevel::Slow, 0)
        .unwrap();
    for _ in 0..8_000 {
        let _ = sim.try_spawn(Direction::South, Turn::Straight, SpeedLevel::Fast, 0);
        sim.update(1.0 / 30.0);
        assert_eq!(sim.statistics().collisions, 0, "{}", sim.debug_dump());
        if sim.completed_trips().iter().any(|trip| trip.id == west) {
            return;
        }
    }
    panic!("west vehicle was starved\n{}", sim.debug_dump());
}

#[test]
fn one_completed_vehicle_has_matching_traversal_times() {
    let mut sim = Simulation::new(2);
    sim.try_spawn(Direction::South, Turn::Right, SpeedLevel::Fast, 0)
        .unwrap();
    finish(&mut sim, 1, 40.0);
    let stats = sim.statistics();
    assert_eq!(stats.completed, 1);
    let min_time = stats.min_traversal.expect("min time");
    let max_time = stats.max_traversal.expect("max time");
    assert!((min_time - max_time).abs() < 1e-4);
    let trip = &sim.completed_trips()[0];
    let measured = trip.completion_time - trip.spawn_time;
    assert!((measured - min_time).abs() < 1e-4);
    assert_eq!(trip.direction, Direction::South);
    assert_eq!(trip.turn, Turn::Right);
}

#[test]
fn four_spawned_vehicles_count_as_four_completions() {
    let mut sim = Simulation::new(99);
    let plan = [
        Direction::South,
        Direction::South,
        Direction::West,
        Direction::West,
    ];
    let mut next = 0;
    for _ in 0..20_000 {
        if next < plan.len() && sim.spawn_from_direction(plan[next]).is_ok() {
            next += 1;
        }
        sim.update(1.0 / 30.0);
        if sim.statistics().completed == 4 && sim.active_count() == 0 {
            break;
        }
    }
    assert_eq!(next, 4, "{}", sim.debug_dump());
    assert_eq!(sim.statistics().completed, 4, "{}", sim.debug_dump());
    assert_eq!(sim.statistics().collisions, 0, "{}", sim.debug_dump());
    sim.freeze();
    sim.update(2.0);
    assert_eq!(sim.statistics().completed, 4);
}

#[test]
fn active_vehicles_are_not_counted_as_completed() {
    let mut sim = Simulation::new(1);
    sim.try_spawn(Direction::East, Turn::Straight, SpeedLevel::Slow, 0)
        .unwrap();
    sim.freeze();
    sim.update(10.0);
    assert_eq!(sim.statistics().completed, 0);
    assert_eq!(sim.active_count(), 1);
    let text = sim.statistics().summary(sim.active_count());
    assert!(text.contains("Vehicles still active: 1"));
    assert!(text.contains(&format!("{LABEL_PASSED}: 0")));
    assert_sane(&text);
}

#[test]
fn empty_statistics_do_not_use_nan_or_infinity() {
    let sim = Simulation::new(1);
    let text = sim.statistics().summary(0);
    assert_sane(&text);
    assert!(text.contains("N/A"));
    for label in [
        LABEL_PASSED,
        LABEL_MAX_VELOCITY,
        LABEL_MIN_VELOCITY,
        LABEL_MAX_TIME,
        LABEL_MIN_TIME,
        LABEL_CLOSE_CALLS,
    ] {
        assert!(text.contains(label), "missing {label}");
    }
}

#[test]
fn mixed_directions_finish_without_collisions() {
    run_pair(Direction::West, Direction::East, 3);
    run_pair(Direction::South, Direction::East, 3);
    run_pair(Direction::South, Direction::West, 3);
    run_pair(Direction::North, Direction::East, 3);
    run_pair(Direction::North, Direction::West, 3);
}

#[test]
fn five_south_and_two_west_finish_without_collisions() {
    let mut sim = Simulation::new(15);
    spawn_n(
        &mut sim,
        Direction::South,
        Turn::Straight,
        5,
        SpeedLevel::Medium,
    );
    spawn_n(
        &mut sim,
        Direction::West,
        Turn::Straight,
        2,
        SpeedLevel::Medium,
    );
    finish(&mut sim, 7, 100.0);
    assert_eq!(sim.statistics().completed, 7);
    assert_eq!(sim.statistics().collisions, 0);
}

#[test]
fn frame_steps_of_different_sizes_stay_safe() {
    for dt in [1.0 / 20.0, 1.0 / 60.0, 1.0 / 120.0] {
        let mut sim = Simulation::new(21);
        spawn_n(
            &mut sim,
            Direction::South,
            Turn::Straight,
            3,
            SpeedLevel::Medium,
        );
        let mut elapsed = 0.0;
        while elapsed < 80.0 {
            sim.update(dt);
            elapsed += dt;
            assert_same_lane_gaps(&sim);
            assert_eq!(sim.statistics().collisions, 0, "dt {dt}");
            if sim.statistics().completed == 3 && sim.active_count() == 0 {
                break;
            }
        }
        assert_eq!(
            sim.statistics().completed,
            3,
            "dt {dt}\n{}",
            sim.debug_dump()
        );
    }
}

#[test]
fn random_minute_stays_clear_and_uncongested() {
    let mut sim = Simulation::new(1_234);
    sim.set_random_enabled(true);
    let mut elapsed = 0.0;
    while elapsed < 60.0 {
        sim.update(1.0 / 30.0);
        elapsed += 1.0 / 30.0;
        assert_eq!(sim.statistics().collisions, 0, "{}", sim.debug_dump());
        assert!(
            sim.statistics().max_queue_overall < 8,
            "{}",
            sim.debug_dump()
        );
    }
    assert!(sim.statistics().completed >= 8, "{}", sim.debug_dump());
    let line_wait = sim.statistics().max_line_wait;
    assert!(
        line_wait < 45.0,
        "stop-line wait {line_wait:.1}s\n{}",
        sim.debug_dump()
    );
    assert_sane(&sim.statistics().summary(sim.active_count()));
}

#[test]
#[ignore = "five one-minute seeds; cargo test -- --ignored --test scenarios longer_random_seeds_stay_safe"]
fn longer_random_seeds_stay_safe() {
    for seed in [7_u64, 11, 19, 42, 99] {
        let mut sim = Simulation::new(seed);
        sim.set_random_enabled(true);
        sim.run_for(60.0, 1.0 / 30.0);
        assert_eq!(sim.statistics().collisions, 0, "seed {seed}");
        assert!(sim.statistics().max_queue_overall < 8, "seed {seed}");
        assert!(sim.statistics().completed >= 8, "seed {seed}");
    }
}

#[test]
fn bundled_textures_are_png_files() {
    for name in ["asphalt.png", "car.png", "arrow.png"] {
        let path = format!("{}/assets/{name}", env!("CARGO_MANIFEST_DIR"));
        let bytes = std::fs::read(&path).unwrap_or_else(|_| panic!("missing {path}"));
        assert!(bytes.len() > 32, "{name}");
        assert_eq!(
            &bytes[..8],
            &[137, 80, 78, 71, 13, 10, 26, 10],
            "{name} is not a png"
        );
    }
}
