//! Simulation constants.
//!
//! # Units
//!
//! The world is measured in pixels and seconds.
//!
//! * Distance advances as `velocity * dt`.
//! * Velocity changes as `acceleration * dt`.
//! * One pixel represents 0.1 m, so `km/h = px/s * 0.36`.
//!
//! A vehicle that never leaves rest has a minimum recorded speed of zero.
//! `Min moving velocity` ignores those stationary samples.

/// Window and world size in pixels. Geometry is defined in this square.
pub const WINDOW: f32 = 1000.0;

/// Shared center of the cross intersection.
pub const CENTER: f32 = 500.0;

/// Width of one lane, in pixels.
pub const LANE_WIDTH: f32 = 48.0;

/// Half-width of a road that carries three lanes in each direction.
pub const ROAD_HALF: f32 = LANE_WIDTH * 3.0;

/// Western/northern edge of the paved intersection square.
pub const ROAD_MIN: f32 = CENTER - ROAD_HALF;

/// Eastern/southern edge of the paved intersection square.
pub const ROAD_MAX: f32 = CENTER + ROAD_HALF;

/// Conflict grid resolution. Six cells cover one road width.
pub const GRID_SIZE: i32 = 6;

/// Width of one conflict cell, equal to one lane.
pub const CELL_SIZE: f32 = (ROAD_MAX - ROAD_MIN) / GRID_SIZE as f32;

/// How far a vehicle center sits inside the window when it appears.
pub const SPAWN_INSET: f32 = 42.0;

/// How far past the window a path continues so the whole body can leave.
pub const EXIT_MARGIN: f32 = 90.0;

/// Radius of a right-turn arc, in pixels.
pub const RIGHT_RADIUS: f32 = 76.0;

/// Radius of a left-turn arc, in pixels.
pub const LEFT_RADIUS: f32 = 176.0;

/// Vehicle length along its heading, in pixels.
pub const VEHICLE_LENGTH: f32 = 36.0;

/// Vehicle width across its heading, in pixels.
pub const VEHICLE_WIDTH: f32 = 18.0;

/// Minimum bumper-to-bumper gap between vehicles on the same route.
pub const MIN_FOLLOWING_GAP: f32 = 32.0;

/// Extra gap the speed controller aims for so the hard limit is not ridden.
pub const FOLLOW_CONTROL_BUFFER: f32 = 10.0;

/// Edge-to-edge gap below this, but still above the collision distance, is a close call.
pub const WARNING_DISTANCE: f32 = 14.0;

/// Edge-to-edge gap below this counts as a collision. Bodies are kept farther apart.
pub const COLLISION_DISTANCE: f32 = 2.0;

/// Slow desired speed in pixels per second.
pub const SPEED_SLOW: f32 = 62.0;

/// Medium desired speed in pixels per second.
pub const SPEED_MEDIUM: f32 = 102.0;

/// Fast desired speed in pixels per second.
pub const SPEED_FAST: f32 = 142.0;

/// Comfortable acceleration in pixels per second squared.
pub const ACCELERATION: f32 = 72.0;

/// Comfortable braking in pixels per second squared.
pub const BRAKING: f32 = 150.0;

/// Braking used when a projected step would enter the collision distance.
pub const EMERGENCY_BRAKING: f32 = 420.0;

/// Speed floor used inside the intersection when the route ahead is clear.
pub const CROSSING_MIN_SPEED: f32 = 48.0;

/// Distance before the stop line at which a vehicle may be granted a reservation.
pub const CONTROL_ZONE: f32 = 260.0;

/// A waiting vehicle older than this blocks conflicting grants.
pub const AGE_LIMIT: f32 = 3.5;

/// Seconds that must elapse after a conflicting vehicle clears before the next grant.
pub const TIME_MARGIN: f32 = 0.30;

/// Random spawning interval while generation is enabled.
pub const RANDOM_PERIOD: f32 = 0.48;

/// How long a blocked spawn request stays queued for one lane.
pub const DEFER_LIMIT: f32 = 1.5;

/// Random generation skips a lane that already has this many vehicles before the box.
pub const SATURATED_QUEUE: u32 = 4;

/// Largest physics step. Bigger frames are split so motion stays stable.
pub const MAX_SUBSTEP: f32 = 1.0 / 60.0;

/// One pixel represents this many meters.
pub const METERS_PER_PIXEL: f32 = 0.1;

/// Multiply pixels per second by this value to get kilometers per hour.
pub const PX_S_TO_KMH: f32 = METERS_PER_PIXEL * 3.6;

/// Stop the center this far before the body would touch the conflict square.
pub const STOP_BUFFER: f32 = 10.0;

/// Extra travel after the body leaves the square before the reservation drops.
pub const CLEAR_BUFFER: f32 = 16.0;

pub fn px_to_kmh(px_per_sec: f32) -> f32 {
    px_per_sec * PX_S_TO_KMH
}
