//! Vehicle state and the numbers tracked for one trip.

use crate::collision::Pose;
use crate::config::{SPEED_FAST, SPEED_MEDIUM, SPEED_SLOW};
use crate::routes::{route_index, routes, Direction, Turn};
use crate::vec2::Vec2;

/// Three non-zero cruise speeds. The value is the desired speed, not a teleport.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SpeedLevel {
    Slow,
    Medium,
    Fast,
}

impl SpeedLevel {
    pub const ALL: [Self; 3] = [Self::Slow, Self::Medium, Self::Fast];

    pub fn from_index(index: u8) -> Self {
        match index % 3 {
            0 => Self::Slow,
            1 => Self::Medium,
            _ => Self::Fast,
        }
    }

    pub const fn speed(self) -> f32 {
        match self {
            Self::Slow => SPEED_SLOW,
            Self::Medium => SPEED_MEDIUM,
            Self::Fast => SPEED_FAST,
        }
    }
}

/// Where the vehicle is in the reservation lifecycle.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VehicleState {
    Approaching,
    Waiting,
    Reserved,
    Crossing,
    Exiting,
    Completed,
}

impl VehicleState {
    pub fn label(self) -> &'static str {
        match self {
            Self::Approaching => "approaching",
            Self::Waiting => "waiting",
            Self::Reserved => "reserved",
            Self::Crossing => "crossing",
            Self::Exiting => "exiting",
            Self::Completed => "completed",
        }
    }
}

/// One car. `direction` and `turn` stay fixed for the whole trip.
#[derive(Clone, Debug)]
pub struct Vehicle {
    pub id: u64,
    pub direction: Direction,
    pub turn: Turn,
    pub route: usize,
    pub speed_level: SpeedLevel,
    pub variant: u8,
    /// Arc length along the assigned route, in pixels.
    pub progress: f32,
    pub position: Vec2,
    /// Heading in radians, from the path tangent. Zero faces east.
    pub rotation: f32,
    pub distance_travelled: f32,
    pub velocity: f32,
    pub desired_speed: f32,
    pub min_velocity: f32,
    pub max_velocity: f32,
    /// Last applied acceleration. Negative values are braking.
    pub acceleration: f32,
    pub spawn_time: f32,
    pub completion_time: Option<f32>,
    pub state: VehicleState,
    /// Simulation time when the vehicle first entered the control zone.
    pub zone_enter: Option<f32>,
    /// Simulation time when the vehicle stopped at the line, if it is still there.
    pub stopped_since: Option<f32>,
    pub pose: Pose,
}

impl Vehicle {
    pub fn new(
        id: u64,
        direction: Direction,
        turn: Turn,
        speed_level: SpeedLevel,
        variant: u8,
        spawn_time: f32,
    ) -> Self {
        let index = route_index(direction, turn);
        let (position, tangent) = routes()[index].path.sample(0.0);
        Self {
            id,
            direction,
            turn,
            route: index,
            speed_level,
            variant,
            progress: 0.0,
            position,
            rotation: tangent.y.atan2(tangent.x),
            distance_travelled: 0.0,
            velocity: 0.0,
            desired_speed: speed_level.speed(),
            min_velocity: 0.0,
            max_velocity: 0.0,
            acceleration: 0.0,
            spawn_time,
            completion_time: None,
            state: VehicleState::Approaching,
            zone_enter: None,
            stopped_since: None,
            pose: Pose::from_tangent(position, tangent),
        }
    }
}

/// Compact record kept after a vehicle leaves the window.
#[derive(Clone, Debug)]
pub struct CompletedTrip {
    pub id: u64,
    pub direction: Direction,
    pub turn: Turn,
    pub spawn_time: f32,
    pub completion_time: f32,
    pub min_velocity: f32,
    pub max_velocity: f32,
    pub distance_travelled: f32,
}
