//! Trip measurements and the end-of-run summary.
//!
//! Times are seconds of simulation time. Speeds are pixels per second, also
//! shown in km/h using the scale in `config`. Missing samples are `N/A`.

use crate::config::{px_to_kmh, WINDOW};
use crate::routes::{Direction, Turn};

pub const LABEL_PASSED: &str = "Max number of vehicles that passed the intersection";
pub const LABEL_MAX_VELOCITY: &str = "Max velocity";
pub const LABEL_MIN_VELOCITY: &str = "Min velocity";
pub const LABEL_MAX_TIME: &str = "Max time that took a vehicle to pass the intersection";
pub const LABEL_MIN_TIME: &str = "Min time that took a vehicle to pass the intersection";
pub const LABEL_CLOSE_CALLS: &str = "Close calls";

#[derive(Clone, Debug)]
pub struct Statistics {
    pub spawned: u32,
    pub completed: u32,
    pub rejected_spawns: u32,
    pub deferred_spawns: u32,
    pub collisions: u32,
    pub close_calls: u32,
    pub emergency_stops: u32,
    pub min_velocity: Option<f32>,
    pub max_velocity: Option<f32>,
    pub min_moving_velocity: Option<f32>,
    pub moving_speed_integral: f32,
    pub moving_time: f32,
    pub min_traversal: Option<f32>,
    pub max_traversal: Option<f32>,
    pub traversal_sum: f32,
    pub max_in_intersection: u32,
    pub max_queue_overall: u32,
    pub max_queue_per_lane: [u32; 12],
    pub min_separation: Option<f32>,
    pub max_line_wait: f32,
    pub simulation_time: f32,
}

impl Default for Statistics {
    fn default() -> Self {
        Self {
            spawned: 0,
            completed: 0,
            rejected_spawns: 0,
            deferred_spawns: 0,
            collisions: 0,
            close_calls: 0,
            emergency_stops: 0,
            min_velocity: None,
            max_velocity: None,
            min_moving_velocity: None,
            moving_speed_integral: 0.0,
            moving_time: 0.0,
            min_traversal: None,
            max_traversal: None,
            traversal_sum: 0.0,
            max_in_intersection: 0,
            max_queue_overall: 0,
            max_queue_per_lane: [0; 12],
            min_separation: None,
            max_line_wait: 0.0,
            simulation_time: 0.0,
        }
    }
}

impl Statistics {
    pub fn observe_velocity(&mut self, velocity: f32, dt: f32) {
        if !velocity.is_finite() {
            return;
        }
        self.min_velocity = Some(
            self.min_velocity
                .map(|v| v.min(velocity))
                .unwrap_or(velocity),
        );
        self.max_velocity = Some(
            self.max_velocity
                .map(|v| v.max(velocity))
                .unwrap_or(velocity),
        );
        if velocity > 1.0 {
            self.min_moving_velocity = Some(
                self.min_moving_velocity
                    .map(|v| v.min(velocity))
                    .unwrap_or(velocity),
            );
            self.moving_speed_integral += velocity * dt;
            self.moving_time += dt;
        }
    }

    pub fn record_completion(&mut self, traversal: f32) {
        if !traversal.is_finite() || traversal < 0.0 {
            return;
        }
        self.completed += 1;
        self.traversal_sum += traversal;
        self.min_traversal = Some(
            self.min_traversal
                .map(|v| v.min(traversal))
                .unwrap_or(traversal),
        );
        self.max_traversal = Some(
            self.max_traversal
                .map(|v| v.max(traversal))
                .unwrap_or(traversal),
        );
    }

    pub fn observe_separation(&mut self, gap: f32) {
        if gap.is_finite() {
            self.min_separation = Some(self.min_separation.map(|v| v.min(gap)).unwrap_or(gap));
        }
    }

    pub fn observe_queue(&mut self, lane: usize, length: u32) {
        if lane < 12 {
            self.max_queue_per_lane[lane] = self.max_queue_per_lane[lane].max(length);
        }
        self.max_queue_overall = self.max_queue_overall.max(length);
    }

    pub fn average_traversal(&self) -> Option<f32> {
        if self.completed == 0 {
            None
        } else {
            Some(self.traversal_sum / self.completed as f32)
        }
    }

    pub fn average_moving_velocity(&self) -> Option<f32> {
        if self.moving_time <= 0.0 {
            None
        } else {
            Some(self.moving_speed_integral / self.moving_time)
        }
    }

    pub fn summary(&self, active: u32) -> String {
        let mut lines = Vec::new();
        lines.push("Simulation statistics".to_string());
        lines.push(format!("{LABEL_PASSED}: {}", self.completed));
        lines.push(format!(
            "{LABEL_MAX_VELOCITY}: {}",
            fmt_speed(self.max_velocity)
        ));
        lines.push(format!(
            "{LABEL_MIN_VELOCITY}: {}",
            fmt_speed(self.min_velocity)
        ));
        lines.push(format!(
            "Min moving velocity: {}",
            fmt_speed(self.min_moving_velocity)
        ));
        lines.push(format!(
            "{LABEL_MAX_TIME}: {}",
            fmt_time(self.max_traversal)
        ));
        lines.push(format!(
            "{LABEL_MIN_TIME}: {}",
            fmt_time(self.min_traversal)
        ));
        lines.push(format!("{LABEL_CLOSE_CALLS}: {}", self.close_calls));
        lines.push(format!("Total vehicles spawned: {}", self.spawned));
        lines.push(format!("Vehicles still active: {active}"));
        lines.push(format!("Actual collisions: {}", self.collisions));
        lines.push(format!(
            "Average traversal time: {}",
            fmt_time(self.average_traversal())
        ));
        lines.push(format!(
            "Average moving velocity: {}",
            fmt_speed(self.average_moving_velocity())
        ));
        lines.push(format!(
            "Maximum simultaneous vehicles inside the intersection: {}",
            self.max_in_intersection
        ));
        lines.push(format!(
            "Maximum queue length overall: {}",
            self.max_queue_overall
        ));
        for direction in Direction::ALL {
            for turn in Turn::ALL {
                let index = crate::routes::route_index(direction, turn);
                lines.push(format!(
                    "Maximum queue length {} {}: {}",
                    direction.label(),
                    turn.label(),
                    self.max_queue_per_lane[index]
                ));
            }
        }
        lines.push(format!(
            "Minimum recorded separation: {}",
            fmt_distance(self.min_separation)
        ));
        lines.push(format!(
            "Rejected or deferred spawn attempts: {}",
            self.rejected_spawns + self.deferred_spawns
        ));
        lines.push(format!(
            "Simulation duration: {}",
            fmt_time(Some(self.simulation_time))
        ));
        lines.push(format!("Emergency stops: {}", self.emergency_stops));
        lines.push(format!(
            "Maximum wait at the stop line: {}",
            fmt_time(Some(self.max_line_wait))
        ));
        lines.push(String::new());
        lines.push(
            "Min velocity is the lowest recorded speed and includes zero when a vehicle is stopped."
                .to_string(),
        );
        lines.push("Min moving velocity is the lowest strictly positive speed.".to_string());
        lines.push(format!(
            "Speeds use pixels and seconds. 1 px = 0.1 m on a {WINDOW:.0} px window."
        ));
        lines.join("\n")
    }
}

pub fn fmt_speed(value: Option<f32>) -> String {
    match value {
        Some(v) if v.is_finite() => format!("{v:.1} px/s ({:.1} km/h)", px_to_kmh(v)),
        _ => "N/A".to_string(),
    }
}

pub fn fmt_time(value: Option<f32>) -> String {
    match value {
        Some(v) if v.is_finite() => format!("{v:.2} s"),
        _ => "N/A".to_string(),
    }
}

pub fn fmt_distance(value: Option<f32>) -> String {
    match value {
        Some(v) if v.is_finite() => format!("{v:.1} px"),
        _ => "N/A".to_string(),
    }
}
