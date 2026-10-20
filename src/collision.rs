//! Oriented rectangle overlap and edge-to-edge separation.

use crate::config::{COLLISION_DISTANCE, VEHICLE_LENGTH, VEHICLE_WIDTH};
use crate::vec2::Vec2;

/// Vehicle body at one instant. The forward axis follows the path tangent.
#[derive(Clone, Copy, Debug)]
pub struct Pose {
    pub center: Vec2,
    pub forward: Vec2,
    pub right: Vec2,
}

impl Pose {
    pub fn from_tangent(center: Vec2, tangent: Vec2) -> Self {
        let forward = tangent.normalized();
        Self {
            center,
            forward,
            right: forward.perp(),
        }
    }

    pub fn corners(self, inflate: f32) -> [Vec2; 4] {
        let half_l = VEHICLE_LENGTH * 0.5 + inflate;
        let half_w = VEHICLE_WIDTH * 0.5 + inflate;
        let f = self.forward * half_l;
        let r = self.right * half_w;
        [
            self.center + f + r,
            self.center + f - r,
            self.center - f - r,
            self.center - f + r,
        ]
    }
}

/// True when the inflated bodies overlap. Inflation is applied on every side.
pub fn rectangles_overlap(a: Pose, b: Pose, inflate: f32) -> bool {
    let axes = [a.forward, a.right, b.forward, b.right];
    for axis in axes {
        if axis.length() <= 1e-6 {
            continue;
        }
        let (a0, a1) = project(a, axis, inflate);
        let (b0, b1) = project(b, axis, inflate);
        if a1 < b0 || b1 < a0 {
            return false;
        }
    }
    true
}

fn project(pose: Pose, axis: Vec2, inflate: f32) -> (f32, f32) {
    let mut min_p = f32::MAX;
    let mut max_p = f32::MIN;
    for corner in pose.corners(inflate) {
        let p = corner.dot(axis);
        min_p = min_p.min(p);
        max_p = max_p.max(p);
    }
    (min_p, max_p)
}

/// Euclidean gap between two bodies. Overlapping bodies report zero.
pub fn edge_distance(a: Pose, b: Pose) -> f32 {
    if rectangles_overlap(a, b, 0.0) {
        return 0.0;
    }
    let ca = a.corners(0.0);
    let cb = b.corners(0.0);
    let mut best = f32::MAX;
    for i in 0..4 {
        for j in 0..4 {
            let d = segment_distance(ca[i], ca[(i + 1) % 4], cb[j], cb[(j + 1) % 4]);
            best = best.min(d);
        }
    }
    best
}

/// Bodies are not allowed to come closer than the configured collision distance.
pub fn too_close(a: Pose, b: Pose) -> bool {
    edge_distance(a, b) < COLLISION_DISTANCE
}

fn segment_distance(p1: Vec2, q1: Vec2, p2: Vec2, q2: Vec2) -> f32 {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);
    let eps = 1e-6;
    let (s, t) = if a <= eps && e <= eps {
        (0.0, 0.0)
    } else if a <= eps {
        (0.0, (f / e).clamp(0.0, 1.0))
    } else {
        let c = d1.dot(r);
        if e <= eps {
            ((-c / a).clamp(0.0, 1.0), 0.0)
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let mut s = if denom.abs() > eps {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let mut t = (b * s + f) / e;
            if t < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            }
            (s, t)
        }
    };
    let c1 = p1 + d1 * s;
    let c2 = p2 + d2 * t;
    (c1 - c2).length()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axis_pose(x: f32, y: f32) -> Pose {
        Pose::from_tangent(Vec2::new(x, y), Vec2::new(1.0, 0.0))
    }

    #[test]
    fn separated_rectangles_do_not_overlap() {
        let a = axis_pose(0.0, 0.0);
        let b = axis_pose(80.0, 0.0);
        assert!(!rectangles_overlap(a, b, 0.0));
        assert!(edge_distance(a, b) > 30.0);
    }

    #[test]
    fn overlapping_rectangles_report_zero_gap() {
        let a = axis_pose(0.0, 0.0);
        let b = axis_pose(10.0, 0.0);
        assert!(rectangles_overlap(a, b, 0.0));
        assert_eq!(edge_distance(a, b), 0.0);
    }

    #[test]
    fn rotated_rectangle_detects_overlap() {
        let a = axis_pose(0.0, 0.0);
        let b = Pose::from_tangent(Vec2::new(20.0, 0.0), Vec2::new(0.0, 1.0));
        assert!(rectangles_overlap(a, b, 0.0));
    }
}
