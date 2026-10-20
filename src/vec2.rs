//! Small 2D vector used by the simulation. Rendering converts it to the
//! graphics crate's vector type at the boundary.

use std::ops::{Add, Mul, Sub};

/// Point or free vector in screen space. X grows east and Y grows south.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    pub fn length(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub fn normalized(self) -> Self {
        let len = self.length();
        if len <= 1e-6 {
            Self::new(1.0, 0.0)
        } else {
            self * (1.0 / len)
        }
    }

    pub fn perp(self) -> Self {
        Self::new(-self.y, self.x)
    }

    pub fn lerp(self, other: Self, t: f32) -> Self {
        self + (other - self) * t
    }

    pub fn distance(self, other: Self) -> f32 {
        (other - self).length()
    }
}

impl Add for Vec2 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Vec2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

/// Signed smallest difference `a - b`, wrapped into (-pi, pi].
pub fn angle_diff(a: f32, b: f32) -> f32 {
    let mut d = a - b;
    let pi = std::f32::consts::PI;
    while d > pi {
        d -= pi * 2.0;
    }
    while d <= -pi {
        d += pi * 2.0;
    }
    d
}
