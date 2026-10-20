//! Smart intersection simulation.
//!
//! The library is headless: it advances vehicles, reservations, and statistics
//! without opening a window. The graphical shell lives in the binary.

#![forbid(unsafe_code)]

pub mod collision;
pub mod config;
pub mod intersection;
pub mod routes;
pub mod simulation;
pub mod statistics;
pub mod vec2;
pub mod vehicle;
