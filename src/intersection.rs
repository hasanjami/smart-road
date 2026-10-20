//! Reservation manager for the conflict grid.
//!
//! A vehicle receives a cell mask only when that mask does not overlap an
//! active reservation or a recent cooldown. Conflicting movements are therefore
//! serialized. Movements with disjoint masks may cross together. The caller
//! applies FIFO aging on top of these spatial locks.

use crate::config::TIME_MARGIN;
use crate::routes::{masks_conflict, routes};

#[derive(Clone, Debug)]
struct ActiveReservation {
    id: u64,
    route: usize,
    mask: u64,
}

#[derive(Clone, Debug)]
struct Ticket {
    id: u64,
    route: usize,
    time: f32,
}

#[derive(Clone, Debug)]
struct Cooldown {
    mask: u64,
    until: f32,
}

/// Spatial locks and the order in which vehicles asked to cross.
#[derive(Clone, Debug, Default)]
pub struct IntersectionManager {
    active: Vec<ActiveReservation>,
    tickets: Vec<Ticket>,
    cooldowns: Vec<Cooldown>,
}

impl IntersectionManager {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn ensure_ticket(&mut self, id: u64, route: usize, time: f32) {
        if self.tickets.iter().any(|t| t.id == id) || self.holds(id) {
            return;
        }
        self.tickets.push(Ticket { id, route, time });
    }

    pub fn ticket_time(&self, id: u64) -> Option<f32> {
        self.tickets.iter().find(|t| t.id == id).map(|t| t.time)
    }

    pub fn holds(&self, id: u64) -> bool {
        self.active.iter().any(|r| r.id == id)
    }

    pub fn held_route(&self, id: u64) -> Option<usize> {
        self.active.iter().find(|r| r.id == id).map(|r| r.route)
    }

    /// True when `route` overlaps an active lock or a cooldown that is still running.
    pub fn blocks(&self, route: usize, now: f32) -> bool {
        let mask = routes()[route].mask;
        self.active.iter().any(|r| r.mask & mask != 0)
            || self
                .cooldowns
                .iter()
                .any(|c| now < c.until && c.mask & mask != 0)
    }

    /// Grant `route` to `id` when the mask is free. Returns false if it conflicts.
    pub fn try_grant(&mut self, id: u64, route: usize, now: f32) -> bool {
        if self.holds(id) {
            return true;
        }
        if self.blocks(route, now) {
            return false;
        }
        let mask = routes()[route].mask;
        self.active.push(ActiveReservation { id, route, mask });
        self.tickets.retain(|t| t.id != id);
        true
    }

    pub fn release(&mut self, id: u64, now: f32) {
        if let Some(pos) = self.active.iter().position(|r| r.id == id) {
            let reservation = self.active.remove(pos);
            self.cooldowns.push(Cooldown {
                mask: reservation.mask,
                until: now + TIME_MARGIN,
            });
        }
        self.tickets.retain(|t| t.id != id);
    }

    pub fn drop_ticket(&mut self, id: u64) {
        self.tickets.retain(|t| t.id != id);
    }

    pub fn prune(&mut self, now: f32) {
        self.cooldowns.retain(|c| now < c.until);
    }

    /// Longest remaining cooldown that overlaps `route`, if any.
    pub fn cooldown_remaining(&self, route: usize, now: f32) -> Option<f32> {
        let mask = routes()[route].mask;
        let mut best: Option<f32> = None;
        for cooldown in &self.cooldowns {
            if cooldown.until > now && cooldown.mask & mask != 0 {
                let left = cooldown.until - now;
                best = Some(best.map(|value| value.max(left)).unwrap_or(left));
            }
        }
        best
    }

    pub fn waiting_tickets(&self) -> impl Iterator<Item = (u64, usize, f32)> + '_ {
        self.tickets.iter().map(|t| (t.id, t.route, t.time))
    }

    pub fn active_routes(&self) -> impl Iterator<Item = (u64, usize)> + '_ {
        self.active.iter().map(|r| (r.id, r.route))
    }
}

/// Convenience used by tests that only care about the static conflict relation.
pub fn reservations_conflict(a: usize, b: usize) -> bool {
    masks_conflict(a, b)
}
