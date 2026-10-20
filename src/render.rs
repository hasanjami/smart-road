//! Intersection, vehicles, live counters, and the statistics screen.

use macroquad::prelude::*;

use smart_road::config::{ROAD_MAX, ROAD_MIN, VEHICLE_LENGTH, VEHICLE_WIDTH, WINDOW};
use smart_road::routes::{routes, Turn};
use smart_road::simulation::Simulation;
use smart_road::vec2::Vec2 as SimVec;

pub struct Assets {
    asphalt: Option<Texture2D>,
    car: Option<Texture2D>,
    arrow: Option<Texture2D>,
}

impl Assets {
    pub fn load() -> Self {
        Self {
            asphalt: load_png(include_bytes!("../assets/asphalt.png")),
            car: load_png(include_bytes!("../assets/car.png")),
            arrow: load_png(include_bytes!("../assets/arrow.png")),
        }
    }
}

fn load_png(bytes: &[u8]) -> Option<Texture2D> {
    let image = image::load_from_memory(bytes).ok()?;
    let rgba = image.to_rgba8();
    let (width, height) = rgba.dimensions();
    if width == 0 || height == 0 || width > u16::MAX as u32 || height > u16::MAX as u32 {
        return None;
    }
    let texture = Texture2D::from_rgba8(width as u16, height as u16, &rgba);
    texture.set_filter(FilterMode::Linear);
    Some(texture)
}

pub fn draw_world(sim: &Simulation, assets: &Assets) {
    clear_background(Color::from_rgba(28, 84, 52, 255));
    draw_roads(assets);
    draw_lane_markings();
    draw_route_guides();
    draw_lane_arrows(assets);
    draw_stop_bars();
    draw_cardinal_labels();
    for vehicle in sim.vehicles() {
        draw_vehicle(vehicle, assets);
    }
    draw_hud(sim);
}

pub fn draw_statistics(sim: &Simulation) {
    clear_background(Color::from_rgba(16, 22, 32, 255));
    let text = sim.statistics().summary(sim.active_count());
    let mut y = 36.0;
    for line in text.lines() {
        let size = if y < 50.0 { 28.0 } else { 20.0 };
        let color = if line.is_empty() {
            y += 8.0;
            continue;
        } else {
            WHITE
        };
        draw_text(line, 36.0, y, size, color);
        y += if size > 24.0 { 36.0 } else { 24.0 };
    }
    draw_text(
        "Press Enter, Space, or Esc to exit",
        36.0,
        WINDOW - 36.0,
        22.0,
        Color::from_rgba(255, 214, 120, 255),
    );
}

fn draw_roads(assets: &Assets) {
    let road = Color::from_rgba(52, 54, 60, 255);
    draw_rectangle(ROAD_MIN, 0.0, ROAD_MAX - ROAD_MIN, WINDOW, road);
    draw_rectangle(0.0, ROAD_MIN, WINDOW, ROAD_MAX - ROAD_MIN, road);
    if let Some(texture) = &assets.asphalt {
        tile_texture(texture, ROAD_MIN, 0.0, ROAD_MAX - ROAD_MIN, WINDOW);
        tile_texture(texture, 0.0, ROAD_MIN, WINDOW, ROAD_MAX - ROAD_MIN);
    }
    let span = ROAD_MAX - ROAD_MIN;
    draw_rectangle(
        ROAD_MIN,
        ROAD_MIN,
        span,
        span,
        Color::from_rgba(255, 255, 255, 18),
    );
    draw_rectangle_lines(
        ROAD_MIN,
        ROAD_MIN,
        span,
        span,
        3.0,
        Color::from_rgba(230, 230, 236, 180),
    );
    draw_circle_lines(
        WINDOW * 0.5,
        WINDOW * 0.5,
        28.0,
        2.0,
        Color::from_rgba(230, 210, 120, 200),
    );
}

fn tile_texture(texture: &Texture2D, x: f32, y: f32, w: f32, h: f32) {
    let tile: f32 = 64.0;
    let mut yy = y;
    while yy < y + h - 0.5 {
        let mut xx = x;
        let row_h = tile.min(y + h - yy);
        while xx < x + w - 0.5 {
            let col_w = tile.min(x + w - xx);
            draw_texture_ex(
                texture,
                xx,
                yy,
                WHITE,
                DrawTextureParams {
                    dest_size: Some(vec2(col_w, row_h)),
                    source: Some(Rect::new(0.0, 0.0, col_w, row_h)),
                    ..Default::default()
                },
            );
            xx += tile;
        }
        yy += tile;
    }
}

fn draw_lane_markings() {
    let white = Color::from_rgba(236, 236, 236, 210);
    let yellow = Color::from_rgba(232, 196, 64, 230);
    for i in 1..6 {
        let offset = ROAD_MIN + i as f32 * (ROAD_MAX - ROAD_MIN) / 6.0;
        let color = if i == 3 { yellow } else { white };
        let width = if i == 3 { 3.0 } else { 1.5 };
        if i == 3 {
            dashed(
                offset - 3.0,
                8.0,
                offset - 3.0,
                ROAD_MIN - 6.0,
                width,
                color,
            );
            dashed(
                offset + 3.0,
                8.0,
                offset + 3.0,
                ROAD_MIN - 6.0,
                width,
                color,
            );
            dashed(
                offset - 3.0,
                ROAD_MAX + 6.0,
                offset - 3.0,
                WINDOW - 8.0,
                width,
                color,
            );
            dashed(
                offset + 3.0,
                ROAD_MAX + 6.0,
                offset + 3.0,
                WINDOW - 8.0,
                width,
                color,
            );
            dashed(
                8.0,
                offset - 3.0,
                ROAD_MIN - 6.0,
                offset - 3.0,
                width,
                color,
            );
            dashed(
                8.0,
                offset + 3.0,
                ROAD_MIN - 6.0,
                offset + 3.0,
                width,
                color,
            );
            dashed(
                ROAD_MAX + 6.0,
                offset - 3.0,
                WINDOW - 8.0,
                offset - 3.0,
                width,
                color,
            );
            dashed(
                ROAD_MAX + 6.0,
                offset + 3.0,
                WINDOW - 8.0,
                offset + 3.0,
                width,
                color,
            );
        } else {
            dashed(offset, 8.0, offset, ROAD_MIN - 6.0, width, color);
            dashed(offset, ROAD_MAX + 6.0, offset, WINDOW - 8.0, width, color);
            dashed(8.0, offset, ROAD_MIN - 6.0, offset, width, color);
            dashed(ROAD_MAX + 6.0, offset, WINDOW - 8.0, offset, width, color);
        }
    }
}

fn dashed(x0: f32, y0: f32, x1: f32, y1: f32, width: f32, color: Color) {
    let dx = x1 - x0;
    let dy = y1 - y0;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1.0 {
        return;
    }
    let ux = dx / len;
    let uy = dy / len;
    let mut travelled = 0.0;
    while travelled < len {
        let dash = 16.0_f32.min(len - travelled);
        draw_line(
            x0 + ux * travelled,
            y0 + uy * travelled,
            x0 + ux * (travelled + dash),
            y0 + uy * (travelled + dash),
            width,
            color,
        );
        travelled += 30.0;
    }
}

fn draw_route_guides() {
    for route in routes() {
        let color = with_alpha(turn_color(route.turn, 0), 0.55);
        let mut s = route.s_enter;
        let (mut previous, _) = route.path.sample(s);
        s += 12.0;
        while s <= route.s_clear {
            let (point, _) = route.path.sample(s);
            draw_line(previous.x, previous.y, point.x, point.y, 2.5, color);
            previous = point;
            s += 12.0;
        }
    }
}

fn draw_lane_arrows(assets: &Assets) {
    for route in routes() {
        let s = (route.s_stop * 0.42).clamp(30.0, route.s_stop - 20.0);
        let (position, tangent) = route.path.sample(s);
        draw_arrow(assets, position, tangent, turn_color(route.turn, 0));
    }
}

fn draw_arrow(assets: &Assets, position: SimVec, tangent: SimVec, color: Color) {
    let angle = tangent.y.atan2(tangent.x);
    if let Some(texture) = &assets.arrow {
        let size = 26.0;
        draw_texture_ex(
            texture,
            position.x - size * 0.5,
            position.y - size * 0.5,
            color,
            DrawTextureParams {
                dest_size: Some(vec2(size, size)),
                rotation: angle,
                pivot: Some(vec2(position.x, position.y)),
                ..Default::default()
            },
        );
    } else {
        let nose = vec2(position.x, position.y) + vec2(tangent.x, tangent.y) * 12.0;
        let side = vec2(-tangent.y, tangent.x) * 7.0;
        let back = vec2(position.x, position.y) - vec2(tangent.x, tangent.y) * 8.0;
        draw_triangle(nose, back + side, back - side, color);
    }
}

fn draw_stop_bars() {
    let color = Color::from_rgba(250, 250, 250, 230);
    for route in routes() {
        let (position, tangent) = route.path.sample(route.s_stop);
        let side = SimVec::new(-tangent.y, tangent.x) * 14.0;
        draw_line(
            position.x - side.x,
            position.y - side.y,
            position.x + side.x,
            position.y + side.y,
            3.0,
            color,
        );
    }
}

fn draw_cardinal_labels() {
    let color = Color::from_rgba(240, 255, 230, 255);
    text("North", 662.0, 48.0, 28.0, color);
    text("South", 662.0, 980.0, 28.0, color);
    text("West", 28.0, 340.0, 28.0, color);
    text("East", 900.0, 340.0, 28.0, color);
}

fn draw_vehicle(vehicle: &smart_road::vehicle::Vehicle, assets: &Assets) {
    let color = turn_color(vehicle.turn, vehicle.variant);
    let center = vec2(vehicle.position.x, vehicle.position.y);
    if let Some(texture) = &assets.car {
        draw_texture_ex(
            texture,
            center.x - VEHICLE_LENGTH * 0.5,
            center.y - VEHICLE_WIDTH * 0.5,
            color,
            DrawTextureParams {
                dest_size: Some(vec2(VEHICLE_LENGTH, VEHICLE_WIDTH)),
                rotation: vehicle.rotation,
                pivot: Some(center),
                ..Default::default()
            },
        );
    } else {
        draw_fallback_car(vehicle, color);
    }
    let label = format!("{}{}", vehicle.id, vehicle.turn.letter());
    text(&label, center.x - 12.0, center.y - 14.0, 14.0, WHITE);
}

fn draw_fallback_car(vehicle: &smart_road::vehicle::Vehicle, color: Color) {
    let forward = vehicle.pose.forward;
    let right = vehicle.pose.right;
    let center = vehicle.position;
    let half_l = VEHICLE_LENGTH * 0.5;
    let half_w = VEHICLE_WIDTH * 0.5;
    let corner = |fu: f32, ru: f32| {
        let p = center + forward * (fu * half_l) + right * (ru * half_w);
        vec2(p.x, p.y)
    };
    let a = corner(1.0, 1.0);
    let b = corner(1.0, -1.0);
    let c = corner(-1.0, -1.0);
    let d = corner(-1.0, 1.0);
    draw_triangle(a, b, c, color);
    draw_triangle(a, c, d, color);
    let nose = with_alpha(WHITE, 0.85);
    draw_triangle(
        corner(1.0, 0.55),
        corner(1.0, -0.55),
        corner(0.45, 0.0),
        nose,
    );
}

fn draw_hud(sim: &Simulation) {
    draw_rectangle(12.0, 12.0, 330.0, 292.0, Color::from_rgba(8, 16, 12, 185));
    text("Smart Road Intersection", 24.0, 40.0, 22.0, WHITE);
    text("Up: from South", 24.0, 68.0, 18.0, WHITE);
    text("Down: from North", 24.0, 90.0, 18.0, WHITE);
    text("Right: from West", 24.0, 112.0, 18.0, WHITE);
    text("Left: from East", 24.0, 134.0, 18.0, WHITE);
    text(
        "R: toggle random    Esc: statistics",
        24.0,
        160.0,
        16.0,
        Color::from_rgba(220, 230, 220, 255),
    );
    let stats = sim.statistics();
    text(
        &format!("Vehicles: {}", sim.active_count()),
        24.0,
        188.0,
        18.0,
        WHITE,
    );
    text(
        &format!("Spawned: {}", stats.spawned),
        24.0,
        210.0,
        18.0,
        WHITE,
    );
    text(
        &format!("Completed: {}", stats.completed),
        24.0,
        232.0,
        18.0,
        WHITE,
    );
    text(
        &format!(
            "Rejected/deferred: {}",
            stats.rejected_spawns + stats.deferred_spawns
        ),
        24.0,
        254.0,
        18.0,
        WHITE,
    );
    text(
        &format!("Collisions: {}", stats.collisions),
        24.0,
        276.0,
        18.0,
        WHITE,
    );
    text(
        &format!("Time: {:.1} s", sim.time()),
        190.0,
        276.0,
        18.0,
        WHITE,
    );

    draw_rectangle(652.0, 12.0, 336.0, 168.0, Color::from_rgba(8, 16, 12, 185));
    text("Route colors", 668.0, 40.0, 22.0, WHITE);
    swatch(668.0, 58.0, turn_color(Turn::Right, 0), "Right turn");
    swatch(668.0, 86.0, turn_color(Turn::Straight, 0), "Straight");
    swatch(668.0, 114.0, turn_color(Turn::Left, 0), "Left turn");
    let random_label = if sim.random_enabled() {
        "Random generation: ON"
    } else {
        "Random generation: OFF"
    };
    let random_color = if sim.random_enabled() {
        Color::from_rgba(130, 255, 150, 255)
    } else {
        Color::from_rgba(255, 196, 150, 255)
    };
    text(random_label, 668.0, 158.0, 18.0, random_color);
    text(
        "1 px = 0.1 m",
        668.0,
        980.0,
        16.0,
        Color::from_rgba(230, 245, 230, 255),
    );
}

fn swatch(x: f32, y: f32, color: Color, label: &str) {
    draw_rectangle(x, y, 22.0, 14.0, color);
    text(label, x + 30.0, y + 13.0, 18.0, WHITE);
}

fn turn_color(turn: Turn, variant: u8) -> Color {
    let (r, g, b) = match turn {
        Turn::Right => (255, 176, 32),
        Turn::Straight => (36, 198, 232),
        Turn::Left => (214, 78, 214),
    };
    let shift = match variant % 3 {
        0 => 0,
        1 => -32,
        _ => 24,
    };
    Color::from_rgba(
        channel(r + shift),
        channel(g + shift),
        channel(b + shift),
        255,
    )
}

fn channel(value: i32) -> u8 {
    value.clamp(0, 255) as u8
}

fn with_alpha(color: Color, alpha: f32) -> Color {
    Color::new(color.r, color.g, color.b, alpha)
}

fn text(value: &str, x: f32, y: f32, size: f32, color: Color) {
    draw_text(
        value,
        x + 1.0,
        y + 1.0,
        size,
        Color::from_rgba(0, 0, 0, 170),
    );
    draw_text(value, x, y, size, color);
}

#[cfg(test)]
mod asset_tests {
    #[test]
    fn embedded_pngs_decode() {
        for bytes in [
            include_bytes!("../assets/asphalt.png").as_slice(),
            include_bytes!("../assets/car.png").as_slice(),
            include_bytes!("../assets/arrow.png").as_slice(),
        ] {
            let image = image::load_from_memory(bytes).expect("png decode");
            assert!(image.width() > 0 && image.height() > 0);
        }
    }
}
