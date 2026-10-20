//! Graphical shell for the smart intersection.
//!
//! The window is a fixed 1000×1000 pixel view of the simulation world.
//! Esc freezes the run and opens the statistics screen. Enter, Space, or Esc
//! closes that screen.

mod render;

use macroquad::prelude::*;
use smart_road::routes::Direction;
use smart_road::simulation::Simulation;

use render::Assets;

fn window_conf() -> Conf {
    Conf {
        window_title: "Smart Road Intersection".to_owned(),
        window_width: 1000,
        window_height: 1000,
        high_dpi: false,
        fullscreen: false,
        sample_count: 1,
        window_resizable: false,
        ..Default::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let seed = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|duration| duration.as_nanos() as u64)
        .unwrap_or(1);
    let mut simulation = Simulation::new(seed);
    let assets = Assets::load();
    let mut repeat = [0.0_f32; 4];
    let mut showing_stats = false;

    loop {
        if showing_stats {
            if is_key_pressed(KeyCode::Enter)
                || is_key_pressed(KeyCode::Space)
                || is_key_pressed(KeyCode::Escape)
            {
                break;
            }
            render::draw_statistics(&simulation);
        } else if is_key_pressed(KeyCode::Escape) {
            simulation.freeze();
            showing_stats = true;
            render::draw_statistics(&simulation);
        } else {
            let dt = get_frame_time();
            poll_input(&mut simulation, &mut repeat, dt);
            simulation.update(dt);
            render::draw_world(&simulation, &assets);
        }
        next_frame().await;
    }
}

fn poll_input(simulation: &mut Simulation, repeat: &mut [f32; 4], dt: f32) {
    let bindings = [
        (KeyCode::Up, Direction::South),
        (KeyCode::Down, Direction::North),
        (KeyCode::Right, Direction::West),
        (KeyCode::Left, Direction::East),
    ];
    for (index, (key, direction)) in bindings.iter().enumerate() {
        if is_key_down(*key) {
            if repeat[index] <= 0.0 {
                simulation.keyboard_spawn(*direction);
                repeat[index] = 0.18;
            } else {
                repeat[index] -= dt;
            }
        } else {
            repeat[index] = 0.0;
        }
    }
    if is_key_pressed(KeyCode::R) {
        simulation.toggle_random();
    }
}
