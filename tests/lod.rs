//! Terrain LOD invariants over a spread of positions and attitudes.

use chopper::camera::{Camera, WorldPos};
use chopper::render::Renderer;
use chopper::terrain::{self, Lod};
use chopper::{horizon, worldgen};

#[test]
fn lod_is_crack_free_and_fits_the_command_list() {
    let world = worldgen::generate(1983);
    let mut state = 0x1234_5678u32;
    let mut next = || {
        state = state.wrapping_mul(1664525).wrapping_add(1013904223);
        state >> 8
    };
    for _ in 0..200 {
        let x = (next() % 50_000) as i32 - 25_000;
        let z = (next() % 50_000) as i32 - 25_000;
        let ground = terrain::height_at(&world, x, z);
        let agl = 8 + (next() % 3000) as i32;
        let camera = Camera::new(
            WorldPos { x, y: ground + agl, z },
            next() as u16,
            (next() % 0x2000) as u16 - 0x1000,
            (next() % 0x4000) as u16 - 0x2000,
        );
        for resolution_q8 in [64, 2048, 16384] {
            let mut renderer = Renderer::new();
            renderer.begin(14);
            horizon::draw_ground(&mut renderer, &camera, 6);
            let stats = terrain::render(&world, &camera, Lod { resolution_q8 }, &mut renderer);
            assert_eq!(stats.restriction_violations, 0, "{camera:?} res {resolution_q8}");
            if resolution_q8 == 64 {
                // The coarsest setting must always leave room for objects.
                assert!(renderer.triangle_count <= 200, "{} commands at {camera:?}", renderer.triangle_count);
            }
        }
    }
}
