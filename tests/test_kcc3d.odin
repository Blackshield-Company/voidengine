package voidengine_test

// Tests for kcc3d.odin: Godot PlayerController numbers (walk/run, braking,
// deadzone), slopes against the 45 deg limit on meshes and on the
// heightfield, steps, walls with slide, falling and landing.

import "core:math"
import "core:math/linalg"
import "core:testing"
import engine "../src/core"

@(private="file")
add_hull :: proc(out: ^[dynamic]engine.R3D_Tri, c: [8][3]f32) {
    faces := [6][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 4, 7, 3}, {1, 2, 6, 5}, {3, 7, 6, 2}, {0, 1, 5, 4}}
    for f in faces {
        append(out, engine.r3d_tri_make(c[f[0]], c[f[1]], c[f[2]], 3), engine.r3d_tri_make(c[f[0]], c[f[2]], c[f[3]], 3))
    }
}
@(private="file")
add_box :: proc(out: ^[dynamic]engine.R3D_Tri, lo, hi: [3]f32) {
    add_hull(out, {{lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z},
                   {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}})
}
// ramp rising along +x from x0 at `deg`, `rise` high, across z -5..5
@(private="file")
add_ramp :: proc(out: ^[dynamic]engine.R3D_Tri, x0, deg, rise: f32) {
    run := rise / math.tan(math.to_radians(deg))
    x1 := x0 + run
    add_hull(out, {{x0, -1, -5}, {x1, -1, -5}, {x1, rise, -5}, {x0, 0, -5}, {x0, -1, 5}, {x1, -1, 5}, {x1, rise, 5}, {x0, 0, 5}})
    add_box(out, {x1, -1, -5}, {x1 + 60, rise, 5})
}

@(private="file")
simulate :: proc(k: ^engine.R3D_Kcc, w: ^engine.R3D_Collision_World, move: [2]f32, run: bool, secs: f32) {
    for _ in 0 ..< int(secs * 60) { engine.r3d_kcc_step(k, w, engine.r3d_kcc_wish(k, move, run, 0), 1.0 / 60) }
}

@(test)
test_kcc_speeds_and_braking :: proc(t: ^testing.T) {
    w: engine.R3D_Collision_World
    defer engine.r3d_world_destroy(&w)
    tris: [dynamic]engine.R3D_Tri
    add_box(&tris, {-100, -1, -10}, {100, 0, 10})
    engine.r3d_world_add_mesh(&w, tris)
    k := engine.r3d_kcc_default()
    engine.r3d_kcc_place(&k, &w, {-90, 0.5, 0})
    testing.expect(t, k.on_floor && abs(k.pos.y - 0.01) < 1e-3)
    simulate(&k, &w, {1, 0}, false, 2)
    testing.expectf(t, abs(engine.r3d_kcc_horizontal_speed(&k) - 1.6) < 1e-3, "walk %v", engine.r3d_kcc_horizontal_speed(&k))
    simulate(&k, &w, {1, 0}, true, 2)
    testing.expectf(t, abs(engine.r3d_kcc_horizontal_speed(&k) - 4.5) < 1e-3, "run %v", engine.r3d_kcc_horizontal_speed(&k))
    steps := 0
    for engine.r3d_kcc_horizontal_speed(&k) >= 0.05 && steps < 100 { engine.r3d_kcc_step(&k, &w, {}, 1.0 / 60); steps += 1 }
    testing.expectf(t, f32(steps) / 60 <= 0.3, "stop took %v s", f32(steps) / 60)
    testing.expect(t, k.on_floor && abs(k.pos.y - 0.01) < 1e-3 && k.pos.z == 0)
    // half stick through the radial deadzone: (0.5 - 0.2) / 0.8
    half := engine.r3d_radial_deadzone({0.5, 0}, 0.2)
    testing.expect(t, abs(half.x - 0.375) < 1e-6)
    testing.expect(t, engine.r3d_radial_deadzone({0.19, 0}, 0.2) == {})
    // camera-relative: yaw 0 forward = -z, right = +x; yaw 90 forward = -x
    f := engine.r3d_move_direction({0, 1}, 0)
    testing.expect(t, linalg.length(f - {0, 0, -1}) < 1e-6)
    f = engine.r3d_move_direction({0, 1}, math.PI / 2)
    testing.expect(t, linalg.length(f - {-1, 0, 0}) < 1e-6)
}

@(test)
test_kcc_slopes_walkable_limit :: proc(t: ^testing.T) {
    for c in ([][3]f32{{25, 1, 0}, {40, 1, 0}, {50, 0, 0}, {60, 0, 0}}) {
        deg := c.x
        walkable := c.y > 0
        w: engine.R3D_Collision_World
        tris: [dynamic]engine.R3D_Tri
        add_box(&tris, {-20, -1, -5}, {0, 0, 5})
        add_ramp(&tris, 0, deg, 4)
        engine.r3d_world_add_mesh(&w, tris)
        k := engine.r3d_kcc_default()
        engine.r3d_kcc_place(&k, &w, {-3, 0.3, 0})
        y0 := k.pos.y
        simulate(&k, &w, {1, 0}, true, 6)
        rise := k.pos.y - y0
        if walkable {
            testing.expectf(t, rise > 3.9, "%v deg: should climb, rose %v", deg, rise)
        } else {
            testing.expectf(t, rise <= 0.5, "%v deg: non-walkable rose %v", deg, rise)
            testing.expectf(t, k.on_floor, "%v deg: grounded against the slope", deg)
        }
        engine.r3d_world_destroy(&w)
    }
}

@(test)
test_kcc_steps_and_walls :: proc(t: ^testing.T) {
    w: engine.R3D_Collision_World
    defer engine.r3d_world_destroy(&w)
    tris: [dynamic]engine.R3D_Tri
    add_box(&tris, {-20, -1, -20}, {20, 0, 20})
    add_box(&tris, {2, 0, -20}, {20, 0.3, -10})   // 0.3 m step (x >= 2, z < -10)
    add_box(&tris, {2, 0, 10}, {20, 0.5, 20})     // 0.5 m ledge (z > 10)
    add_box(&tris, {-20, 0, -2}, {-19, 3, 2})     // wall at x = -19
    engine.r3d_world_add_mesh(&w, tris)
    k := engine.r3d_kcc_default()
    engine.r3d_kcc_place(&k, &w, {0, 0.2, -15})
    simulate(&k, &w, {1, 0}, false, 3)
    testing.expectf(t, k.pos.x > 3 && abs(k.pos.y - 0.31) < 0.02, "0.3 m step: at %v", k.pos)
    testing.expect(t, k.steps_up >= 1)
    k = engine.r3d_kcc_default()
    engine.r3d_kcc_place(&k, &w, {0, 0.2, 15})
    simulate(&k, &w, {1, 0}, false, 3)
    testing.expectf(t, k.pos.x < 2 - 0.39 && k.pos.y < 0.05, "0.5 m ledge blocks: at %v", k.pos)
    // diagonal into the wall: blocked in x, keeps sliding in z
    k = engine.r3d_kcc_default()
    engine.r3d_kcc_place(&k, &w, {-17, 0.2, 0.5})
    z0 := k.pos.z
    simulate(&k, &w, {-0.7071, 0.7071}, false, 2)
    testing.expectf(t, k.pos.x >= -19 + 0.4 - 1e-3, "wall: x %v", k.pos.x)
    testing.expectf(t, z0 - k.pos.z > 1.0, "wall slide: moved %v in z", z0 - k.pos.z)
}

@(test)
test_kcc_fall_and_land :: proc(t: ^testing.T) {
    w: engine.R3D_Collision_World
    defer engine.r3d_world_destroy(&w)
    tris: [dynamic]engine.R3D_Tri
    add_box(&tris, {-10, -1, -5}, {0, 0, 5})
    add_box(&tris, {0, -4, -5}, {10, -3, 5})
    engine.r3d_world_add_mesh(&w, tris)
    k := engine.r3d_kcc_default()
    engine.r3d_kcc_place(&k, &w, {-1, 0.2, 0})
    simulate(&k, &w, {1, 0}, true, 2)
    testing.expectf(t, k.on_floor && abs(k.pos.y - (-3 + 0.01)) < 0.02, "landed at %v", k.pos)
    testing.expect(t, k.air_s == 0)
}

@(test)
test_kcc_heightfield_slopes :: proc(t: ^testing.T) {
    // 25 deg walkable and 50 deg not, as heightfield terrain
    for c in ([][2]f32{{25, 1}, {50, 0}}) {
        hf := engine.r3d_heightfield_make(65, 1, {-32, -32})
        tn := math.tan(math.to_radians(c.x))
        for j in 0 ..< 65 { for i in 0 ..< 65 { x := f32(i) - 32; hf.heights[j * 65 + i] = x > 0 ? min(x, 15) * tn : 0 } }
        w: engine.R3D_Collision_World
        w.hf = &hf
        w.hf_mask = 3
        k := engine.r3d_kcc_default()
        engine.r3d_kcc_place(&k, &w, {-4, 0.3, 0})
        simulate(&k, &w, {1, 0}, true, 5)
        if c.y > 0 {
            testing.expectf(t, k.pos.x > 10 && k.on_floor, "25 deg terrain: at %v", k.pos)
            testing.expect(t, abs(k.pos.y - engine.r3d_hf_height_tri(&hf, k.pos.x, k.pos.z)) < 0.1)
        } else {
            testing.expectf(t, k.pos.y <= 0.5, "50 deg terrain rose to %v", k.pos.y)
        }
        w.hf = nil
        engine.r3d_world_destroy(&w)
        engine.r3d_heightfield_destroy(&hf)
    }
}
