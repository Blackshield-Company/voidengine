package voidengine_test

import "core:testing"
import "core:math/linalg"
import engine "../src/core"

@(test)
camera_deadzone :: proc(t: ^testing.T) {
    c := engine.camera2d_make(640, 360)
    c.lookahead_dist = 0
    engine.camera2d_update(&c, {10, 0}, 1, 1.0 / 60)
    testing.expect_value(t, c.center, [2]f32{0, 0})
    engine.camera2d_update(&c, {100, 0}, 1, 1.0 / 60)
    testing.expect_value(t, c.center.x, f32(100 - 20))
    engine.camera2d_update(&c, {80, -100}, 1, 1.0 / 60)
    testing.expect_value(t, c.center.y, f32(-100 + 28))
}

@(test)
camera_lookahead :: proc(t: ^testing.T) {
    c := engine.camera2d_make(640, 360)
    c.deadzone = {0, 0}
    engine.camera2d_update(&c, {0, 0}, 1, 1)
    testing.expect_value(t, c.lookahead, c.lookahead_dist)
    testing.expect_value(t, c.center.x, c.lookahead_dist)
    engine.camera2d_update(&c, {0, 0}, -1, 0.1) // turns around: slides, not snaps
    testing.expect(t, c.lookahead < c.lookahead_dist && c.lookahead > -c.lookahead_dist)
}

@(test)
camera_room_bounds :: proc(t: ^testing.T) {
    c := engine.camera2d_make(640, 360)
    engine.camera2d_set_bounds(&c, {0, 0, 1000, 360})
    engine.camera2d_update(&c, {5000, 900}, 1, 1.0 / 60)
    testing.expect_value(t, c.center, [2]f32{1000 - 320, 180})
    engine.camera2d_update(&c, {-5000, 0}, -1, 1.0 / 60)
    testing.expect_value(t, c.center.x, f32(320))
    // a room smaller than the view is centered
    engine.camera2d_set_bounds(&c, {0, 0, 400, 200})
    testing.expect_value(t, c.center, [2]f32{200, 100})
}

@(test)
camera_shake_decays_and_is_seeded :: proc(t: ^testing.T) {
    a := engine.camera2d_make(640, 360)
    b := engine.camera2d_make(640, 360)
    engine.camera2d_add_trauma(&a, 1)
    engine.camera2d_add_trauma(&b, 1)
    engine.camera2d_update(&a, {0, 0}, 1, 1.0 / 60)
    engine.camera2d_update(&b, {0, 0}, 1, 1.0 / 60)
    testing.expect(t, a.shake_offset != {0, 0}, "shaking")
    testing.expect_value(t, a.shake_offset, b.shake_offset)
    testing.expect(t, abs(a.shake_offset.x) <= a.shake_max && abs(a.shake_offset.y) <= a.shake_max)
    for _ in 0 ..< 120 { engine.camera2d_update(&a, {0, 0}, 1, 1.0 / 60) }
    testing.expect_value(t, a.trauma, f32(0))
    testing.expect_value(t, a.shake_offset, [2]f32{0, 0})
}

@(test)
camera_view_origin_is_whole_pixels :: proc(t: ^testing.T) {
    c := engine.camera2d_make(640, 360)
    c.center = {100.7, 50.2}
    o := engine.camera2d_view_origin(&c)
    testing.expect_value(t, o, [2]f32{-220, -130})
}

@(test)
pixel_fit_integer_scales :: proc(t: ^testing.T) {
    f := engine.pixel_fit(1920, 1080, 640, 360)
    testing.expect_value(t, f, engine.Pixel_Fit{scale = 3, x = 0, y = 0, w = 1920, h = 1080})
    f = engine.pixel_fit(2560, 1440, 640, 360)
    testing.expect_value(t, f.scale, i32(4))
    f = engine.pixel_fit(1280, 800, 640, 360) // Steam Deck
    testing.expect_value(t, f, engine.Pixel_Fit{scale = 2, x = 0, y = 40, w = 1280, h = 720})
    f = engine.pixel_fit(800, 600, 640, 360)
    testing.expect_value(t, f, engine.Pixel_Fit{scale = 1, x = 80, y = 120, w = 640, h = 360})
    f = engine.pixel_fit(320, 200, 640, 360) // tiny window: never below 1x
    testing.expect_value(t, f.scale, i32(1))
    ix, iy, ok := engine.pixel_fit_to_internal(engine.pixel_fit(1280, 800, 640, 360), 641, 41)
    testing.expect(t, ok)
    testing.expect_value(t, ix, i32(320))
    testing.expect_value(t, iy, i32(0))
    _, _, ok2 := engine.pixel_fit_to_internal(engine.pixel_fit(1280, 800, 640, 360), 10, 10)
    testing.expect(t, !ok2, "letterbox bar")
}

@(test)
ortho_camera_projection :: proc(t: ^testing.T) {
    cam := engine.R3D_Camera{position = {100, 50, 100}, near_z = 1, far_z = 500, ortho_height = 360}
    vp := engine.r3d_camera_view_proj(&cam, 640.0 / 360.0, false)
    p0 := vp * linalg.Vector4f32{100, 50, 0, 1}
    testing.expect(t, abs(p0.x) < 1e-4 && abs(p0.y) < 1e-4, "center maps to NDC 0")
    p1 := vp * linalg.Vector4f32{100 + 320, 50 + 180, 0, 1}
    testing.expectf(t, abs(p1.x - 1) < 1e-4 && abs(p1.y - 1) < 1e-4, "corner maps to NDC 1,1 (got %v)", p1)
    p2 := vp * linalg.Vector4f32{100 + 320, 50 + 180, -200, 1} // depth doesn't shrink things
    testing.expect(t, abs(p2.x - 1) < 1e-4 && abs(p2.w - 1) < 1e-4)
    persp := engine.R3D_Camera{position = {0, 0, 5}, fov_y = 1, near_z = 0.1, far_z = 10}
    vpp := engine.r3d_camera_view_proj(&persp, 1, false)
    q := vpp * linalg.Vector4f32{0, 0, 0, 1}
    testing.expect(t, abs(q.w - 5) < 1e-4, "ortho_height 0 keeps perspective")
}

@(test)
parallax_layers :: proc(t: ^testing.T) {
    l := engine.Parallax_Layer{factor = {0.5, 0.5}}
    testing.expect_value(t, engine.parallax_screen_pos(l, {100, 40}), [2]f32{-50, -20})
    testing.expect_value(t, engine.parallax_world_pos(l, {100, 40}), [2]f32{50, 20})
    sky := engine.Parallax_Layer{factor = {0, 0}, offset = {0, 10}}
    testing.expect_value(t, engine.parallax_screen_pos(sky, {9999, 9999}), [2]f32{0, 10})
    rep := engine.Parallax_Layer{factor = {1, 1}, width = 200}
    first, count := engine.parallax_repeat(rep, {350, 0}, 640)
    testing.expect_value(t, first, f32(-150))
    testing.expect_value(t, count, 4)
}

@(test)
rooms_and_transitions :: proc(t: ^testing.T) {
    a_doors := []engine.Door{{name = "east", rect = {620, 280, 20, 48}, target_room = "B", target_door = "west"}}
    b_doors := []engine.Door{{name = "west", rect = {0, 280, 20, 48}, target_room = "A", target_door = "east", spawn_offset = {30, 0}}}
    rooms := []engine.Room{{name = "A", bounds = {0, 0, 640, 360}, doors = a_doors}, {name = "B", bounds = {0, 0, 1280, 360}, doors = b_doors}}

    bi, ok := engine.room_find(rooms, "B")
    testing.expect(t, ok && bi == 1)
    _, ok = engine.room_find(rooms, "nope")
    testing.expect(t, !ok)
    di, dok := engine.room_door_find(&rooms[1], "west")
    testing.expect(t, dok && di == 0)
    testing.expect_value(t, engine.door_spawn_point(rooms[1].doors[0]), [2]f32{40, 328})

    rt := engine.Room_Transition{fade_time = 0.25}
    in_door := engine.Rect2{625, 300, 12, 24}
    clear_box := engine.Rect2{300, 300, 12, 24}
    _, started := engine.room_transition_check(&rt, &rooms[0], in_door)
    testing.expect(t, !started, "not armed yet (spawned inside a door)")
    _, started = engine.room_transition_check(&rt, &rooms[0], clear_box)
    testing.expect(t, !started)
    _, started = engine.room_transition_check(&rt, &rooms[0], in_door)
    testing.expect(t, started, "armed, then entered the door")
    testing.expect(t, engine.room_transition_busy(&rt))
    testing.expect_value(t, rt.target_room, "B")
    testing.expect(t, !engine.room_transition_update(&rt, 0.1))
    testing.expect(t, engine.room_transition_alpha(&rt) > 0.3)
    testing.expect(t, engine.room_transition_update(&rt, 0.2), "swap at full black")
    testing.expect(t, !engine.room_transition_update(&rt, 0.05), "swap fires once")
    testing.expect(t, engine.room_transition_alpha(&rt) < 1)
    engine.room_transition_update(&rt, 0.3)
    testing.expect(t, !engine.room_transition_busy(&rt))
    testing.expect_value(t, engine.room_transition_alpha(&rt), f32(0))
    _, started = engine.room_transition_check(&rt, &rooms[1], engine.Rect2{5, 300, 12, 24})
    testing.expect(t, !started, "arriving inside a door doesn't bounce back")
}

Test_Streams :: enum { World, Loot, Combat }

@(test)
rng_streams_are_deterministic_and_independent :: proc(t: ^testing.T) {
    a := engine.rng_stream(42, 1)
    b := engine.rng_stream(42, 1)
    for _ in 0 ..< 50 { testing.expect_value(t, engine.rng_next(&a), engine.rng_next(&b)) }

    w := engine.rng_stream(42, 0)
    l := engine.rng_stream(42, 1)
    testing.expect(t, engine.rng_next(&w) != engine.rng_next(&l), "streams differ")

    // drawing from one stream never shifts another
    s1: [Test_Streams]engine.Rng
    s2: [Test_Streams]engine.Rng
    engine.rng_streams_fill(&s1, engine.rng_seed_from_string("FRAY-7KQ2-ASHE"))
    engine.rng_streams_fill(&s2, engine.rng_seed_from_string("FRAY-7KQ2-ASHE"))
    for _ in 0 ..< 100 { engine.rng_next(&s1[.Loot]) }
    for _ in 0 ..< 20 { testing.expect_value(t, engine.rng_next(&s1[.World]), engine.rng_next(&s2[.World])) }

    testing.expect(t, engine.rng_seed_from_string("2026-11-22") != engine.rng_seed_from_string("2026-11-23"))
    testing.expect_value(t, engine.rng_seed_from_string("abc"), engine.rng_seed_from_string("abc"))
}

@(test)
rng_ranges :: proc(t: ^testing.T) {
    r := engine.rng_make(7)
    seen: [7]bool
    for _ in 0 ..< 2000 {
        v := engine.rng_range_i(&r, 1, 6)
        testing.expect(t, v >= 1 && v <= 6)
        seen[v] = true
        f := engine.rng_f32(&r)
        testing.expect(t, f >= 0 && f < 1)
        g := engine.rng_range_f(&r, -3, 3)
        testing.expect(t, g >= -3 && g < 3)
    }
    for i in 1 ..= 6 { testing.expectf(t, seen[i], "value %d never rolled", i) }
    testing.expect_value(t, engine.rng_range_i(&r, 5, 5), 5)
    z := engine.rng_make(0)
    testing.expect(t, engine.rng_next(&z) != 0 || engine.rng_next(&z) != 0, "seed 0 still works")
}
