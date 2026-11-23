package voidengine_test

import "core:testing"
import "core:math"
import engine "../src/core"

@(private="file")
near :: proc(a, b: f32, eps: f32 = 0.01) -> bool { return abs(a - b) <= eps }

// ---------------------------------------------------------------- collision

@(test)
collision_lands_on_floor :: proc(t: ^testing.T) {
    g := engine.collision_grid_from_strings({"........", "........", "########"}, 16)
    defer engine.collision_grid_destroy(&g)
    b := engine.AABB{20, 0, 12, 24}
    res := engine.body_move(&g, &b, {0, 20})
    testing.expect(t, res.on_ground, "should land")
    testing.expect_value(t, b.y, f32(8)) // feet on y=32
}

@(test)
collision_fast_fall_does_not_tunnel :: proc(t: ^testing.T) {
    g := engine.collision_grid_from_strings({"........", "........", "........", "########", "........", "........"}, 16)
    defer engine.collision_grid_destroy(&g)
    b := engine.AABB{20, 0, 12, 24}
    res := engine.body_move(&g, &b, {0, 400})
    testing.expect(t, res.on_ground)
    testing.expect_value(t, b.y, f32(48 - 24))
}

@(test)
collision_wall_stops_horizontal :: proc(t: ^testing.T) {
    g := engine.collision_grid_from_strings({"....#", "....#", "#####"}, 16)
    defer engine.collision_grid_destroy(&g)
    b := engine.AABB{40, 8, 12, 24}
    res := engine.body_move(&g, &b, {30, 0.5})
    testing.expect(t, res.hit_right, "should hit the wall")
    testing.expect(t, res.on_ground, "still standing")
    testing.expect_value(t, b.x, f32(64 - 12))
    b2 := engine.AABB{40, 8, 12, 24}
    res2 := engine.body_move(&g, &b2, {-60, 0})
    testing.expect(t, !res2.hit_left, "open to the left (edge is empty)")
}

@(test)
collision_ceiling_stops_rise :: proc(t: ^testing.T) {
    g := engine.collision_grid_from_strings({"####", "....", "....", "####"}, 16)
    defer engine.collision_grid_destroy(&g)
    b := engine.AABB{10, 20, 12, 24}
    res := engine.body_move(&g, &b, {0, -20})
    testing.expect(t, res.hit_ceiling)
    testing.expect_value(t, b.y, f32(16))
}

@(test)
collision_edge_solid_option :: proc(t: ^testing.T) {
    g := engine.collision_grid_from_strings({"....", "....", "####"}, 16)
    defer engine.collision_grid_destroy(&g)
    g.edge_solid = true
    b := engine.AABB{4, 8, 12, 24}
    res := engine.body_move(&g, &b, {-20, 0})
    testing.expect(t, res.hit_left)
    testing.expect_value(t, b.x, f32(0))
}

@(test)
collision_one_way_platforms :: proc(t: ^testing.T) {
    rows := []string{"........", "..====..", "........", "........", "########"}
    g := engine.collision_grid_from_strings(rows, 16)
    defer engine.collision_grid_destroy(&g)

    // jump up through it from below
    b := engine.AABB{36, 40, 12, 24}
    res := engine.body_move(&g, &b, {0, -30})
    testing.expect(t, !res.hit_ceiling, "one-way must not block from below")
    testing.expect_value(t, b.y, f32(10))

    // land on it from above
    b = engine.AABB{36, -10, 12, 24}
    res = engine.body_move(&g, &b, {0, 10})
    testing.expect(t, res.on_ground && res.on_one_way)
    testing.expect_value(t, b.y, f32(16 - 24))

    // drop through it
    b = engine.AABB{36, -8, 12, 24}
    res = engine.body_move(&g, &b, {0, 8}, engine.Move_Options{drop_through = true})
    testing.expect(t, !res.on_ground, "drop-through falls past")
    testing.expect_value(t, b.y, f32(0))

    // sideways it is never a wall
    b = engine.AABB{0, 8, 12, 24} // body overlaps row 1 height
    res = engine.body_move(&g, &b, {40, 0})
    testing.expect(t, !res.hit_right)
}

@(test)
collision_walk_up_slope_and_off_the_top :: proc(t: ^testing.T) {
    rows := []string{"........", "........", "....../#", "########"}
    g := engine.collision_grid_from_strings(rows, 16)
    defer engine.collision_grid_destroy(&g)
    b := engine.AABB{70, 24, 12, 24} // center x 76, feet on y 48
    res := engine.body_move(&g, &b, {28, 1}, engine.Move_Options{grounded = true, snap_down = 6})
    // center x 104 -> 8px into the slope tile -> floor at 48 - 8 = 40
    testing.expect(t, res.on_ground && res.on_slope, "on the slope")
    testing.expectf(t, near(b.y + b.h, 40), "feet %v, want 40", b.y + b.h)
    testing.expect(t, !res.hit_right, "slope is not a wall")

    // keep walking off the top onto the flat solid tile (no wall, no pop)
    res = engine.body_move(&g, &b, {20, 1}, engine.Move_Options{grounded = true, snap_down = 6})
    testing.expect(t, !res.hit_right, "slope top must not act as a wall")
    testing.expect(t, res.on_ground)
    testing.expectf(t, near(b.y + b.h, 32), "feet %v, want 32", b.y + b.h)
}

@(test)
collision_walk_down_slope_sticks :: proc(t: ^testing.T) {
    rows := []string{"........", "........", "....../#", "########"}
    g := engine.collision_grid_from_strings(rows, 16)
    defer engine.collision_grid_destroy(&g)
    b := engine.AABB{114, 8, 12, 24} // on the flat top, center 120
    res := engine.body_move(&g, &b, {-16, 1}, engine.Move_Options{grounded = true, snap_down = 8})
    testing.expect(t, res.on_ground, "snapped to the slope")
    testing.expectf(t, near(b.y + b.h, 40), "feet %v, want 40", b.y + b.h) // center 104
    // all the way down onto the flat floor
    res = engine.body_move(&g, &b, {-30, 1}, engine.Move_Options{grounded = true, snap_down = 8})
    testing.expect(t, res.on_ground)
    testing.expectf(t, near(b.y + b.h, 48), "feet %v, want 48", b.y + b.h)
}

@(test)
collision_slope_up_left_mirror :: proc(t: ^testing.T) {
    rows := []string{"........", "........", "#\\......", "########"}
    g := engine.collision_grid_from_strings(rows, 16)
    defer engine.collision_grid_destroy(&g)
    b := engine.AABB{46, 24, 12, 24} // center 52, flat floor
    res := engine.body_move(&g, &b, {-28, 1}, engine.Move_Options{grounded = true, snap_down = 6})
    // center 24 -> lx 8 in the ◣ tile at x 16..32 -> floor 32 + 8 = 40
    testing.expect(t, res.on_ground && res.on_slope)
    testing.expectf(t, near(b.y + b.h, 40), "feet %v, want 40", b.y + b.h)
}

@(test)
collision_ground_probe_and_blocked :: proc(t: ^testing.T) {
    g := engine.collision_grid_from_strings({"....", "....", "####"}, 16)
    defer engine.collision_grid_destroy(&g)
    b := engine.AABB{4, 4, 12, 24} // feet at 28, floor at 32
    d, ok, _ := engine.ground_probe(&g, &b, 8)
    testing.expect(t, ok)
    testing.expect_value(t, d, f32(4))
    _, ok2, _ := engine.ground_probe(&g, &b, 2)
    testing.expect(t, !ok2)
    testing.expect(t, engine.collision_box_blocked(&g, {0, 30, 8, 8}))
    testing.expect(t, !engine.collision_box_blocked(&g, {0, 0, 8, 8}))
}

@(test)
collision_grid_from_tilemap_shapes :: proc(t: ^testing.T) {
    tm := engine.tilemap_create(3, 1, 16)
    defer engine.tilemap_destroy(tm)
    engine.tilemap_set(tm, 0, 0, 1)
    engine.tilemap_set(tm, 1, 0, 2)
    engine.tilemap_set(tm, 2, 0, 3)
    engine.tilemap_set_solid(tm, 1)
    shapes := make(map[i32]engine.Tile_Shape)
    defer delete(shapes)
    shapes[2] = .One_Way
    g := engine.collision_grid_from_tilemap(tm, shapes)
    defer engine.collision_grid_destroy(&g)
    testing.expect_value(t, engine.collision_grid_get(&g, 0, 0), engine.Tile_Shape.Solid)
    testing.expect_value(t, engine.collision_grid_get(&g, 1, 0), engine.Tile_Shape.One_Way)
    testing.expect_value(t, engine.collision_grid_get(&g, 2, 0), engine.Tile_Shape.Empty)
}

// ---------------------------------------------------------------- controller

DT :: f32(1.0 / 60.0)

@(test)
controller_coyote_time :: proc(t: ^testing.T) {
    p := engine.platformer_default_params()
    s := engine.Platformer_State{grounded = true}
    engine.platformer_update(&p, &s, {}, DT)
    s.grounded = false // walked off a ledge
    for _ in 0 ..< 3 { engine.platformer_update(&p, &s, {}, DT) } // 50 ms
    _, jumped := engine.platformer_update(&p, &s, {jump_pressed = true, jump_held = true}, DT)
    testing.expect(t, jumped, "jump inside the coyote window")

    s2 := engine.Platformer_State{grounded = true}
    engine.platformer_update(&p, &s2, {}, DT)
    s2.grounded = false
    for _ in 0 ..< 10 { engine.platformer_update(&p, &s2, {}, DT) } // 167 ms
    _, jumped2 := engine.platformer_update(&p, &s2, {jump_pressed = true, jump_held = true}, DT)
    testing.expect(t, !jumped2, "no jump after the coyote window")
}

@(test)
controller_jump_buffer :: proc(t: ^testing.T) {
    p := engine.platformer_default_params()
    s := engine.Platformer_State{}
    engine.platformer_update(&p, &s, {jump_pressed = true, jump_held = true}, DT) // pressed in the air
    for _ in 0 ..< 4 { engine.platformer_update(&p, &s, {jump_held = true}, DT) }
    s.grounded = true // lands 83 ms later
    _, jumped := engine.platformer_update(&p, &s, {jump_held = true}, DT)
    testing.expect(t, jumped, "buffered jump fires on landing")

    s2 := engine.Platformer_State{}
    engine.platformer_update(&p, &s2, {jump_pressed = true, jump_held = true}, DT)
    for _ in 0 ..< 10 { engine.platformer_update(&p, &s2, {jump_held = true}, DT) }
    s2.grounded = true
    _, jumped2 := engine.platformer_update(&p, &s2, {jump_held = true}, DT)
    testing.expect(t, !jumped2, "buffer expired")
}

@(private="file")
apex_of :: proc(p: ^engine.Platformer_Params, hold_frames: int) -> f32 {
    s := engine.Platformer_State{grounded = true}
    y: f32 = 0
    min_y: f32 = 0
    for i in 0 ..< 240 {
        held := i < hold_frames
        d, _ := engine.platformer_update(p, &s, {jump_pressed = i == 0, jump_held = held}, DT)
        s.grounded = false
        y += d.y
        min_y = min(min_y, y)
        if i > 0 && s.vel.y > 0 && y >= 0 { break }
    }
    return -min_y
}

@(test)
controller_variable_jump_height :: proc(t: ^testing.T) {
    p := engine.platformer_default_params()
    full := apex_of(&p, 1000)
    short := apex_of(&p, 4)
    testing.expectf(t, short < full * 0.6, "short hop %v should be well under full %v", short, full)
    testing.expectf(t, abs(full - engine.platformer_full_jump_height(&p)) < 6, "full %v vs formula %v", full, engine.platformer_full_jump_height(&p))
}

@(test)
controller_run_accel_and_landing :: proc(t: ^testing.T) {
    p := engine.platformer_default_params()
    s := engine.Platformer_State{grounded = true}
    for _ in 0 ..< 30 { engine.platformer_update(&p, &s, {move_x = 1}, DT) }
    testing.expect_value(t, s.vel.x, p.run_speed)
    testing.expect_value(t, s.facing, f32(1))
    engine.platformer_update(&p, &s, {move_x = -1}, DT)
    testing.expect_value(t, s.facing, f32(-1))
    testing.expect(t, s.vel.x < p.run_speed)

    s2 := engine.Platformer_State{grounded = false, vel = {0, 200}}
    landed := engine.platformer_apply_move(&s2, engine.Move_Result{on_ground = true})
    testing.expect(t, landed)
    testing.expect_value(t, s2.vel.y, f32(0))
    testing.expect(t, !engine.platformer_apply_move(&s2, engine.Move_Result{on_ground = true}), "landed only once")
}

@(test)
controller_and_collision_together :: proc(t: ^testing.T) {
    // a full-held jump from the floor clears a 2-tile (32px) ledge
    g := engine.collision_grid_from_strings({
        "..........",
        "..........",
        "..........",
        "......####",
        "......####",
        "##########",
    }, 16)
    defer engine.collision_grid_destroy(&g)
    g.edge_solid = true
    p := engine.platformer_default_params()
    s := engine.Platformer_State{grounded = true}
    b := engine.AABB{40, 80 - 24, 12, 24}
    for i in 0 ..< 120 {
        d, _ := engine.platformer_update(&p, &s, {move_x = 1, jump_pressed = i == 0, jump_held = true}, DT)
        res := engine.body_move(&g, &b, d, engine.Move_Options{grounded = s.grounded, snap_down = 4})
        engine.platformer_apply_move(&s, res)
    }
    testing.expect(t, s.grounded)
    testing.expectf(t, near(b.y + b.h, 48), "feet %v: should stand on the ledge top (48)", b.y + b.h)
    _ = math.PI
}
