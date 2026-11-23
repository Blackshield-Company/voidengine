package voidengine_test

// Tests for follow3d.odin (Templar FollowCamera port): snaps, follow lag,
// boom retraction and recovery, ground clearance on 25 deg terrain at all
// eight yaws (F2), and the interior blend.

import "core:math"
import "core:testing"
import engine "../src/core"

@(private="file")
DT :: f32(1.0 / 60)

@(test)
test_follow_snaps :: proc(t: ^testing.T) {
    c := engine.r3d_follow_default()
    engine.r3d_follow_set_yaw_index(&c, 0)
    engine.r3d_follow_request_snap(&c, 1)
    engine.r3d_follow_request_snap(&c, 1)
    engine.r3d_follow_request_snap(&c, 1) // replaces the queued one: at most one pending
    testing.expect(t, c.pending == 1)
    steps := 0
    for c.snaps_started < 2 && steps < 100 { engine.r3d_follow_advance_snap(&c, DT); steps += 1 }
    testing.expectf(t, f32(steps) * DT <= 0.25, "first snap settled in %v s", f32(steps) * DT)
    for engine.r3d_follow_is_snapping(&c) && steps < 100 { engine.r3d_follow_advance_snap(&c, DT); steps += 1 }
    testing.expectf(t, f32(steps) * DT <= 0.45, "two chained snaps took %v s", f32(steps) * DT)
    testing.expect(t, engine.r3d_follow_snap_index(&c) == 2)
    testing.expect(t, abs(engine.r3d_follow_displayed_yaw(&c) - math.PI / 2) < 1e-5)
    // boom at yaw 90 points over +x; camera looks back along -x
    b := engine.r3d_follow_boom_direction(math.PI / 2, 55)
    testing.expect(t, b.x > 0.5 && abs(b.z) < 1e-6 && abs(b.y - math.sin(math.to_radians(f32(55)))) < 1e-6)
}

@(test)
test_follow_lag_steady_state :: proc(t: ^testing.T) {
    c := engine.r3d_follow_default()
    c.probe_enabled = false
    p := [3]f32{0, 0, 0}
    engine.r3d_follow_reset(&c, nil, p)
    for _ in 0 ..< 240 { p.x += 4.5 * DT; engine.r3d_follow_step(&c, nil, p, DT) }
    lag := p.x - c.focus.x
    testing.expectf(t, abs(lag - 4.5 * 0.2) < 0.01, "steady lag %v", lag)
    for _ in 0 ..< 60 { p.x += 30 * DT; engine.r3d_follow_step(&c, nil, p, DT) }
    testing.expect(t, p.x - c.focus.x <= 2.5 + 1e-4)
}

@(test)
test_follow_boom_retracts_and_recovers :: proc(t: ^testing.T) {
    w: engine.R3D_Collision_World
    defer engine.r3d_world_destroy(&w)
    tris: [dynamic]engine.R3D_Tri
    lo, hi := [3]f32{-50, -1, -50}, [3]f32{50, 0, 50}
    for b in ([][2][3]f32{{lo, hi}, {{3, 0, -10}, {4, 10, 10}}}) {
        l, h := b[0], b[1]
        c := [8][3]f32{{l.x, l.y, l.z}, {h.x, l.y, l.z}, {h.x, h.y, l.z}, {l.x, h.y, l.z}, {l.x, l.y, h.z}, {h.x, l.y, h.z}, {h.x, h.y, h.z}, {l.x, h.y, h.z}}
        faces := [6][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 4, 7, 3}, {1, 2, 6, 5}, {3, 7, 6, 2}, {0, 1, 5, 4}}
        for f in faces { append(&tris, engine.r3d_tri_make(c[f[0]], c[f[1]], c[f[2]], 3), engine.r3d_tri_make(c[f[0]], c[f[2]], c[f[3]], 3)) }
    }
    engine.r3d_world_add_mesh(&w, tris)
    c := engine.r3d_follow_default()
    engine.r3d_follow_set_zoom(&c, 14)
    engine.r3d_follow_set_yaw_index(&c, 2) // boom over +x, into the wall at x = 3
    engine.r3d_follow_reset(&c, &w, {0, 0, 0})
    testing.expectf(t, c.effective_distance < 8, "retracted to %v", c.effective_distance)
    cam := engine.r3d_follow_camera_position(&c)
    testing.expect(t, !engine.r3d_sphere_overlaps(&w, cam, 0.2, engine.R3D_COL_CAMERA))
    // snap away from the wall: recovers smoothly to the full boom
    engine.r3d_follow_set_yaw_index(&c, 6)
    prev := c.effective_distance
    for _ in 0 ..< 240 {
        engine.r3d_follow_step(&c, &w, {0, 0, 0}, DT)
        testing.expect(t, c.effective_distance - prev <= max(1.0, (14 - prev) * 4) * DT + 1e-4)
        prev = c.effective_distance
    }
    testing.expectf(t, abs(c.effective_distance - 14) < 0.05, "recovered to %v", c.effective_distance)
}

@(test)
test_follow_clearance_on_25deg_terrain :: proc(t: ^testing.T) {
    hf := engine.r3d_heightfield_make(129, 2, {-128, -128})
    defer engine.r3d_heightfield_destroy(&hf)
    tn := math.tan(math.to_radians(f32(25)))
    for j in 0 ..< 129 { for i in 0 ..< 129 { hf.heights[j * 129 + i] = (f32(j) * 2 - 128) * tn } } // rises to +z
    w: engine.R3D_Collision_World
    w.hf = &hf
    w.hf_mask = 3
    defer { w.hf = nil; engine.r3d_world_destroy(&w) }
    for zoom in ([]f32{8, 14, 20}) {
        for yaw in 0 ..< 8 {
            c := engine.r3d_follow_default()
            engine.r3d_follow_set_zoom(&c, zoom)
            engine.r3d_follow_set_yaw_index(&c, yaw)
            feet := [3]f32{3, engine.r3d_hf_height(&hf, 3, 5), 5}
            engine.r3d_follow_reset(&c, &w, feet)
            for _ in 0 ..< 30 { engine.r3d_follow_step(&c, &w, feet, DT) }
            cam := engine.r3d_follow_camera_position(&c)
            clr := engine.r3d_follow_clearance_at(&c, &w, cam)
            testing.expectf(t, clr >= 0.5, "zoom %v yaw %v: clearance %v", zoom, yaw, clr)
        }
    }
}

@(test)
test_follow_interior_blend :: proc(t: ^testing.T) {
    c := engine.r3d_follow_default()
    c.probe_enabled = false
    engine.r3d_follow_set_zoom(&c, 20)
    engine.r3d_follow_reset(&c, nil, {})
    c.interior_mode = true
    for _ in 0 ..< 18 { engine.r3d_follow_step(&c, nil, {}, DT) } // 0.3 s
    testing.expectf(t, abs(c.boom_length - 5) < 1e-3 && abs(c.pitch_deg - 60) < 1e-3, "interior boom %v pitch %v", c.boom_length, c.pitch_deg)
    testing.expect(t, c.requested_zoom == 20)
    c.interior_mode = false
    for _ in 0 ..< 18 { engine.r3d_follow_step(&c, nil, {}, DT) }
    testing.expect(t, abs(c.boom_length - 20) < 1e-3 && abs(c.pitch_deg - 55) < 1e-3)
    testing.expect(t, engine.r3d_follow_clamp_zoom(3) == 8 && engine.r3d_follow_clamp_zoom(99) == 20)
}
