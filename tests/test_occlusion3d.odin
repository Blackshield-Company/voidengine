package voidengine_test

// Tests for occlusion3d.odin (Templar OcclusionDirector/Fader port): wall
// fades and roof cuts with their timings, readability on the thin lines,
// interior entry/exit with hysteresis, the active cap, terrain blocking.

import "core:testing"
import engine "../src/core"

@(private="file")
DT :: f32(1.0 / 60)

@(test)
test_segment_hits_box :: proc(t: ^testing.T) {
    testing.expect(t, engine.r3d_segment_hits_box({-5, 0.5, 0.5}, {5, 0.5, 0.5}, {0, 0, 0}, {1, 1, 1}))
    testing.expect(t, !engine.r3d_segment_hits_box({-5, 2, 0.5}, {5, 2, 0.5}, {0, 0, 0}, {1, 1, 1}))
    testing.expect(t, engine.r3d_segment_hits_box({-5, 1.2, 0.5}, {5, 1.2, 0.5}, {0, 0, 0}, {1, 1, 1}, 0.3))
    testing.expect(t, !engine.r3d_segment_hits_box({-5, 0.5, 0.5}, {-1, 0.5, 0.5}, {0, 0, 0}, {1, 1, 1}))
}

@(test)
test_occlusion_fade_cut_and_readability :: proc(t: ^testing.T) {
    o: engine.R3D_Occlusion
    engine.r3d_occlusion_init(&o)
    defer engine.r3d_occlusion_destroy(&o)
    wall := engine.r3d_occlusion_add(&o, {-3, 0, 2}, {3, 3, 2.3}, .Wall, 7)
    roof := engine.r3d_occlusion_add(&o, {-3.4, 3, -2}, {3.4, 3.5, 2.7}, .Roof, 7)
    tree := engine.r3d_occlusion_add(&o, {20, 2, 20}, {24, 6, 24}, .Canopy)
    cam := [3]f32{0, 10, 10}
    feet := [3]f32{0, 0, 0} // camera behind the wall (south), player north of it
    engine.r3d_occlusion_step(&o, DT, cam, feet)
    testing.expect(t, !o.readable && o.lines_blocked >= 1)
    a, hidden := engine.r3d_occlusion_alpha(&o, i32(wall))
    testing.expect(t, a < 1 && !hidden)
    steps := 1
    for !o.readable && steps < 60 { engine.r3d_occlusion_step(&o, DT, cam, feet); steps += 1 }
    // a cut roof counts as cleared only when hidden (0.15 s), a fade at amount 0.5
    testing.expectf(t, f32(steps) * DT <= 0.2, "readable after %v s", f32(steps) * DT)
    _, hidden = engine.r3d_occlusion_alpha(&o, i32(roof))
    testing.expect(t, hidden)
    a, _ = engine.r3d_occlusion_alpha(&o, i32(wall))
    testing.expect(t, abs(a - 0.3) < 1e-4)
    a, _ = engine.r3d_occlusion_alpha(&o, i32(tree))
    testing.expect(t, a == 1)
    faded, cut := engine.r3d_occlusion_cleared_counts(&o)
    testing.expect(t, faded == 1 && cut == 1)
    // walk out: held 0.2 s, then fades back over 0.15 s and is released
    feet = {10, 0, 10}
    for _ in 0 ..< 30 { engine.r3d_occlusion_step(&o, DT, cam, feet) }
    a, hidden = engine.r3d_occlusion_alpha(&o, i32(wall))
    testing.expect(t, a == 1 && !hidden && len(o.active) == 0)
}

@(test)
test_occlusion_interior_and_cap :: proc(t: ^testing.T) {
    o: engine.R3D_Occlusion
    engine.r3d_occlusion_init(&o)
    defer engine.r3d_occlusion_destroy(&o)
    roof := engine.r3d_occlusion_add(&o, {-5, 4, -5}, {5, 4.5, 5}, .Roof, 3)
    engine.r3d_occlusion_add_interior(&o, {-4.6, 0, -4.6}, {4.6, 4, 4.6}, 3)
    cam := [3]f32{0, 3, 2}
    testing.expect(t, !engine.r3d_occlusion_step(&o, DT, cam, {6, 0, 0}))
    testing.expect(t, engine.r3d_occlusion_step(&o, DT, cam, {0, 0, 0}))
    testing.expect(t, o.interior_entries == 1)
    // the interior's roof is cut even though no line touches it
    for _ in 0 ..< 12 { engine.r3d_occlusion_step(&o, DT, cam, {0, 0, 0}) }
    _, hidden := engine.r3d_occlusion_alpha(&o, i32(roof))
    testing.expect(t, hidden)
    // hysteresis: 0.3 m past the wall is still inside, 0.5 m is out
    testing.expect(t, engine.r3d_occlusion_step(&o, DT, cam, {4.9, 0, 0}))
    testing.expect(t, !engine.r3d_occlusion_step(&o, DT, cam, {5.1, 0, 0}))
    testing.expect(t, o.interior_exits == 1)
    // cap: at most max_active entries at once
    o2: engine.R3D_Occlusion
    engine.r3d_occlusion_init(&o2)
    defer engine.r3d_occlusion_destroy(&o2)
    o2.max_active = 4
    for i in 0 ..< 8 { engine.r3d_occlusion_add(&o2, {-3, 0, f32(i) + 1}, {3, 3, f32(i) + 1.2}, .Wall) }
    engine.r3d_occlusion_step(&o2, DT, {0, 2, 12}, {0, 0, 0})
    testing.expect(t, len(o2.active) == 4 && o2.refused == 4)
}

@(test)
test_occlusion_terrain_blocks :: proc(t: ^testing.T) {
    hf := engine.r3d_heightfield_make(33, 1, {-16, -16})
    defer engine.r3d_heightfield_destroy(&hf)
    for j in 0 ..< 33 { for i in 0 ..< 33 { if j == 20 { hf.heights[j * 33 + i] = 8 } } } // ridge at z = 4
    o: engine.R3D_Occlusion
    engine.r3d_occlusion_init(&o)
    defer engine.r3d_occlusion_destroy(&o)
    o.hf = &hf
    engine.r3d_occlusion_step(&o, DT, {0, 3, 10}, {0, 0, 0})
    testing.expect(t, o.terrain_blocked && !o.readable)
    engine.r3d_occlusion_step(&o, DT, {0, 20, 10}, {0, 0, 0})
    testing.expect(t, !o.terrain_blocked && o.readable)
}
