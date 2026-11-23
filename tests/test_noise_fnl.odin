package voidengine_test

// Tests for noise_fnl.odin against golden values printed by Godot 4.7.2's own
// FastNoiseLite (examples/templar3d/tools/bake_valley.gd), using Templar
// Valley's three noises: height (seed 1165, 0.004, FBm 4 octaves), forest edge
// (seed 2207, 0.01, 1D, no fractal) and forest patches (seed 3313, 0.006).

import "core:math"
import "core:testing"
import engine "../src/core"

@(private="file")
Gold :: struct { x, z: f32, h: f64, n, e, p: f32 }

@(private="file")
GOLD := [?]Gold{
    {0, 0, 8.000000000, 0.000000000, -0.000000000, 0.000000000},
    {120, -562, 14.050841908, 0.001854565, -0.216188803, -0.237876520},
    {-731.25, 404.5, 101.359403326, 0.157795608, -0.552559435, -0.489679605},
    {1023, -1023, 157.080145412, -0.038898982, 0.472483724, -0.174426630},
    {13.7, 999.1, 0.210491985, -0.445646286, 0.413912356, 0.222700700},
    {-0.3, -0.7, 8.209009871, 0.011569872, 0.015350740, 0.008404677},
    {555.5, -77.25, 57.510683895, -0.579028308, -0.496052265, -0.261331111},
    {-1024, 1024, 158.502901718, 0.027938984, 0.633521676, 0.281908184},
}

@(test)
test_fnl_matches_godot :: proc(t: ^testing.T) {
    hn := engine.fnl_godot(1165, 0.004, .FBm, 4)
    en := engine.fnl_godot(2207, 0.01, .None)
    pn := engine.fnl_godot(3313, 0.006, .None)
    for g in GOLD {
        n := engine.fnl_noise_2d(&hn, g.x, g.z)
        e := engine.fnl_noise_1d(&en, g.z)
        p := engine.fnl_noise_2d(&pn, g.x, g.z)
        testing.expectf(t, abs(n - g.n) < 2e-7, "height noise at (%v,%v): %v want %v", g.x, g.z, n, g.n)
        testing.expectf(t, abs(e - g.e) < 2e-7, "edge noise at %v: %v want %v", g.z, e, g.e)
        testing.expectf(t, abs(p - g.p) < 2e-7, "patch noise at (%v,%v): %v want %v", g.x, g.z, p, g.p)
        // ValleyHeight.height_at (GDScript doubles around the f32 noise)
        h := 8.0 + 150.0 * math.pow(abs(f64(g.x)) / 1024.0, 1.5) + f64(n) * 18.0
        testing.expectf(t, abs(h - g.h) < 1e-5, "height at (%v,%v): %v want %v", g.x, g.z, h, g.h)
    }
}

@(test)
test_fnl_range_and_continuity :: proc(t: ^testing.T) {
    hn := engine.fnl_godot(1165, 0.004, .FBm, 4)
    lo, hi: f32 = 10, -10
    prev := engine.fnl_noise_2d(&hn, -500, 3)
    for i in 1 ..< 4000 {
        x := -500 + f32(i) * 0.25
        v := engine.fnl_noise_2d(&hn, x, 3)
        lo = min(lo, v); hi = max(hi, v)
        testing.expect(t, abs(v - prev) < 0.02) // smooth at 0.25 m steps
        prev = v
    }
    testing.expect(t, lo > -1 && hi < 1 && hi - lo > 0.3)
}
