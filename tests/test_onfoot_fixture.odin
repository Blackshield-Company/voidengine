package voidengine_test

// Templar on-foot acceptance (roadmap 3D-3 / T-PARITY on-foot part) as
// deterministic sim-time routes on the Godot OnFootFixture course rebuilt in
// examples/templar_valley/onfoot: the fixture route (speeds, stops, kb vs
// pad, half stick, non-walkable slope, wall/overhang boom), the 25 deg
// hillside camera clearance (F2) with a 50 deg bank, and occlusion/interior.

import "core:testing"
import onfoot "../examples/templar_valley/onfoot"

@(private="file")
run_and_expect :: proc(t: ^testing.T, route: string) {
    fx: onfoot.Fixture
    onfoot.fixture_init(&fx)
    defer onfoot.fixture_destroy(&fx)
    r: onfoot.Run
    defer onfoot.run_destroy(&r)
    testing.expect(t, onfoot.run_route(&r, &fx, route))
    testing.expect(t, len(r.checks) > 8)
    for c in r.checks {
        testing.expectf(t, c.ok, "%s %s: %s (want %s)", route, c.name, c.value, c.threshold)
    }
}

@(test) test_onfoot_fixture_route :: proc(t: ^testing.T) { run_and_expect(t, "fixture") }
@(test) test_onfoot_hill_route :: proc(t: ^testing.T) { run_and_expect(t, "hill") }
@(test) test_onfoot_occlusion_route :: proc(t: ^testing.T) { run_and_expect(t, "occlusion") }

@(test)
test_onfoot_route_is_deterministic :: proc(t: ^testing.T) {
    fx: onfoot.Fixture
    onfoot.fixture_init(&fx)
    defer onfoot.fixture_destroy(&fx)
    a, b: onfoot.Run
    defer { onfoot.run_destroy(&a); onfoot.run_destroy(&b) }
    onfoot.run_route(&a, &fx, "fixture")
    onfoot.run_route(&b, &fx, "fixture")
    testing.expect_value(t, len(a.rows), len(b.rows))
    same := true
    for i in 0 ..< min(len(a.rows), len(b.rows)) {
        if a.rows[i].pos != b.rows[i].pos || a.rows[i].cam != b.rows[i].cam { same = false; break }
    }
    testing.expect(t, same)
}
