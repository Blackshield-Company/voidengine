package voidengine_test

// Tests for camera3d.odin: orbit camera aims at its target from the asked
// distance/elevation, keeps ground clearance, damps toward input, clamps
// pitch/zoom; free-fly camera moves along its view.

import "core:math"
import "core:testing"
import "core:math/linalg"
import engine "../src/core"

@(test)
test_orbit_camera_looks_at_target :: proc(t: ^testing.T) {
    o := engine.r3d_orbit_camera_default()
    o.damping = 0
    o.target = {3, 1.5, -2}
    o.want_yaw = 0.7
    o.want_pitch = math.to_radians(f32(30))
    o.want_dist = 8
    engine.r3d_orbit_camera_update(&o, 0.016)
    cam: engine.R3D_Camera
    engine.r3d_orbit_camera_apply(&o, &cam)
    testing.expect(t, near(linalg.length(cam.position - o.target), 8, 1e-3))
    _, _, fwd := engine.r3d_camera_basis(&cam)
    to_target := linalg.normalize(o.target - cam.position)
    testing.expect(t, linalg.dot(fwd, to_target) > 0.9999)
    testing.expect(t, near(cam.pitch, -math.to_radians(f32(30)), 1e-3)) // looking down
    testing.expect(t, cam.position.y > o.target.y)
}

@(test)
test_orbit_camera_ground_clearance :: proc(t: ^testing.T) {
    o := engine.r3d_orbit_camera_default()
    o.damping = 0
    o.min_pitch = math.to_radians(f32(-40))
    o.target = {0, 1, 0}
    o.want_pitch = math.to_radians(f32(-30)) // would go under the ground
    o.want_dist = 6
    o.ground_y = 0
    o.clearance = 0.5
    engine.r3d_orbit_camera_update(&o, 0.016)
    cam: engine.R3D_Camera
    engine.r3d_orbit_camera_apply(&o, &cam)
    testing.expect(t, cam.position.y >= 0.5 - 1e-5)
    _, _, fwd := engine.r3d_camera_basis(&cam)
    testing.expect(t, linalg.dot(fwd, linalg.normalize(o.target - cam.position)) > 0.9999)
}

@(test)
test_orbit_camera_clamps_and_damps :: proc(t: ^testing.T) {
    o := engine.r3d_orbit_camera_default()
    engine.r3d_orbit_camera_input(&o, 0, 10, 100) // way past the limits
    testing.expect(t, near(o.want_pitch, o.max_pitch))
    testing.expect(t, near(o.want_dist, o.max_dist))
    d0 := o.distance
    engine.r3d_orbit_camera_update(&o, 1.0 / 60)
    testing.expect(t, o.distance > d0 && o.distance < o.max_dist) // moving, not snapped
    for _ in 0 ..< 600 { engine.r3d_orbit_camera_update(&o, 1.0 / 60) }
    testing.expect(t, near(o.distance, o.max_dist, 1e-3))
    testing.expect(t, near(o.pitch, o.max_pitch, 1e-4))
}

@(test)
test_fly_camera_moves_along_view :: proc(t: ^testing.T) {
    f := engine.r3d_fly_camera_default()
    f.speed = 2
    engine.r3d_fly_camera_update(&f, {0, 0, 1}, {0, 0}, false, 0.5) // yaw 0: forward is -Z
    testing.expect(t, near3(f.position, {0, 0, -1}))
    engine.r3d_fly_camera_update(&f, {1, 0, 0}, {0, 0}, true, 0.25) // fast strafe right (+X)
    testing.expect(t, near3(f.position, {2, 0, -1}))
    // looking: mouse right turns toward +X, mouse down looks down; pitch clamps
    engine.r3d_fly_camera_update(&f, {}, {100, 100000}, false, 0)
    testing.expect(t, f.yaw > 0)
    testing.expect(t, near(f.pitch, -1.55))
    cam: engine.R3D_Camera
    engine.r3d_fly_camera_apply(&f, &cam)
    testing.expect(t, near3(cam.position, f.position) && near(cam.yaw, f.yaw))
}
