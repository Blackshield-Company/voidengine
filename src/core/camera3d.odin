package voidengine

// ============================================================================
// camera3d — free-fly and third-person orbit cameras that drive R3D_Camera.
// Pure math (unit-tested). Yaw/pitch follow r3d_camera_basis: yaw 0 looks
// down -Z, positive yaw turns toward +X, positive pitch looks up.
// ============================================================================

import "core:math"
import "core:math/linalg"

// Free-fly (debug / editor) camera.
R3D_Fly_Camera :: struct {
    position:    [3]f32,
    yaw, pitch:  f32,
    speed:       f32, // m/s
    fast_mult:   f32, // speed multiplier while "fast" is held
    look_sens:   f32, // radians per mouse pixel
}

r3d_fly_camera_default :: proc() -> R3D_Fly_Camera {
    return {speed = 6, fast_mult = 4, look_sens = 0.0025}
}

// move: x = strafe right, y = up, z = forward (each in -1..1). look: mouse
// delta in pixels (x right, y down).
r3d_fly_camera_update :: proc(c: ^R3D_Fly_Camera, move: [3]f32, look: [2]f32, fast: bool, dt: f32) {
    c.yaw += look.x * c.look_sens
    c.pitch -= look.y * c.look_sens
    c.pitch = clamp(c.pitch, -1.55, 1.55)
    tmp := R3D_Camera{yaw = c.yaw, pitch = c.pitch}
    right, _, forward := r3d_camera_basis(&tmp)
    v := right * move.x + [3]f32{0, 1, 0} * move.y + forward * move.z
    l := linalg.length(v)
    if l > 1 { v /= l }
    sp := c.speed * (fast ? c.fast_mult : 1)
    c.position += v * sp * dt
}

r3d_fly_camera_apply :: proc(c: ^R3D_Fly_Camera, cam: ^R3D_Camera) {
    cam.position = c.position
    cam.yaw = c.yaw
    cam.pitch = c.pitch
}

// Third-person orbit camera around a target (the knight's chest).
R3D_Orbit_Camera :: struct {
    target:        [3]f32,
    yaw:           f32, // camera heading (same convention as R3D_Camera.yaw)
    pitch:         f32, // elevation above the target, radians (positive = above)
    distance:      f32,
    min_pitch, max_pitch: f32,
    min_dist, max_dist:   f32,
    // smoothing: desired values approach at rate `damping` (1/s); 0 = snap
    damping:       f32,
    want_yaw, want_pitch, want_dist: f32,
    // ground clearance: camera never goes below ground_y + clearance
    ground_y:      f32,
    clearance:     f32,
}

r3d_orbit_camera_default :: proc() -> R3D_Orbit_Camera {
    c := R3D_Orbit_Camera{
        yaw = 0, pitch = math.to_radians(f32(25)), distance = 6,
        min_pitch = math.to_radians(f32(-10)), max_pitch = math.to_radians(f32(80)),
        min_dist = 1.5, max_dist = 30, damping = 12,
        ground_y = -1e9, clearance = 0.5,
    }
    c.want_yaw, c.want_pitch, c.want_dist = c.yaw, c.pitch, c.distance
    return c
}

// Feed input deltas (radians / metres) into the desired orbit.
r3d_orbit_camera_input :: proc(c: ^R3D_Orbit_Camera, d_yaw, d_pitch, d_dist: f32) {
    c.want_yaw += d_yaw
    c.want_pitch = clamp(c.want_pitch + d_pitch, c.min_pitch, c.max_pitch)
    c.want_dist = clamp(c.want_dist + d_dist, c.min_dist, c.max_dist)
}

r3d_orbit_camera_update :: proc(c: ^R3D_Orbit_Camera, dt: f32) {
    c.want_pitch = clamp(c.want_pitch, c.min_pitch, c.max_pitch)
    c.want_dist = clamp(c.want_dist, c.min_dist, c.max_dist)
    if c.damping <= 0 {
        c.yaw, c.pitch, c.distance = c.want_yaw, c.want_pitch, c.want_dist
        return
    }
    k := 1 - math.exp(-c.damping * dt)
    c.yaw += (c.want_yaw - c.yaw) * k
    c.pitch += (c.want_pitch - c.pitch) * k
    c.distance += (c.want_dist - c.distance) * k
}

// Camera world position for the current orbit (before ground clearance).
r3d_orbit_camera_eye :: proc(c: ^R3D_Orbit_Camera) -> [3]f32 {
    // the camera looks along forward(yaw, -pitch); it sits behind the target
    tmp := R3D_Camera{yaw = c.yaw, pitch = -c.pitch}
    _, _, fwd := r3d_camera_basis(&tmp)
    return c.target - fwd * c.distance
}

// Write position/yaw/pitch into an R3D_Camera so it looks at the target. If
// the eye would drop below ground_y + clearance it is raised, and the pitch
// re-aimed so the target stays centred.
r3d_orbit_camera_apply :: proc(c: ^R3D_Orbit_Camera, cam: ^R3D_Camera) {
    eye := r3d_orbit_camera_eye(c)
    floor := c.ground_y + c.clearance
    if eye.y < floor { eye.y = floor }
    d := c.target - eye
    horiz := math.sqrt(d.x * d.x + d.z * d.z)
    cam.position = eye
    cam.yaw = math.atan2(d.x, -d.z)
    cam.pitch = math.atan2(d.y, max(horiz, 1e-6))
}
