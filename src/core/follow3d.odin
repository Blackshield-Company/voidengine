package voidengine

// ============================================================================
// follow3d — Templar's on-foot elevated three-quarter follow camera (port of
// the Godot FollowCamera at 1356930): fixed 55 deg pitch, requested zoom
// 8-20 m, 45 deg yaw snaps settling in 0.2 s (one queued), exact first-order
// follow lag (0.2 s) capped at 2.5 m, and collision safety:
//   * a 0.35 m sphere is swept head -> lagged pivot and pivot -> boom end
//     (a head that starts inside a low ceiling is first lowered toward the
//     body), with a 2 cm gap;
//   * the boom is shortened further (bisection) until the camera has 0.5 m
//     (+3 cm margin) over the ground below it;
//   * retraction is immediate, the return is smooth (rate 4/s, >= 1 m/s).
// Interior mode (set by the occlusion director) blends the boom to <= 5 m
// and the pitch to 60 deg over 0.3 s. Requested and effective distances are
// kept apart. Riding is 3D-4 and is not part of this file.
// ============================================================================

import "core:math"
import "core:math/linalg"

R3D_FOLLOW_PITCH_DEG :: f32(55)
R3D_FOLLOW_ZOOM_MIN :: f32(8)
R3D_FOLLOW_ZOOM_MAX :: f32(20)
R3D_FOLLOW_ZOOM_DEFAULT :: f32(14)
R3D_FOLLOW_SNAP_DEG :: f32(45)

R3D_Follow_Camera :: struct {
    snap_time_s, lag_time_s, max_separation_m: f32,
    pivot_height_m, probe_radius_m, body_probe_height_m: f32,
    min_ground_clearance_m, clearance_margin_m: f32,
    zoom_smooth_time_s, boom_return_rate, boom_return_min_mps: f32,
    interior_distance_m, interior_pitch_deg, interior_blend_s: f32,
    mask: u32,
    probe_enabled: bool,

    requested_zoom, zoom_current: f32,
    effective_distance, safe_distance, boom_length: f32,
    pitch_deg: f32,
    interior_mode: bool,
    interior_blend: f32,
    pivot: [3]f32,
    snaps_started: int,

    yaw: f32,
    snap_index: int,
    snap_from, snap_to, snap_t: f32,
    pending: int,
    focus, prev_target: [3]f32,
    initialized: bool,
}

r3d_follow_default :: proc() -> R3D_Follow_Camera {
    c := R3D_Follow_Camera{
        snap_time_s = 0.2, lag_time_s = 0.2, max_separation_m = 2.5,
        pivot_height_m = 1.6, probe_radius_m = 0.35, body_probe_height_m = 0.9,
        min_ground_clearance_m = 0.5, clearance_margin_m = 0.03,
        zoom_smooth_time_s = 0.12, boom_return_rate = 4, boom_return_min_mps = 1,
        interior_distance_m = 5, interior_pitch_deg = 60, interior_blend_s = 0.3,
        mask = R3D_COL_CAMERA, probe_enabled = true,
        requested_zoom = R3D_FOLLOW_ZOOM_DEFAULT, zoom_current = R3D_FOLLOW_ZOOM_DEFAULT,
        effective_distance = R3D_FOLLOW_ZOOM_DEFAULT, safe_distance = R3D_FOLLOW_ZOOM_DEFAULT,
        boom_length = R3D_FOLLOW_ZOOM_DEFAULT, pitch_deg = R3D_FOLLOW_PITCH_DEG,
        snap_t = -1,
    }
    return c
}

r3d_follow_clamp_zoom :: proc "contextless" (m: f32) -> f32 {
    return math.is_nan(m) || math.is_inf(m) ? R3D_FOLLOW_ZOOM_DEFAULT : math.clamp(m, R3D_FOLLOW_ZOOM_MIN, R3D_FOLLOW_ZOOM_MAX)
}
r3d_follow_set_zoom :: proc "contextless" (c: ^R3D_Follow_Camera, m: f32) { c.requested_zoom = r3d_follow_clamp_zoom(m) }
r3d_follow_add_zoom :: proc "contextless" (c: ^R3D_Follow_Camera, m: f32) { c.requested_zoom = r3d_follow_clamp_zoom(c.requested_zoom + m) }

r3d_wrap_pi :: proc "contextless" (a: f32) -> f32 {
    x := math.mod(a + math.PI, 2 * math.PI)
    if x < 0 { x += 2 * math.PI }
    return x - math.PI
}

r3d_follow_index_yaw :: proc "contextless" (index: int) -> f32 {
    return math.to_radians(R3D_FOLLOW_SNAP_DEG * f32(((index % 8) + 8) % 8))
}
r3d_follow_displayed_yaw :: proc "contextless" (c: ^R3D_Follow_Camera) -> f32 { return r3d_wrap_pi(c.yaw) }
r3d_follow_is_snapping :: proc "contextless" (c: ^R3D_Follow_Camera) -> bool { return c.snap_t >= 0 }
r3d_follow_snap_index :: proc "contextless" (c: ^R3D_Follow_Camera) -> int { return ((c.snap_index % 8) + 8) % 8 }

r3d_follow_set_yaw_index :: proc "contextless" (c: ^R3D_Follow_Camera, index: int) {
    c.snap_index = ((index % 8) + 8) % 8
    c.yaw = r3d_follow_index_yaw(c.snap_index)
    c.snap_t = -1
    c.pending = 0
}

@(private="file")
start_snap :: proc "contextless" (c: ^R3D_Follow_Camera, dir: int) {
    c.snap_from = c.yaw
    c.snap_index = (((c.snap_index + dir) % 8) + 8) % 8
    c.snap_to = c.snap_from + math.to_radians(R3D_FOLLOW_SNAP_DEG) * f32(dir)
    c.snap_t = 0
    c.snaps_started += 1
}

// One 45 deg step (+1 / -1); while a snap runs at most one more is queued.
r3d_follow_request_snap :: proc "contextless" (c: ^R3D_Follow_Camera, dir: int) {
    if dir == 0 { return }
    d := dir > 0 ? 1 : -1
    if c.snap_t < 0 { start_snap(c, d) } else { c.pending = d }
}

@(private="file")
smoothstep01 :: #force_inline proc "contextless" (u: f32) -> f32 {
    x := math.clamp(u, 0, 1)
    return x * x * (3 - 2 * x)
}

r3d_follow_advance_snap :: proc "contextless" (c: ^R3D_Follow_Camera, dt: f32) {
    if c.snap_t < 0 { return }
    c.snap_t += max(dt, 0)
    u := math.clamp(c.snap_t / c.snap_time_s, 0, 1)
    if u >= 1 {
        c.yaw = r3d_follow_index_yaw(c.snap_index)
        c.snap_t = -1
        if c.pending != 0 {
            nx := c.pending
            c.pending = 0
            start_snap(c, nx)
        }
        return
    }
    c.yaw = c.snap_from + (c.snap_to - c.snap_from) * smoothstep01(u)
}

// boom direction: Basis(UP, yaw) * (0, sin p, cos p)
r3d_follow_boom_direction :: proc "contextless" (yaw, pitch_deg: f32) -> [3]f32 {
    p := math.to_radians(pitch_deg)
    return {math.cos(p) * math.sin(yaw), math.sin(p), math.cos(p) * math.cos(yaw)}
}

r3d_follow_camera_position :: proc "contextless" (c: ^R3D_Follow_Camera) -> [3]f32 {
    return c.pivot + r3d_follow_boom_direction(r3d_follow_displayed_yaw(c), c.pitch_deg) * c.effective_distance
}

// Writes the camera pose (looks along -boom).
r3d_follow_apply :: proc "contextless" (c: ^R3D_Follow_Camera, cam: ^R3D_Camera) {
    boom := r3d_follow_boom_direction(r3d_follow_displayed_yaw(c), c.pitch_deg)
    cam.position = c.pivot + boom * c.effective_distance
    cam.pitch = -math.to_radians(c.pitch_deg)
    cam.yaw = math.atan2(-boom.x, boom.z)
}

r3d_follow_cap_separation :: proc "contextless" (e: [3]f32, cap: f32) -> [3]f32 {
    flat := [2]f32{e.x, e.z}
    l := linalg.length(flat)
    if l > cap { flat *= cap / l }
    return {flat.x, math.clamp(e.y, -cap, cap), flat.y}
}

r3d_follow_update_follow :: proc "contextless" (c: ^R3D_Follow_Camera, target: [3]f32, dt: f32) -> [3]f32 {
    if !c.initialized {
        c.focus = target; c.prev_target = target; c.initialized = true
        return c.focus
    }
    if dt <= 0 { return c.focus }
    vel := (target - c.prev_target) / dt
    r := math.exp(-dt / c.lag_time_s)
    steady := vel * c.lag_time_s
    err := steady + ((c.prev_target - c.focus) - steady) * r
    err = r3d_follow_cap_separation(err, c.max_separation_m)
    c.focus = target - err
    c.prev_target = target
    return c.focus
}

r3d_follow_advance_interior :: proc "contextless" (c: ^R3D_Follow_Camera, dt: f32) {
    goal: f32 = c.interior_mode ? 1 : 0
    if c.interior_blend != goal {
        step := max(dt, 0) / max(c.interior_blend_s, 0.0001)
        c.interior_blend = goal > c.interior_blend ? min(goal, c.interior_blend + step) : max(goal, c.interior_blend - step)
    }
    if c.interior_blend <= 0 {
        c.boom_length = c.zoom_current
        c.pitch_deg = R3D_FOLLOW_PITCH_DEG
        return
    }
    s := smoothstep01(c.interior_blend)
    c.boom_length = math.lerp(c.zoom_current, min(c.zoom_current, c.interior_distance_m), s)
    c.pitch_deg = math.lerp(R3D_FOLLOW_PITCH_DEG, c.interior_pitch_deg, s)
}

r3d_follow_is_interior :: proc "contextless" (c: ^R3D_Follow_Camera) -> bool { return c.interior_mode || c.interior_blend > 0 }

// Ground under a point: highest of the world below it and the visual terrain.
r3d_follow_ground_below :: proc(c: ^R3D_Follow_Camera, w: ^R3D_Collision_World, point: [3]f32) -> f32 {
    g := r3d_ground_below(w, point, c.mask)
    if w.hf != nil {
        // the visual terrain is the bilinear surface; the collider is its triangulation
        g = max(g, r3d_hf_height(w.hf, point.x, point.z), r3d_hf_height_tri(w.hf, point.x, point.z))
    }
    return g
}

r3d_follow_clearance_at :: proc(c: ^R3D_Follow_Camera, w: ^R3D_Collision_World, point: [3]f32) -> f32 {
    return point.y - r3d_follow_ground_below(c, w, point)
}

// Safe fraction of the probe sphere's motion (0 when it starts in contact).
r3d_follow_sweep :: proc(c: ^R3D_Follow_Camera, w: ^R3D_Collision_World, from, motion: [3]f32) -> f32 {
    if r3d_sphere_overlaps(w, from, c.probe_radius_m, c.mask) { return 0 }
    l := linalg.length(motion)
    if l < 1e-3 { return 1 }
    hit := r3d_sweep_sphere(w, from, c.probe_radius_m, motion, c.mask, 0)
    if !hit.hit { return 1 }
    return math.clamp(hit.t - 0.02 / l, 0, 1)
}

@(private="file")
limit_by_clearance :: proc(c: ^R3D_Follow_Camera, w: ^R3D_Collision_World, from, dir: [3]f32, distance: f32) -> f32 {
    need := c.min_ground_clearance_m + c.clearance_margin_m
    if r3d_follow_clearance_at(c, w, from + dir * distance) >= need { return distance }
    lo, hi: f32 = 0, distance
    for _ in 0 ..< 10 {
        mid := (lo + hi) * 0.5
        if r3d_follow_clearance_at(c, w, from + dir * mid) >= need { lo = mid } else { hi = mid }
    }
    return lo
}

r3d_follow_update_boom :: proc(c: ^R3D_Follow_Camera, w: ^R3D_Collision_World, dt: f32) {
    height := c.pivot_height_m
    head := c.prev_target + {0, height, 0}
    wanted := c.focus + {0, height, 0}
    dir := r3d_follow_boom_direction(r3d_follow_displayed_yaw(c), c.pitch_deg)
    if c.probe_enabled && w != nil {
        if r3d_sphere_overlaps(w, head, c.probe_radius_m, c.mask) {
            body := c.prev_target + {0, min(c.body_probe_height_m, height), 0}
            head = body + (head - body) * r3d_follow_sweep(c, w, body, head - body)
        }
        c.pivot = head + (wanted - head) * r3d_follow_sweep(c, w, head, wanted - head)
        c.safe_distance = c.boom_length * r3d_follow_sweep(c, w, c.pivot, dir * c.boom_length)
        c.safe_distance = limit_by_clearance(c, w, c.pivot, dir, c.safe_distance)
    } else {
        c.pivot = wanted
        c.safe_distance = c.boom_length
    }
    if dt <= 0 || c.safe_distance <= c.effective_distance {
        c.effective_distance = c.safe_distance
    } else {
        speed := max(c.boom_return_min_mps, (c.safe_distance - c.effective_distance) * c.boom_return_rate)
        c.effective_distance = min(c.safe_distance, c.effective_distance + speed * dt)
    }
}

// Starts aligned on the target (spawn / reset): no lag sweep, immediate boom.
r3d_follow_reset :: proc(c: ^R3D_Follow_Camera, w: ^R3D_Collision_World, target: [3]f32) {
    c.focus = target
    c.prev_target = target
    c.zoom_current = c.requested_zoom
    c.interior_mode = false
    c.interior_blend = 0
    r3d_follow_advance_interior(c, 0)
    c.initialized = true
    c.effective_distance = max(f32)
    r3d_follow_update_boom(c, w, 0)
}

r3d_follow_step :: proc(c: ^R3D_Follow_Camera, w: ^R3D_Collision_World, target: [3]f32, dt: f32) {
    if !c.initialized { r3d_follow_reset(c, w, target) }
    r3d_follow_advance_snap(c, dt)
    c.zoom_current += (c.requested_zoom - c.zoom_current) * (1 - math.exp(-max(dt, 0) / c.zoom_smooth_time_s))
    if abs(c.requested_zoom - c.zoom_current) < 0.001 { c.zoom_current = c.requested_zoom }
    r3d_follow_advance_interior(c, dt)
    r3d_follow_update_follow(c, target, dt)
    r3d_follow_update_boom(c, w, dt)
}
