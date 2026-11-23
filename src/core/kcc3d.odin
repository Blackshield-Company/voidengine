package voidengine

// ============================================================================
// kcc3d — kinematic capsule character controller (on foot). Port of Templar's
// Godot PlayerController movement: camera-relative walk 1.6 / run 4.5 m/s,
// acceleration 20 and braking 30 m/s^2, gravity 9.8 capped at 50 m/s, 45 deg
// walkable limit, 0.6 m floor snap, constant horizontal speed on slopes.
// On top of Godot's CharacterBody3D it adds an explicit step-up (0.35 m) that
// only accepts a walkable landing, so a non-walkable slope never becomes a
// staircase. Steep faces met while grounded act as vertical walls.
// The capsule's feet are at `pos`; the core segment runs from pos + r to
// pos + height - r.
// ============================================================================

import "core:math"
import "core:math/linalg"

R3D_Kcc :: struct {
    pos:            [3]f32, // feet
    vel:            [3]f32,
    radius, height: f32,
    walk_speed, run_speed: f32,
    accel, decel:   f32,
    gravity, max_fall: f32,
    max_floor_deg:  f32,
    snap_len:       f32,
    step_height:    f32,
    skin:           f32,
    mask:           u32,
    // state
    on_floor:       bool,
    floor_n:        [3]f32,
    air_s:          f32,
    wish:           [3]f32,
    // stats (cumulative)
    steps_up:       int,
    wall_hits:      int,
    depen_m:        f32,
}

r3d_kcc_default :: proc() -> R3D_Kcc {
    return {
        radius = 0.4, height = 1.8,
        walk_speed = 1.6, run_speed = 4.5,
        accel = 20, decel = 30,
        gravity = 9.8, max_fall = 50,
        max_floor_deg = 45, snap_len = 0.6, step_height = 0.35, skin = 0.01,
        mask = R3D_COL_PLAYER, floor_n = {0, 1, 0},
    }
}

r3d_kcc_segment :: #force_inline proc "contextless" (k: ^R3D_Kcc, at: [3]f32) -> (p, q: [3]f32) {
    return at + {0, k.radius, 0}, at + {0, k.height - k.radius, 0}
}

// Radial deadzone with rescale (Godot PlayerInput.radial_deadzone, 0.2).
r3d_radial_deadzone :: proc "contextless" (raw: [2]f32, deadzone: f32 = 0.2) -> [2]f32 {
    v := raw
    l := linalg.length(v)
    if l > 1 { v /= l; l = 1 }
    if !(l > deadzone + 1e-6) { return {} }
    return v / l * min(1, (l - deadzone) / (1 - deadzone))
}

// World direction of a camera-relative request (x right, y forward) at the
// camera's displayed yaw (Godot: Basis(UP, yaw) * (x, 0, -y)); length kept.
r3d_move_direction :: proc "contextless" (move: [2]f32, yaw: f32) -> [3]f32 {
    s, c := math.sin(yaw), math.cos(yaw)
    x, z := move.x, -move.y
    return {c * x + s * z, 0, -s * x + c * z}
}

r3d_kcc_wish :: proc "contextless" (k: ^R3D_Kcc, move: [2]f32, running: bool, yaw: f32) -> [3]f32 {
    m := move
    l := linalg.length(m)
    if l > 1 { m /= l }
    return r3d_move_direction(m, yaw) * (running ? k.run_speed : k.walk_speed)
}

r3d_kcc_walkable :: #force_inline proc "contextless" (k: ^R3D_Kcc, n: [3]f32) -> bool {
    return n.y >= math.cos(math.to_radians(k.max_floor_deg)) - 1e-4
}

// Horizontal velocity after one step toward the wish (Godot next_horizontal).
r3d_kcc_next_horizontal :: proc "contextless" (k: ^R3D_Kcc, current, wish: [3]f32, dt: f32) -> [3]f32 {
    rate := k.accel
    if linalg.length(wish) < linalg.length(current) - 0.001 || linalg.dot(wish, current) < 0 { rate = k.decel }
    d := wish - current
    l := linalg.length(d)
    step := rate * dt
    if l <= step || l < 1e-9 { return wish }
    return current + d / l * step
}

@(private="file")
kcc_sweep :: proc(k: ^R3D_Kcc, w: ^R3D_Collision_World, at, motion: [3]f32) -> R3D_Sweep_Hit {
    p, q := r3d_kcc_segment(k, at)
    return r3d_sweep_capsule(w, p, q, k.radius, motion, k.mask, k.skin)
}

// Horizontal motion along a walkable floor plane, horizontal length kept.
@(private="file")
along_floor :: proc "contextless" (m, n: [3]f32) -> [3]f32 {
    if n.y < 1e-3 { return m }
    return {m.x, -(n.x * m.x + n.z * m.z) / n.y, m.z}
}

// Try climbing a ledge: up by step_height, across, then down onto a walkable
// surface. Returns the new feet position.
@(private="file")
try_step :: proc(k: ^R3D_Kcc, w: ^R3D_Collision_World, at, horiz: [3]f32) -> (to: [3]f32, ok: bool) {
    if linalg.length2(horiz) < 1e-10 { return }
    up := kcc_sweep(k, w, at, {0, k.step_height, 0})
    lift := k.step_height * up.t
    if lift < 0.05 { return }
    raised := at + {0, lift, 0}
    // at least half a radius of travel, so the landing contact is on the
    // ledge top rather than its lip (a lip contact is not walkable)
    hl := linalg.length(horiz)
    probe := horiz * max(1, (k.radius * 0.5) / hl)
    fwd := kcc_sweep(k, w, raised, probe)
    moved := probe * fwd.t
    if linalg.length(moved) < 0.005 { return }
    across := raised + moved
    down := kcc_sweep(k, w, across, {0, -(lift + 0.05), 0})
    if !down.hit || !r3d_kcc_walkable(k, down.normal) { return }
    land := across + {0, -(lift + 0.05) * down.t, 0}
    if land.y - at.y < 0.02 { return } // no real ledge: let the slide handle it
    return land, true
}

// One fixed step. wish = requested horizontal velocity (world).
r3d_kcc_step :: proc(k: ^R3D_Kcc, w: ^R3D_Collision_World, wish: [3]f32, dt: f32) {
    if dt <= 0 { return }
    k.wish = wish
    h := r3d_kcc_next_horizontal(k, {k.vel.x, 0, k.vel.z}, {wish.x, 0, wish.z}, dt)
    k.vel.x, k.vel.z = h.x, h.z
    was_on_floor := k.on_floor
    if was_on_floor { k.vel.y = 0 } else { k.vel.y = max(k.vel.y - k.gravity * dt, -k.max_fall) }

    // start clear of geometry
    {
        p, q := r3d_kcc_segment(k, k.pos)
        fix := r3d_capsule_depenetrate(w, p, q, k.radius, k.mask, k.skin)
        if linalg.length2(fix) > 0 { k.pos += fix; k.depen_m += linalg.length(fix) }
    }

    motion := k.vel * dt
    if was_on_floor && r3d_kcc_walkable(k, k.floor_n) { motion = along_floor(motion, k.floor_n) }
    grounded := was_on_floor
    k.on_floor = false
    for _ in 0 ..< 5 {
        if linalg.length2(motion) < 1e-12 { break }
        hit := kcc_sweep(k, w, k.pos, motion)
        k.pos += motion * hit.t
        if !hit.hit { break }
        rem := motion * (1 - hit.t)
        n := hit.normal
        if r3d_kcc_walkable(k, n) {
            k.on_floor = true
            k.floor_n = n
            grounded = true
            if k.vel.y < 0 { k.vel.y = 0 }
            motion = along_floor({rem.x, 0, rem.z}, n)
            continue
        }
        // steep face or wall: try a step first when grounded
        horiz := [3]f32{rem.x, 0, rem.z}
        if grounded && n.y > -0.2 {
            if to, ok := try_step(k, w, k.pos, horiz); ok {
                k.pos = to
                k.steps_up += 1
                k.on_floor = true
                motion = {}
                break
            }
        }
        k.wall_hits += 1
        nn := n
        if grounded && n.y > 0 {
            // a steep slope met on foot is a wall: never climb it
            hn := [3]f32{n.x, 0, n.z}
            if linalg.length2(hn) > 1e-8 { nn = linalg.normalize(hn) }
        }
        if d := linalg.dot(rem, nn); d < 0 { rem -= nn * d }
        if vd := linalg.dot(k.vel, nn); vd < 0 { k.vel -= nn * vd }
        if n.y < -0.1 && k.vel.y > 0 { k.vel.y = 0 } // ceiling
        motion = rem
    }

    // floor probe / snap
    {
        reach := was_on_floor && k.vel.y <= 0 ? k.snap_len : 2 * k.skin + 0.01
        hit := kcc_sweep(k, w, k.pos, {0, -reach, 0})
        if hit.hit && r3d_kcc_walkable(k, hit.normal) && k.vel.y <= 0.001 {
            k.pos.y -= reach * hit.t
            k.on_floor = true
            k.floor_n = hit.normal
            k.vel.y = 0
        } else if !k.on_floor {
            k.floor_n = {0, 1, 0}
        }
    }
    if k.on_floor { k.air_s = 0 } else { k.air_s += dt }
}

r3d_kcc_horizontal_speed :: proc "contextless" (k: ^R3D_Kcc) -> f32 { return linalg.length([2]f32{k.vel.x, k.vel.z}) }

// Places the feet at `at` (zero velocity) and settles onto the floor below.
r3d_kcc_place :: proc(k: ^R3D_Kcc, w: ^R3D_Collision_World, at: [3]f32) {
    k.pos = at + {0, 0.05, 0}
    k.vel = {}
    k.air_s = 0
    hit := kcc_sweep(k, w, k.pos, {0, -1.0, 0})
    if hit.hit { k.pos.y -= 1.0 * hit.t; k.on_floor = r3d_kcc_walkable(k, hit.normal); k.floor_n = hit.normal }
}
