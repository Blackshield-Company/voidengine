package voidengine

// ============================================================================
// Platformer controller helpers — the feel layer on top of body_move.
//
// Pixels and seconds, y DOWN. Gives you: acceleration/deceleration, coyote
// time (jump shortly after leaving a ledge), a jump buffer (press jump just
// before landing and it still fires), variable jump height (release early
// for a short hop), heavier fall gravity and a fall-speed cap.
//
//   delta, jumped := platformer_update(&params, &state, input, dt)
//   res := body_move(&grid, &body, delta, Move_Options{grounded = state.grounded, snap_down = 6})
//   landed := platformer_apply_move(&state, res)
// ============================================================================

Platformer_Params :: struct {
    run_speed:         f32, // px/s
    ground_accel:      f32, // px/s² toward the target speed
    ground_decel:      f32, // px/s² when there's no input
    air_accel:         f32, // px/s² in the air
    gravity:           f32, // px/s²
    fall_gravity_mult: f32, // gravity multiplier while falling (>1 = snappier)
    max_fall:          f32, // px/s
    jump_speed:        f32, // px/s upward at take-off
    jump_cut:          f32, // vel.y multiplier when jump is released while rising
    coyote_time:       f32, // s
    jump_buffer:       f32, // s
}

platformer_default_params :: proc "contextless" () -> Platformer_Params {
    return Platformer_Params{
        run_speed         = 130,
        ground_accel      = 1500,
        ground_decel      = 2000,
        air_accel         = 1000,
        gravity           = 1150,
        fall_gravity_mult = 1.55,
        max_fall          = 380,
        jump_speed        = 340,
        jump_cut          = 0.45,
        coyote_time       = 0.10,
        jump_buffer       = 0.12,
    }
}

Platformer_State :: struct {
    vel:          [2]f32,
    grounded:     bool,
    coyote_timer: f32,
    buffer_timer: f32,
    rising:       bool, // in the rising part of a player jump (jump cut allowed)
    facing:       f32,  // +1 right, -1 left
}

Platformer_Input :: struct {
    move_x:       f32,  // -1..1
    jump_pressed: bool, // this frame
    jump_held:    bool,
}

approach :: proc "contextless" (v, target, step: f32) -> f32 {
    if v < target { return min(v + step, target) }
    if v > target { return max(v - step, target) }
    return v
}

// Advance velocity for one step and return this step's displacement.
platformer_update :: proc "contextless" (p: ^Platformer_Params, s: ^Platformer_State, input: Platformer_Input, dt: f32) -> (delta: [2]f32, jumped: bool) {
    if s.facing == 0 { s.facing = 1 }

    if s.grounded {
        s.coyote_timer = p.coyote_time
    } else {
        s.coyote_timer = max(s.coyote_timer - dt, 0)
    }
    if input.jump_pressed {
        s.buffer_timer = p.jump_buffer
    } else {
        s.buffer_timer = max(s.buffer_timer - dt, 0)
    }

    mx := clampf(input.move_x, -1, 1)
    target := mx * p.run_speed
    accel: f32
    if s.grounded {
        accel = abs(mx) > 0.01 ? p.ground_accel : p.ground_decel
    } else {
        accel = p.air_accel
    }
    s.vel.x = approach(s.vel.x, target, accel * dt)
    if mx > 0.1 { s.facing = 1 } else if mx < -0.1 { s.facing = -1 }

    if s.buffer_timer > 0 && s.coyote_timer > 0 {
        s.vel.y = -p.jump_speed
        s.buffer_timer = 0
        s.coyote_timer = 0
        s.grounded = false
        s.rising = true
        jumped = true
    }

    if s.rising && !input.jump_held && s.vel.y < 0 {
        s.vel.y *= p.jump_cut
        s.rising = false
    }
    if s.vel.y >= 0 { s.rising = false }

    g := p.gravity
    if s.vel.y > 0 { g *= p.fall_gravity_mult }
    s.vel.y = min(s.vel.y + g * dt, p.max_fall)

    delta = s.vel * dt
    return
}

// Feed body_move's contacts back into the controller. Returns true on the
// step the body lands.
platformer_apply_move :: proc "contextless" (s: ^Platformer_State, res: Move_Result) -> (landed: bool) {
    was := s.grounded
    s.grounded = res.on_ground
    if res.on_ground && s.vel.y > 0 { s.vel.y = 0 }
    if res.hit_ceiling && s.vel.y < 0 {
        s.vel.y = 0
        s.rising = false
    }
    if (res.hit_left && s.vel.x < 0) || (res.hit_right && s.vel.x > 0) { s.vel.x = 0 }
    return !was && s.grounded
}

// Apex height (px) of a full-held jump with these params (for level design).
platformer_full_jump_height :: proc "contextless" (p: ^Platformer_Params) -> f32 {
    if p.gravity <= 0 { return 0 }
    return p.jump_speed * p.jump_speed / (2 * p.gravity)
}
