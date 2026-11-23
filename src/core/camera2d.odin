package voidengine

import "core:math"

// ============================================================================
// 2D follow camera — dead-zone, look-ahead, room bounds, trauma shake.
// World pixels, y DOWN. `center` is the point the view is centered on.
//
//   cam := camera2d_make(640, 360)
//   camera2d_set_bounds(&cam, room_rect)
//   camera2d_update(&cam, player_center, facing, dt)
//   origin := camera2d_view_origin(&cam)   // top-left, whole pixels, shake applied
// ============================================================================

Camera2D :: struct {
    center:          [2]f32,
    view:            [2]f32, // view size in world px
    deadzone:        [2]f32, // full width/height of the box the target may roam freely in
    lookahead_dist:  f32,    // px ahead of the target in its facing direction
    lookahead_speed: f32,    // px/s the look-ahead slides at
    lookahead:       f32,    // current look-ahead offset (x)
    smoothing:       f32,    // 0 = follow exactly; else exponential rate (1/s)
    bounds:          Rect2,
    use_bounds:      bool,
    trauma:          f32,    // 0..1; shake = trauma² * shake_max
    shake_max:       f32,    // px
    shake_decay:     f32,    // trauma lost per second
    shake_offset:    [2]f32,
    rng:             Rng,
}

camera2d_make :: proc "contextless" (view_w, view_h: f32) -> Camera2D {
    return Camera2D{
        view            = {view_w, view_h},
        deadzone        = {40, 56},
        lookahead_dist  = 40,
        lookahead_speed = 140,
        smoothing       = 0,
        shake_max       = 6,
        shake_decay     = 1.6,
        rng             = rng_make(0xF4A7),
    }
}

camera2d_set_bounds :: proc "contextless" (c: ^Camera2D, bounds: Rect2) {
    c.bounds = bounds
    c.use_bounds = true
    camera2d_clamp(c)
}

camera2d_clamp :: proc "contextless" (c: ^Camera2D) {
    if !c.use_bounds { return }
    hw := c.view.x * 0.5
    hh := c.view.y * 0.5
    if c.bounds.w <= c.view.x {
        c.center.x = c.bounds.x + c.bounds.w * 0.5
    } else {
        c.center.x = clampf(c.center.x, c.bounds.x + hw, c.bounds.x + c.bounds.w - hw)
    }
    if c.bounds.h <= c.view.y {
        c.center.y = c.bounds.y + c.bounds.h * 0.5
    } else {
        c.center.y = clampf(c.center.y, c.bounds.y + hh, c.bounds.y + c.bounds.h - hh)
    }
}

// Jump straight to a target (room entry, respawn) with no easing.
camera2d_snap_to :: proc "contextless" (c: ^Camera2D, target: [2]f32, facing: f32 = 0) {
    c.lookahead = facing * c.lookahead_dist
    c.center = target + {c.lookahead, 0}
    camera2d_clamp(c)
}

camera2d_update :: proc "contextless" (c: ^Camera2D, target: [2]f32, facing: f32, dt: f32) {
    c.lookahead = approach(c.lookahead, facing * c.lookahead_dist, c.lookahead_speed * dt)
    focus := target + {c.lookahead, 0}

    desired := c.center
    dzx := c.deadzone.x * 0.5
    dzy := c.deadzone.y * 0.5
    if focus.x > c.center.x + dzx {
        desired.x = focus.x - dzx
    } else if focus.x < c.center.x - dzx {
        desired.x = focus.x + dzx
    }
    if focus.y > c.center.y + dzy {
        desired.y = focus.y - dzy
    } else if focus.y < c.center.y - dzy {
        desired.y = focus.y + dzy
    }

    if c.smoothing > 0 {
        k := 1 - math.exp(-c.smoothing * dt)
        c.center += (desired - c.center) * k
    } else {
        c.center = desired
    }
    camera2d_clamp(c)

    // shake
    c.trauma = max(c.trauma - c.shake_decay * dt, 0)
    amount := c.trauma * c.trauma * c.shake_max
    if amount > 0 {
        c.shake_offset = {rng_range_f(&c.rng, -1, 1) * amount, rng_range_f(&c.rng, -1, 1) * amount}
    } else {
        c.shake_offset = {0, 0}
    }
}

camera2d_add_trauma :: proc "contextless" (c: ^Camera2D, amount: f32) {
    c.trauma = clampf(c.trauma + amount, 0, 1)
}

// Top-left of the view in world px, shake applied, floored to whole pixels
// so pixel art never shimmers.
camera2d_view_origin :: proc "contextless" (c: ^Camera2D) -> [2]f32 {
    o := c.center - c.view * 0.5 + c.shake_offset
    return {math.floor(o.x), math.floor(o.y)}
}

camera2d_view_rect :: proc "contextless" (c: ^Camera2D) -> Rect2 {
    o := camera2d_view_origin(c)
    return Rect2{o.x, o.y, c.view.x, c.view.y}
}
