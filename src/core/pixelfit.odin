package voidengine

// ============================================================================
// Pixel-perfect scaling: fit a fixed internal resolution (e.g. 640x360) into
// a window at the largest whole-number scale, centered with black bars.
// Pure math — used by gl3d_set_present and by SDL_Renderer games alike.
// ============================================================================

Pixel_Fit :: struct {
    scale: i32, // whole-number scale (>= 1)
    x, y:  i32, // top-left of the scaled image inside the window (y from top)
    w, h:  i32, // scaled image size
}

pixel_fit :: proc "contextless" (win_w, win_h, base_w, base_h: i32) -> Pixel_Fit {
    if base_w <= 0 || base_h <= 0 {
        return Pixel_Fit{scale = 1, w = win_w, h = win_h}
    }
    s := min(win_w / base_w, win_h / base_h)
    if s < 1 { s = 1 }
    w := base_w * s
    h := base_h * s
    return Pixel_Fit{scale = s, x = (win_w - w) / 2, y = (win_h - h) / 2, w = w, h = h}
}

// Map a window-space point (e.g. the mouse) to internal pixels. ok is false
// when the point falls in the letterbox bars.
pixel_fit_to_internal :: proc "contextless" (fit: Pixel_Fit, wx, wy: i32) -> (ix, iy: i32, ok: bool) {
    if fit.scale <= 0 { return 0, 0, false }
    lx := wx - fit.x
    ly := wy - fit.y
    if lx < 0 || ly < 0 || lx >= fit.w || ly >= fit.h { return 0, 0, false }
    return lx / fit.scale, ly / fit.scale, true
}

// contextless f32 clamp (the package-level clamp in engine.odin needs a context)
clampf :: #force_inline proc "contextless" (v, lo, hi: f32) -> f32 {
    return v < lo ? lo : (v > hi ? hi : v)
}
