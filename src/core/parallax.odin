package voidengine

import "core:math"

// ============================================================================
// Parallax layers. factor 0 = glued to the screen (sky), 1 = moves with the
// world (the play layer), between = distant scenery. Works in world space
// (for gl3d sprites) or screen space (for SDL_Renderer).
// ============================================================================

Parallax_Layer :: struct {
    factor:   [2]f32, // scroll factor per axis
    offset:   [2]f32, // where the layer's origin sits when the camera is at 0,0
    width:    f32,    // image width, for horizontal repeat (0 = no repeat)
}

// Screen-space position of the layer's origin for a camera top-left.
parallax_screen_pos :: proc "contextless" (l: Parallax_Layer, cam_origin: [2]f32) -> [2]f32 {
    return l.offset - cam_origin * l.factor
}

// World-space position of the layer's origin (draw it here in a world-space
// renderer and it scrolls at `factor`).
parallax_world_pos :: proc "contextless" (l: Parallax_Layer, cam_origin: [2]f32) -> [2]f32 {
    return cam_origin + parallax_screen_pos(l, cam_origin)
}

// For repeating layers: the screen x of the first copy and how many copies
// cover a view of view_w pixels.
parallax_repeat :: proc "contextless" (l: Parallax_Layer, cam_origin: [2]f32, view_w: f32) -> (first_x: f32, count: int) {
    sx := parallax_screen_pos(l, cam_origin).x
    if l.width <= 0 { return sx, 1 }
    first_x = sx - math.ceil(sx / l.width) * l.width // <= 0
    if first_x > 0 { first_x -= l.width }
    count = int(math.ceil((view_w - first_x) / l.width))
    return
}
