package voidengine

import "core:math"

// ============================================================================
// Platformer tile collision — swept AABB movement against a tile grid with
// solid tiles, one-way platforms and 45-degree floor slopes.
//
// Coordinates are pixels, y grows DOWN (screen convention). A body is an
// AABB with its top-left at (x, y). body_move splits a frame's movement into
// sub-steps no longer than a quarter tile, so fast bodies can't tunnel, and
// resolves X then Y each sub-step:
//
//   Solid           blocks from every side
//   One_Way         lands you only when falling onto it from above; you jump
//                   up through it, and drop through it with opts.drop_through
//   Slope_Up_Right  floor rising to the right  (◢)  — floor only, no ceiling
//   Slope_Up_Left   floor rising to the left   (◣)
//
// Slopes are sampled at the body's bottom-center, so a character's feet sit
// on the slope line. While the center is on a slope, solid tiles at the
// slope's high end that are within max_step of the feet don't count as walls,
// so walking off the top of a slope onto flat ground is seamless.
// ============================================================================

Tile_Shape :: enum u8 {
    Empty,
    Solid,
    One_Way,
    Slope_Up_Right,
    Slope_Up_Left,
}

Collision_Grid :: struct {
    width, height: i32,
    tile_size:     f32,
    shapes:        []Tile_Shape,
    edge_solid:    bool, // what lies outside the grid: true = Solid, false = Empty
}

// Axis-aligned rectangle: top-left x/y, width, height (y down).
Rect2 :: struct {
    x, y, w, h: f32,
}

rect2_overlaps :: proc "contextless" (a, b: Rect2) -> bool {
    return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h
}

rect2_contains :: proc "contextless" (r: Rect2, p: [2]f32) -> bool {
    return p.x >= r.x && p.x < r.x + r.w && p.y >= r.y && p.y < r.y + r.h
}

// A body's box.
AABB :: Rect2

Move_Options :: struct {
    drop_through: bool, // fall through one-way platforms this move
    grounded:     bool, // the body was on the ground before this move (enables snap_down)
    snap_down:    f32,  // max px to stick to the ground when walking down slopes/steps
    max_step:     f32,  // slope-top tolerance in px (0 = tile_size / 2)
}

Move_Result :: struct {
    on_ground:   bool,
    on_slope:    bool,
    on_one_way:  bool,
    hit_left:    bool,
    hit_right:   bool,
    hit_ceiling: bool,
}

COLLISION_EPS :: f32(0.001)

collision_grid_create :: proc(width, height: i32, tile_size: f32, allocator := context.allocator) -> Collision_Grid {
    return Collision_Grid{
        width = width, height = height, tile_size = tile_size,
        shapes = make([]Tile_Shape, int(width * height), allocator),
    }
}

collision_grid_destroy :: proc(g: ^Collision_Grid) {
    delete(g.shapes)
    g.shapes = nil
}

collision_grid_get :: proc "contextless" (g: ^Collision_Grid, tx, ty: i32) -> Tile_Shape {
    if tx < 0 || ty < 0 || tx >= g.width || ty >= g.height {
        return g.edge_solid ? .Solid : .Empty
    }
    return g.shapes[ty * g.width + tx]
}

collision_grid_set :: proc "contextless" (g: ^Collision_Grid, tx, ty: i32, shape: Tile_Shape) {
    if tx < 0 || ty < 0 || tx >= g.width || ty >= g.height { return }
    g.shapes[ty * g.width + tx] = shape
}

// Build a grid from a Tilemap: ids listed in `shapes` get that shape, other
// ids in the map's solid_ids become Solid, everything else is Empty.
collision_grid_from_tilemap :: proc(tm: ^Tilemap, shapes: map[i32]Tile_Shape = nil, allocator := context.allocator) -> Collision_Grid {
    g := collision_grid_create(tm.width, tm.height, f32(tm.tile_size), allocator)
    for ty in 0 ..< tm.height {
        for tx in 0 ..< tm.width {
            id := tm.tiles[ty * tm.width + tx]
            if id == 0 { continue }
            if s, ok := shapes[id]; ok {
                g.shapes[ty * tm.width + tx] = s
            } else if tm.solid_ids[id] {
                g.shapes[ty * tm.width + tx] = .Solid
            }
        }
    }
    return g
}

// Parse one CSV-like row string of shape codes: '.' empty, '#' solid,
// '=' one-way, '/' slope up-right, '\' slope up-left. Handy for tests and
// hand-written collision layers.
collision_grid_from_strings :: proc(rows: []string, tile_size: f32, allocator := context.allocator) -> Collision_Grid {
    h := i32(len(rows))
    w: i32 = 0
    for r in rows { w = max(w, i32(len(r))) }
    g := collision_grid_create(w, h, tile_size, allocator)
    for r, ty in rows {
        for tx in 0 ..< len(r) {
            s: Tile_Shape
            switch r[tx] {
            case '#': s = .Solid
            case '=': s = .One_Way
            case '/': s = .Slope_Up_Right
            case '\\': s = .Slope_Up_Left
            case:    s = .Empty
            }
            g.shapes[i32(ty) * w + i32(tx)] = s
        }
    }
    return g
}

@(private)
tile_of :: #force_inline proc "contextless" (v, ts: f32) -> i32 {
    return i32(math.floor(v / ts))
}

is_slope :: #force_inline proc "contextless" (s: Tile_Shape) -> bool {
    return s == .Slope_Up_Right || s == .Slope_Up_Left
}

// Floor height (y, down) of a slope tile at world x.
slope_surface_y :: proc "contextless" (shape: Tile_Shape, tx, ty: i32, ts: f32, x: f32) -> f32 {
    left := f32(tx) * ts
    top  := f32(ty) * ts
    lx := clampf(x - left, 0, ts)
    #partial switch shape {
    case .Slope_Up_Right: return top + ts - lx
    case .Slope_Up_Left:  return top + lx
    }
    return top
}

// Is the body's bottom-center over a slope? Returns the slope surface there.
@(private)
slope_under_center :: proc "contextless" (g: ^Collision_Grid, b: ^AABB) -> (surface: f32, found: bool) {
    ts := g.tile_size
    cx := b.x + b.w * 0.5
    bottom := b.y + b.h
    tcx := tile_of(cx, ts)
    r0 := tile_of(bottom - COLLISION_EPS, ts)
    best := f32(max(f32))
    for ty in ([3]i32{r0 - 1, r0, r0 + 1}) {
        s := collision_grid_get(g, tcx, ty)
        if !is_slope(s) { continue }
        sy := slope_surface_y(s, tcx, ty, ts, cx)
        // only surfaces near the feet (sunk up to a tile, or floating up to 1px)
        if sy >= bottom - ts && sy <= bottom + 1 && sy < best {
            best = sy
            found = true
        }
    }
    return best, found
}

@(private)
move_x :: proc(g: ^Collision_Grid, b: ^AABB, sx: f32, max_step: f32, res: ^Move_Result) {
    ts := g.tile_size
    nx := b.x + sx
    top := b.y
    bottom := b.y + b.h
    ty0 := tile_of(top, ts)
    ty1 := tile_of(bottom - COLLISION_EPS, ts)
    _, on_slope := slope_under_center(g, b)

    blocked := proc "contextless" (g: ^Collision_Grid, tx, ty: i32, bottom, ts, max_step: f32, on_slope: bool) -> bool {
        if collision_grid_get(g, tx, ty) != .Solid { return false }
        if on_slope && bottom - f32(ty) * ts <= max_step + COLLISION_EPS {
            return false // the flat top at a slope's high end
        }
        return true
    }

    if sx > 0 {
        tx := tile_of(nx + b.w - COLLISION_EPS, ts)
        for ty in ty0 ..= ty1 {
            if blocked(g, tx, ty, bottom, ts, max_step, on_slope) {
                nx = f32(tx) * ts - b.w
                res.hit_right = true
                break
            }
        }
    } else if sx < 0 {
        tx := tile_of(nx, ts)
        for ty in ty0 ..= ty1 {
            if blocked(g, tx, ty, bottom, ts, max_step, on_slope) {
                nx = f32(tx + 1) * ts
                res.hit_left = true
                break
            }
        }
    }
    b.x = nx
}

@(private)
move_y :: proc(g: ^Collision_Grid, b: ^AABB, sy: f32, opts: Move_Options, res: ^Move_Result) {
    ts := g.tile_size
    ny := b.y + sy
    tx0 := tile_of(b.x, ts)
    tx1 := tile_of(b.x + b.w - COLLISION_EPS, ts)

    if sy > 0 {
        old_bottom := b.y + b.h
        new_bottom := ny + b.h
        r0 := tile_of(old_bottom - COLLISION_EPS, ts)
        r1 := tile_of(new_bottom - COLLISION_EPS, ts)
        landed := false
        rows: for ty in r0 ..= r1 {
            top := f32(ty) * ts
            if old_bottom > top + COLLISION_EPS { continue } // already past this row's top
            for tx in tx0 ..= tx1 {
                s := collision_grid_get(g, tx, ty)
                if s == .Solid || (s == .One_Way && !opts.drop_through) {
                    ny = top - b.h
                    landed = true
                    res.on_one_way = s == .One_Way
                    break rows
                }
            }
        }
        if landed { res.on_ground = true }
    } else if sy < 0 {
        ty := tile_of(ny, ts)
        for tx in tx0 ..= tx1 {
            if collision_grid_get(g, tx, ty) == .Solid {
                ny = f32(ty + 1) * ts
                res.hit_ceiling = true
                break
            }
        }
    }
    b.y = ny
}

@(private)
resolve_slope :: proc(g: ^Collision_Grid, b: ^AABB, sy: f32, max_step: f32, res: ^Move_Result) {
    surface, found := slope_under_center(g, b)
    if !found {
        // Just walked off a slope's high end onto flat ground: the feet may
        // be up to max_step inside the solid tile under the center. Lift.
        if sy < 0 { return }
        ts := g.tile_size
        cx := b.x + b.w * 0.5
        bottom := b.y + b.h
        tcx := tile_of(cx, ts)
        ty := tile_of(bottom - COLLISION_EPS, ts)
        top := f32(ty) * ts
        if collision_grid_get(g, tcx, ty) == .Solid && collision_grid_get(g, tcx, ty - 1) != .Solid &&
           bottom - top > COLLISION_EPS && bottom - top <= max_step + COLLISION_EPS {
            b.y = top - b.h
            res.on_ground = true
        }
        return
    }
    bottom := b.y + b.h
    if bottom > surface + COLLISION_EPS {
        b.y = surface - b.h
        if sy >= 0 {
            res.on_ground = true
            res.on_slope = true
        }
    } else if sy >= 0 && surface - bottom <= COLLISION_EPS * 10 {
        res.on_ground = true
        res.on_slope = true
    }
}

// Distance from the body's feet down to the nearest floor (solid/one-way
// tops under its width, slope surface under its center), up to max_dist.
ground_probe :: proc(g: ^Collision_Grid, b: ^AABB, max_dist: f32, drop_through := false) -> (dist: f32, found: bool, shape: Tile_Shape) {
    ts := g.tile_size
    bottom := b.y + b.h
    tx0 := tile_of(b.x, ts)
    tx1 := tile_of(b.x + b.w - COLLISION_EPS, ts)
    best := max_dist + 1
    r0 := tile_of(bottom - COLLISION_EPS, ts)
    r1 := tile_of(bottom + max_dist, ts)
    for ty in r0 ..= r1 {
        top := f32(ty) * ts
        if top < bottom - COLLISION_EPS { continue }
        for tx in tx0 ..= tx1 {
            s := collision_grid_get(g, tx, ty)
            if s == .Solid || (s == .One_Way && !drop_through) {
                if top - bottom < best || (top - bottom == best && s == .Solid) {
                    best = top - bottom
                    shape = s
                }
            }
        }
    }
    cx := b.x + b.w * 0.5
    tcx := tile_of(cx, ts)
    for ty in r0 ..= r1 {
        s := collision_grid_get(g, tcx, ty)
        if !is_slope(s) { continue }
        sy := slope_surface_y(s, tcx, ty, ts, cx)
        d := sy - bottom
        if d >= -COLLISION_EPS && d < best { best = max(d, 0); shape = s }
    }
    if best <= max_dist { return best, true, shape }
    return 0, false, .Empty
}

// Move a body by delta (px) through the grid. Returns contact flags.
body_move :: proc(g: ^Collision_Grid, b: ^AABB, delta: [2]f32, opts := Move_Options{}) -> (res: Move_Result) {
    ts := g.tile_size
    max_step := opts.max_step > 0 ? opts.max_step : ts * 0.5
    step_len := ts * 0.25
    longest := max(abs(delta.x), abs(delta.y))
    n := max(1, int(math.ceil(longest / step_len)))
    sx := delta.x / f32(n)
    sy := delta.y / f32(n)
    for _ in 0 ..< n {
        if sx != 0 {
            move_x(g, b, sx, max_step, &res)
        }
        if sy != 0 {
            move_y(g, b, sy, opts, &res)
            if res.hit_ceiling { sy = 0 }
        }
        resolve_slope(g, b, sy, max_step, &res)
        if res.on_ground && sy > 0 { sy = 0 } // landed: stop falling for the rest of the frame
    }
    // Final contact comes from where the body ended up, not from contacts
    // during the sub-steps (it may have walked off a ledge after landing).
    // Grounded bodies also stick to the floor up to snap_down px below them,
    // so walking down slopes and small steps doesn't turn into falling.
    res.on_ground, res.on_slope, res.on_one_way = false, false, false
    if delta.y >= 0 && !res.hit_ceiling {
        reach := f32(0.02)
        if opts.grounded && opts.snap_down > reach { reach = opts.snap_down }
        if d, ok, shape := ground_probe(g, b, reach, opts.drop_through); ok {
            b.y += d
            res.on_ground = true
            res.on_slope = is_slope(shape)
            res.on_one_way = shape == .One_Way
        }
    }
    return
}

// Is any solid tile overlapping the box? (spawn checks, crush detection)
collision_box_blocked :: proc(g: ^Collision_Grid, b: AABB) -> bool {
    ts := g.tile_size
    tx0 := tile_of(b.x, ts)
    tx1 := tile_of(b.x + b.w - COLLISION_EPS, ts)
    ty0 := tile_of(b.y, ts)
    ty1 := tile_of(b.y + b.h - COLLISION_EPS, ts)
    for ty in ty0 ..= ty1 {
        for tx in tx0 ..= tx1 {
            if collision_grid_get(g, tx, ty) == .Solid { return true }
        }
    }
    return false
}

aabb_overlaps :: rect2_overlaps
