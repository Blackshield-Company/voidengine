package voidengine

// ============================================================================
// occlusion3d — on-foot readability (port of Templar's Godot OcclusionDirector,
// OccluderRegistry and OcclusionFader at 1356930).
// Each step, segments from the camera to the player's feet, torso and head
// (0.15 / 0.9 / 1.7 m) are tested against registered occluder AABBs grown by
// 0.3 m. Roofs on a line, the roof of a building whose wall is on a line and
// the roof of the interior the player stands in are cut (faded to 0, then
// hidden); walls, canopies and props are faded to 0.3 alpha. Fades run 0.15 s
// in, hold 0.2 s, 0.15 s out, quantised to 8 levels, at most 16 at once.
// Entering an interior volume asks the camera for interior mode (0.4 m exit
// hysteresis). A step is readable when no uncleared occluder (fade amount
// < 0.5, or a cut not yet hidden) and no terrain touches the thin lines.
// ============================================================================

import "core:math"
import "core:math/linalg"

R3D_Occluder_Kind :: enum u8 { Wall, Roof, Canopy, Prop }

R3D_Occluder :: struct {
    bmin, bmax: [3]f32,
    kind:       R3D_Occluder_Kind,
    group:      i32, // building (-1 none)
    user:       i32, // caller's id (render object)
    // fader state
    amount:     f32,
    level:      i32,
    wanted_at:  f64,
    cut:        bool,
    busy:       bool,
}

R3D_Interior :: struct {
    bmin, bmax: [3]f32,
    group:      i32,
}

R3D_OCC_LEVELS :: 8
R3D_OCC_CLEAR_AMOUNT :: f32(0.5)
R3D_OCC_BUCKET :: f32(8)

R3D_Occlusion :: struct {
    occluders:  [dynamic]R3D_Occluder,
    interiors:  [dynamic]R3D_Interior,
    roof_of:    map[i32]i32,
    buckets:    map[[2]i32][dynamic]i32,
    hf:         ^R3D_Heightfield, // terrain readability (optional)
    enabled:    bool,
    sample_heights: [3]f32,
    inflate_m, exit_margin_m: f32,
    fade_in_s, fade_out_s, hold_s, fade_alpha: f32,
    max_active: int,

    time:       f64,
    active:     [dynamic]i32,
    refused:    int,
    // per-step results
    readable:   bool,
    lines_blocked, blockers, candidates, wanted: int,
    terrain_blocked: bool,
    interior_index: int,
    interior_group: i32,
    interior_entries, interior_exits: int,
    cand:       [dynamic]i32,
    want:       [dynamic]i32,
    want_d:     [dynamic]f32,
    want_cut:   [dynamic]bool,
    stamp:      [dynamic]u32, // de-dup for bucket collection
    stamp_n:    u32,
}

r3d_occlusion_init :: proc(o: ^R3D_Occlusion) {
    o.enabled = true
    o.sample_heights = {0.15, 0.9, 1.7}
    o.inflate_m = 0.3
    o.exit_margin_m = 0.4
    o.fade_in_s = 0.15; o.fade_out_s = 0.15; o.hold_s = 0.2; o.fade_alpha = 0.3
    o.max_active = 16
    o.interior_index = -1
    o.interior_group = -1
    o.readable = true
}

r3d_occlusion_destroy :: proc(o: ^R3D_Occlusion) {
    delete(o.occluders); delete(o.interiors); delete(o.roof_of)
    for _, v in o.buckets { delete(v) }
    delete(o.buckets)
    delete(o.active); delete(o.cand); delete(o.want); delete(o.want_d); delete(o.want_cut); delete(o.stamp)
    o^ = {}
}

@(private="file")
bucket_of :: #force_inline proc "contextless" (x, z: f32) -> [2]i32 {
    return {i32(math.floor(x / R3D_OCC_BUCKET)), i32(math.floor(z / R3D_OCC_BUCKET))}
}

r3d_occlusion_add :: proc(o: ^R3D_Occlusion, bmin, bmax: [3]f32, kind: R3D_Occluder_Kind, group: i32 = -1, user: i32 = -1) -> int {
    id := i32(len(o.occluders))
    append(&o.occluders, R3D_Occluder{bmin = bmin, bmax = bmax, kind = kind, group = group, user = user, wanted_at = -1e9})
    append(&o.stamp, 0)
    if kind == .Roof && group >= 0 { o.roof_of[group] = id }
    b0 := bucket_of(bmin.x, bmin.z)
    b1 := bucket_of(bmax.x, bmax.z)
    for bz in b0.y ..= b1.y {
        for bx in b0.x ..= b1.x {
            k := [2]i32{bx, bz}
            if k not_in o.buckets { o.buckets[k] = make([dynamic]i32) }
            append(&o.buckets[k], id)
        }
    }
    return int(id)
}

r3d_occlusion_add_interior :: proc(o: ^R3D_Occlusion, bmin, bmax: [3]f32, group: i32) -> int {
    append(&o.interiors, R3D_Interior{bmin = bmin, bmax = bmax, group = group})
    return len(o.interiors) - 1
}

r3d_occlusion_interior_at :: proc "contextless" (o: ^R3D_Occlusion, p: [3]f32, margin: f32) -> int {
    for &it, i in o.interiors {
        if p.x >= it.bmin.x - margin && p.x <= it.bmax.x + margin && p.y >= it.bmin.y - margin && p.y <= it.bmax.y + margin &&
           p.z >= it.bmin.z - margin && p.z <= it.bmax.z + margin { return i }
    }
    return -1
}

// Segment vs AABB grown by `inflate` (slab test).
r3d_segment_hits_box :: proc "contextless" (from, to, bmin, bmax: [3]f32, inflate: f32 = 0) -> bool {
    lo := bmin - inflate
    hi := bmax + inflate
    d := to - from
    t0: f32 = 0
    t1: f32 = 1
    for a in 0 ..< 3 {
        if abs(d[a]) < 1e-9 {
            if from[a] < lo[a] || from[a] > hi[a] { return false }
            continue
        }
        inv := 1 / d[a]
        ta := (lo[a] - from[a]) * inv
        tb := (hi[a] - from[a]) * inv
        if ta > tb { ta, tb = tb, ta }
        t0 = max(t0, ta); t1 = min(t1, tb)
        if t0 > t1 { return false }
    }
    return true
}

@(private="file")
collect :: proc(o: ^R3D_Occlusion, lo, hi: [3]f32) {
    clear(&o.cand)
    o.stamp_n += 1
    b0 := bucket_of(lo.x, lo.z)
    b1 := bucket_of(hi.x, hi.z)
    for bz in b0.y ..= b1.y {
        for bx in b0.x ..= b1.x {
            list, ok := o.buckets[[2]i32{bx, bz}]
            if !ok { continue }
            for id in list {
                if o.stamp[id] == o.stamp_n { continue }
                o.stamp[id] = o.stamp_n
                oc := &o.occluders[id]
                if r3d_aabb_overlap(lo, hi, oc.bmin, oc.bmax) { append(&o.cand, id) }
            }
        }
    }
}

@(private="file")
want_add :: proc(o: ^R3D_Occlusion, id: i32, cut: bool, d: f32) {
    if id < 0 || o.wanted >= 64 { return }
    for i in 0 ..< o.wanted {
        if o.want[i] == id { if cut { o.want_cut[i] = true }; return }
    }
    resize(&o.want, o.wanted + 1); resize(&o.want_d, o.wanted + 1); resize(&o.want_cut, o.wanted + 1)
    at := o.wanted
    for at > 0 && o.want_d[at - 1] > d {
        o.want[at] = o.want[at - 1]; o.want_d[at] = o.want_d[at - 1]; o.want_cut[at] = o.want_cut[at - 1]
        at -= 1
    }
    o.want[at] = id; o.want_d[at] = d; o.want_cut[at] = cut
    o.wanted += 1
}

// Fader: ask for id to be cleared this step; false if refused by the cap.
r3d_occlusion_request :: proc(o: ^R3D_Occlusion, id: i32, cut: bool) -> bool {
    oc := &o.occluders[id]
    if !oc.busy {
        if len(o.active) >= o.max_active { o.refused += 1; return false }
        oc.busy = true; oc.amount = 0; oc.level = 0; oc.cut = cut
        append(&o.active, id)
    } else if cut && !oc.cut {
        oc.cut = true
    }
    oc.wanted_at = o.time
    return true
}

@(private="file")
fader_apply :: proc(o: ^R3D_Occlusion, dt: f32) {
    i := 0
    for i < len(o.active) {
        oc := &o.occluders[o.active[i]]
        since := o.time - oc.wanted_at
        if since <= 0 {
            oc.amount = min(1, oc.amount + dt / max(o.fade_in_s, 0.0001))
            if oc.amount > 0.9999 { oc.amount = 1 }
        } else if since > f64(o.hold_s) + 0.0001 {
            oc.amount = max(0, oc.amount - dt / max(o.fade_out_s, 0.0001))
            if oc.amount < 0.0001 { oc.amount = 0 }
        }
        oc.level = i32(math.round(oc.amount * R3D_OCC_LEVELS))
        if oc.amount <= 0 {
            oc.busy = false; oc.cut = false; oc.level = 0
            unordered_remove(&o.active, i)
            continue
        }
        i += 1
    }
}

r3d_occlusion_is_cleared :: proc "contextless" (o: ^R3D_Occlusion, id: i32) -> bool {
    oc := &o.occluders[id]
    if !oc.busy { return false }
    if oc.cut { return oc.level >= R3D_OCC_LEVELS }
    return oc.amount >= R3D_OCC_CLEAR_AMOUNT
}

// Render state of an occluder: alpha (1 = untouched) and hidden (cut done).
r3d_occlusion_alpha :: proc "contextless" (o: ^R3D_Occlusion, id: i32) -> (alpha: f32, hidden: bool) {
    oc := &o.occluders[id]
    if !oc.busy || oc.level <= 0 { return 1, false }
    if oc.cut && oc.level >= R3D_OCC_LEVELS { return 0, true }
    return math.lerp(f32(1), oc.cut ? 0 : o.fade_alpha, f32(oc.level) / R3D_OCC_LEVELS), false
}

// Counts of busy entries at or past the clear threshold (fades, cuts).
r3d_occlusion_cleared_counts :: proc "contextless" (o: ^R3D_Occlusion) -> (faded, cut: int) {
    for id in o.active {
        if !r3d_occlusion_is_cleared(o, id) { continue }
        if o.occluders[id].cut { cut += 1 } else { faded += 1 }
    }
    return
}

@(private="file")
terrain_hits :: proc(o: ^R3D_Occlusion, cam, point: [3]f32) -> bool {
    if o.hf == nil { return false }
    n := math.clamp(int(math.ceil(linalg.length(point - cam) / 1.0)), 8, 32)
    for k in 1 ..< n {
        p := linalg.lerp(cam, point, f32(k) / f32(n))
        if p.y < r3d_hf_height(o.hf, p.x, p.z) { return true }
    }
    return false
}

// One step after the camera. Returns whether the camera should be in
// interior mode.
r3d_occlusion_step :: proc(o: ^R3D_Occlusion, dt: f32, cam, feet: [3]f32) -> (interior: bool) {
    pts: [3][3]f32
    for i in 0 ..< 3 { pts[i] = feet + {0, o.sample_heights[i], 0} }
    inside := r3d_occlusion_interior_at(o, feet + {0, 0.1, 0}, o.interior_index >= 0 ? o.exit_margin_m : 0)
    g: i32 = inside >= 0 ? o.interiors[inside].group : -1
    if g != o.interior_group {
        if o.interior_group >= 0 { o.interior_exits += 1 }
        if g >= 0 { o.interior_entries += 1 }
        o.interior_group = g
    }
    o.interior_index = inside
    interior = o.enabled && inside >= 0

    o.time += f64(max(dt, 0))
    o.wanted = 0
    if interior {
        if r, ok := o.roof_of[g]; ok { want_add(o, r, true, -1) }
    }
    margin := [3]f32{1, 1, 1} * (o.inflate_m + 0.1)
    lo := linalg.min(linalg.min(cam, pts[0]), pts[2]) - margin
    hi := linalg.max(linalg.max(cam, pts[0]), pts[2]) + margin
    collect(o, lo, hi)
    o.candidates = len(o.cand)
    if o.enabled {
        for id in o.cand {
            oc := &o.occluders[id]
            on := false
            for p in pts { if r3d_segment_hits_box(cam, p, oc.bmin, oc.bmax, o.inflate_m) { on = true; break } }
            if !on { continue }
            c := (oc.bmin + oc.bmax) * 0.5
            d := linalg.length2(c - feet)
            want_add(o, id, oc.kind == .Roof, d)
            if oc.kind == .Wall && oc.group >= 0 {
                if r, ok := o.roof_of[oc.group]; ok { want_add(o, r, true, d) }
            }
        }
    }
    for i in 0 ..< o.wanted { r3d_occlusion_request(o, o.want[i], o.want_cut[i]) }
    fader_apply(o, max(dt, 0))

    // readability on the thin lines
    o.blockers = 0
    mask := 0
    for id in o.cand {
        if r3d_occlusion_is_cleared(o, id) { continue }
        oc := &o.occluders[id]
        hit := false
        for p, li in pts {
            if r3d_segment_hits_box(cam, p, oc.bmin, oc.bmax, 0) { mask |= 1 << uint(li); hit = true }
        }
        if hit { o.blockers += 1 }
    }
    o.terrain_blocked = false
    for p, li in pts {
        if terrain_hits(o, cam, p) { mask |= 1 << uint(li); o.terrain_blocked = true }
    }
    o.lines_blocked = (mask & 1) + ((mask >> 1) & 1) + ((mask >> 2) & 1)
    o.readable = o.lines_blocked == 0
    return
}
