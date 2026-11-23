package voidengine

// ============================================================================
// collide3d — backend-agnostic static collision for the 3D side: triangle
// meshes with an AABB BVH, the terrain heightfield as an implicit triangle
// collider (same diagonal as the rendered LOD 0 grid), ray casts, and
// capsule / sphere sweeps.
//
// Sweeps use conservative advancement with Newton steps. The distance between
// a translating convex shape (capsule, sphere) and a static triangle is a
// convex function of the sweep parameter t, so the tangent at t lies below
// it and stepping to the tangent's root never passes the true time of impact.
// A sweep that starts in contact (within `skin`) only blocks motion that
// closes the distance, so sliding along a surface is never stopped by it.
// ============================================================================

import "core:math"
import "core:math/linalg"

R3D_COL_PLAYER :: u32(1) // capsule controller
R3D_COL_CAMERA :: u32(2) // camera boom probes, ground-below rays

R3D_Tri :: struct {
    a, b, c: [3]f32,
    n:       [3]f32, // unit face normal (counter-clockwise a, b, c)
    mask:    u32,
}

R3D_Bvh_Node :: struct {
    bmin, bmax: [3]f32,
    first:      i32, // leaf: first triangle; inner: right child (left is node + 1)
    count:      i32, // > 0 leaf
}

R3D_Tri_Mesh :: struct {
    tris:       [dynamic]R3D_Tri,
    nodes:      [dynamic]R3D_Bvh_Node,
    bmin, bmax: [3]f32,
}

R3D_Collision_World :: struct {
    hf:      ^R3D_Heightfield, // optional terrain collider
    hf_mask: u32,
    meshes:  [dynamic]R3D_Tri_Mesh,
    scratch: [dynamic]R3D_Tri, // candidates of the last query (reused)
    queries: int,              // stats
    tests:   int,
}

R3D_BVH_LEAF :: 4

r3d_tri_make :: proc "contextless" (a, b, c: [3]f32, mask: u32) -> R3D_Tri {
    n := linalg.cross(b - a, c - a)
    l := linalg.length(n)
    return {a = a, b = b, c = c, n = l > 1e-12 ? n / l : {0, 1, 0}, mask = mask}
}

r3d_tri_bounds :: #force_inline proc "contextless" (t: ^R3D_Tri) -> (mn, mx: [3]f32) {
    return linalg.min(linalg.min(t.a, t.b), t.c), linalg.max(linalg.max(t.a, t.b), t.c)
}

// Builds the BVH over `tris` (the slice is reordered in place; the mesh takes
// ownership of the dynamic array).
r3d_tri_mesh_build :: proc(tris: [dynamic]R3D_Tri) -> (m: R3D_Tri_Mesh) {
    m.tris = tris
    m.bmin = {max(f32), max(f32), max(f32)}
    m.bmax = {min(f32), min(f32), min(f32)}
    if len(m.tris) == 0 { m.bmin, m.bmax = {}, {}; return }
    reserve(&m.nodes, 2 * len(m.tris) / R3D_BVH_LEAF + 2)
    bvh_build(&m, 0, len(m.tris))
    m.bmin, m.bmax = m.nodes[0].bmin, m.nodes[0].bmax
    return
}

@(private="file")
bvh_build :: proc(m: ^R3D_Tri_Mesh, first, count: int) -> int {
    idx := len(m.nodes)
    append(&m.nodes, R3D_Bvh_Node{})
    mn := [3]f32{max(f32), max(f32), max(f32)}
    mx := [3]f32{min(f32), min(f32), min(f32)}
    cmn, cmx := mn, mx
    for i in first ..< first + count {
        a, b := r3d_tri_bounds(&m.tris[i])
        mn = linalg.min(mn, a); mx = linalg.max(mx, b)
        c := (m.tris[i].a + m.tris[i].b + m.tris[i].c) / 3
        cmn = linalg.min(cmn, c); cmx = linalg.max(cmx, c)
    }
    m.nodes[idx].bmin, m.nodes[idx].bmax = mn, mx
    if count <= R3D_BVH_LEAF {
        m.nodes[idx].first, m.nodes[idx].count = i32(first), i32(count)
        return idx
    }
    ext := cmx - cmn
    axis := 0
    if ext.y > ext[axis] { axis = 1 }
    if ext.z > ext[axis] { axis = 2 }
    // median split by centroid (nth-element via partial quickselect)
    mid := first + count / 2
    lo, hi := first, first + count - 1
    cen :: #force_inline proc "contextless" (t: ^R3D_Tri, ax: int) -> f32 { return t.a[ax] + t.b[ax] + t.c[ax] }
    for lo < hi {
        pivot := cen(&m.tris[(lo + hi) / 2], axis)
        i, j := lo, hi
        for i <= j {
            for cen(&m.tris[i], axis) < pivot { i += 1 }
            for cen(&m.tris[j], axis) > pivot { j -= 1 }
            if i <= j { m.tris[i], m.tris[j] = m.tris[j], m.tris[i]; i += 1; j -= 1 }
        }
        if mid <= j { hi = j } else if mid >= i { lo = i } else { break }
    }
    bvh_build(m, first, mid - first)
    right := bvh_build(m, mid, first + count - mid)
    m.nodes[idx].first, m.nodes[idx].count = i32(right), 0
    return idx
}

r3d_tri_mesh_destroy :: proc(m: ^R3D_Tri_Mesh) {
    delete(m.tris)
    delete(m.nodes)
    m^ = {}
}

r3d_aabb_overlap :: #force_inline proc "contextless" (amin, amax, bmin, bmax: [3]f32) -> bool {
    return amin.x <= bmax.x && amax.x >= bmin.x && amin.y <= bmax.y && amax.y >= bmin.y && amin.z <= bmax.z && amax.z >= bmin.z
}

// Appends the mesh triangles whose bounds overlap [bmin, bmax] and match mask.
r3d_tri_mesh_query :: proc(m: ^R3D_Tri_Mesh, bmin, bmax: [3]f32, mask: u32, out: ^[dynamic]R3D_Tri) {
    if len(m.nodes) == 0 || !r3d_aabb_overlap(bmin, bmax, m.bmin, m.bmax) { return }
    stack: [64]i32
    sp := 1
    stack[0] = 0
    for sp > 0 {
        sp -= 1
        ni := stack[sp]
        nd := &m.nodes[ni]
        if !r3d_aabb_overlap(bmin, bmax, nd.bmin, nd.bmax) { continue }
        if nd.count > 0 {
            for i in nd.first ..< nd.first + nd.count {
                t := &m.tris[i]
                if t.mask & mask == 0 { continue }
                a, b := r3d_tri_bounds(t)
                if r3d_aabb_overlap(bmin, bmax, a, b) { append(out, t^) }
            }
        } else {
            // depth is bounded by log2(n / leaf) for a median split
            stack[sp] = nd.first; sp += 1
            stack[sp] = ni + 1; sp += 1
        }
    }
}

// Heightfield triangles of the cells overlapping [bmin, bmax]: per cell
// (i, j) the rendered split a-c-b, b-c-d (diagonal from (i+1, j) to (i, j+1)).
r3d_hf_query :: proc(hf: ^R3D_Heightfield, bmin, bmax: [3]f32, mask: u32, out: ^[dynamic]R3D_Tri) {
    if hf == nil || len(hf.heights) == 0 { return }
    s := hf.spacing
    i0 := max(int(math.floor((bmin.x - hf.origin[0]) / s)), 0)
    i1 := min(int(math.floor((bmax.x - hf.origin[0]) / s)), hf.size - 2)
    j0 := max(int(math.floor((bmin.z - hf.origin[1]) / s)), 0)
    j1 := min(int(math.floor((bmax.z - hf.origin[1]) / s)), hf.size - 2)
    for j in j0 ..= j1 {
        for i in i0 ..= i1 {
            h00 := r3d_hf_at(hf, i, j); h10 := r3d_hf_at(hf, i + 1, j)
            h01 := r3d_hf_at(hf, i, j + 1); h11 := r3d_hf_at(hf, i + 1, j + 1)
            lo := min(h00, h10, h01, h11)
            hi := max(h00, h10, h01, h11)
            if hi < bmin.y || lo > bmax.y { continue }
            x0 := hf.origin[0] + f32(i) * s
            z0 := hf.origin[1] + f32(j) * s
            a := [3]f32{x0, h00, z0}
            b := [3]f32{x0 + s, h10, z0}
            c := [3]f32{x0, h01, z0 + s}
            d := [3]f32{x0 + s, h11, z0 + s}
            append(out, r3d_tri_make(a, c, b, mask), r3d_tri_make(b, c, d, mask))
        }
    }
}

// Height of the triangulated (collision / rendered LOD 0) terrain surface.
r3d_hf_height_tri :: proc "contextless" (hf: ^R3D_Heightfield, x, z: f32) -> f32 {
    gx := math.clamp((x - hf.origin[0]) / hf.spacing, 0, f32(hf.size - 1))
    gz := math.clamp((z - hf.origin[1]) / hf.spacing, 0, f32(hf.size - 1))
    i := min(int(gx), hf.size - 2)
    j := min(int(gz), hf.size - 2)
    fx := gx - f32(i)
    fz := gz - f32(j)
    h00 := r3d_hf_at(hf, i, j); h10 := r3d_hf_at(hf, i + 1, j)
    h01 := r3d_hf_at(hf, i, j + 1); h11 := r3d_hf_at(hf, i + 1, j + 1)
    if fx + fz <= 1 { return h00 + (h10 - h00) * fx + (h01 - h00) * fz }
    return h11 + (h01 - h11) * (1 - fx) + (h10 - h11) * (1 - fz)
}

r3d_world_destroy :: proc(w: ^R3D_Collision_World) {
    for &m in w.meshes { r3d_tri_mesh_destroy(&m) }
    delete(w.meshes)
    delete(w.scratch)
    w^ = {}
}

// Adds a mesh (ownership moves to the world); returns its index.
r3d_world_add_mesh :: proc(w: ^R3D_Collision_World, tris: [dynamic]R3D_Tri) -> int {
    append(&w.meshes, r3d_tri_mesh_build(tris))
    return len(w.meshes) - 1
}

// Gathers candidate triangles overlapping the box into w.scratch.
r3d_world_gather :: proc(w: ^R3D_Collision_World, bmin, bmax: [3]f32, mask: u32) -> []R3D_Tri {
    clear(&w.scratch)
    w.queries += 1
    if w.hf != nil && w.hf_mask & mask != 0 { r3d_hf_query(w.hf, bmin, bmax, w.hf_mask, &w.scratch) }
    for &m in w.meshes { r3d_tri_mesh_query(&m, bmin, bmax, mask, &w.scratch) }
    return w.scratch[:]
}

// ---------------------------------------------------------------------------
// Closest points
// ---------------------------------------------------------------------------

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection 5.1.5).
r3d_closest_point_tri :: proc "contextless" (p, a, b, c: [3]f32) -> [3]f32 {
    ab := b - a; ac := c - a; ap := p - a
    d1 := linalg.dot(ab, ap); d2 := linalg.dot(ac, ap)
    if d1 <= 0 && d2 <= 0 { return a }
    bp := p - b
    d3 := linalg.dot(ab, bp); d4 := linalg.dot(ac, bp)
    if d3 >= 0 && d4 <= d3 { return b }
    vc := d1 * d4 - d3 * d2
    if vc <= 0 && d1 >= 0 && d3 <= 0 { return a + ab * (d1 / (d1 - d3)) }
    cp := p - c
    d5 := linalg.dot(ab, cp); d6 := linalg.dot(ac, cp)
    if d6 >= 0 && d5 <= d6 { return c }
    vb := d5 * d2 - d1 * d6
    if vb <= 0 && d2 >= 0 && d6 <= 0 { return a + ac * (d2 / (d2 - d6)) }
    va := d3 * d6 - d5 * d4
    if va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0 {
        return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)))
    }
    denom := 1 / (va + vb + vc)
    return a + ab * (vb * denom) + ac * (vc * denom)
}

// Closest points between segments p1-q1 and p2-q2 (Ericson 5.1.9).
r3d_closest_seg_seg :: proc "contextless" (p1, q1, p2, q2: [3]f32) -> (c1, c2: [3]f32) {
    d1 := q1 - p1; d2 := q2 - p2; r := p1 - p2
    a := linalg.dot(d1, d1); e := linalg.dot(d2, d2); f := linalg.dot(d2, r)
    s, t: f32
    EPS :: 1e-12
    if a <= EPS && e <= EPS { return p1, p2 }
    if a <= EPS {
        s = 0; t = math.clamp(f / e, 0, 1)
    } else {
        c := linalg.dot(d1, r)
        if e <= EPS {
            t = 0; s = math.clamp(-c / a, 0, 1)
        } else {
            b := linalg.dot(d1, d2)
            denom := a * e - b * b
            s = denom != 0 ? math.clamp((b * f - c * e) / denom, 0, 1) : 0
            t = (b * s + f) / e
            if t < 0 { t = 0; s = math.clamp(-c / a, 0, 1) } else if t > 1 { t = 1; s = math.clamp((b - c) / a, 0, 1) }
        }
    }
    return p1 + d1 * s, p2 + d2 * t
}

// Segment p-q against triangle: intersection point, if any.
r3d_seg_tri_intersect :: proc "contextless" (p, q: [3]f32, t: ^R3D_Tri) -> (hit: [3]f32, ok: bool) {
    d := q - p
    e1 := t.b - t.a; e2 := t.c - t.a
    h := linalg.cross(d, e2)
    det := linalg.dot(e1, h)
    if abs(det) < 1e-12 { return }
    inv := 1 / det
    s := p - t.a
    u := linalg.dot(s, h) * inv
    if u < 0 || u > 1 { return }
    qv := linalg.cross(s, e1)
    v := linalg.dot(d, qv) * inv
    if v < 0 || u + v > 1 { return }
    tt := linalg.dot(e2, qv) * inv
    if tt < 0 || tt > 1 { return }
    return p + d * tt, true
}

// Closest points between segment p-q and a triangle; squared distance.
r3d_closest_seg_tri :: proc "contextless" (p, q: [3]f32, t: ^R3D_Tri) -> (on_seg, on_tri: [3]f32, dist2: f32) {
    if x, ok := r3d_seg_tri_intersect(p, q, t); ok { return x, x, 0 }
    best := max(f32)
    try :: #force_inline proc "contextless" (a, b: [3]f32, best: ^f32, os, ot: ^[3]f32) {
        d := linalg.length2(a - b)
        if d < best^ { best^ = d; os^ = a; ot^ = b }
    }
    try(p, r3d_closest_point_tri(p, t.a, t.b, t.c), &best, &on_seg, &on_tri)
    if linalg.length2(q - p) > 1e-12 {
        try(q, r3d_closest_point_tri(q, t.a, t.b, t.c), &best, &on_seg, &on_tri)
        e := [3][2][3]f32{{t.a, t.b}, {t.b, t.c}, {t.c, t.a}}
        for k in 0 ..< 3 {
            s1, s2 := r3d_closest_seg_seg(p, q, e[k][0], e[k][1])
            try(s1, s2, &best, &on_seg, &on_tri)
        }
    }
    return on_seg, on_tri, best
}

// Signed-ish distance from a capsule (core segment p-q, radius r) to a
// triangle and the separating normal (from the triangle toward the capsule).
r3d_capsule_tri_distance :: proc "contextless" (p, q: [3]f32, r: f32, t: ^R3D_Tri) -> (d: f32, n: [3]f32, contact: [3]f32) {
    s, c, d2 := r3d_closest_seg_tri(p, q, t)
    if d2 > 1e-14 {
        l := math.sqrt(d2)
        return l - r, (s - c) / l, c
    }
    // core segment touches the triangle: push along the face normal, toward the capsule middle
    n = t.n
    mid := (p + q) * 0.5
    if linalg.dot(mid - c, n) < 0 { n = -n }
    return -r, n, c
}

// ---------------------------------------------------------------------------
// Sweeps and rays
// ---------------------------------------------------------------------------

R3D_Sweep_Hit :: struct {
    hit:    bool,
    t:      f32,     // fraction of the motion that is free
    normal: [3]f32,  // separating normal at contact (toward the moving shape)
    point:  [3]f32,  // contact point on the obstacle
    start_dist: f32, // distance at t = 0 to the obstacle that was hit
}

// Sweeps a capsule (core segment p-q, radius r) along `motion` and returns
// the first time it comes within `skin` of a triangle while closing in. Hits
// within ~0.1 mm of the first are merged, preferring the most upward normal
// (so a floor wins over a steep face it meets at the same time).
r3d_sweep_capsule :: proc(w: ^R3D_Collision_World, p, q: [3]f32, r: f32, motion: [3]f32, mask: u32, skin: f32 = 0.01) -> (res: R3D_Sweep_Hit) {
    res.t = 1
    mlen := linalg.length(motion)
    pad := [3]f32{r + skin + 0.02, r + skin + 0.02, r + skin + 0.02}
    bmin := linalg.min(linalg.min(p, q), linalg.min(p, q) + motion) - pad
    bmax := linalg.max(linalg.max(p, q), linalg.max(p, q) + motion) + pad
    tris := r3d_world_gather(w, bmin, bmax, mask)
    tie := mlen > 1e-9 ? 1e-4 / mlen : 0
    for &tri in tris {
        w.tests += 1
        t: f32 = 0
        hit := false
        n, c: [3]f32
        d0: f32
        for it in 0 ..< 24 {
            d: f32
            d, n, c = r3d_capsule_tri_distance(p + motion * t, q + motion * t, r, &tri)
            if it == 0 { d0 = d }
            rate := -linalg.dot(n, motion) // distance lost per unit t
            if d <= skin {
                // in contact: only a closing motion is blocked
                if rate > 1e-5 * max(mlen, 1e-6) || (t > 0) { hit = true }
                break
            }
            if rate <= 1e-9 { break }
            t += (d - skin) / rate
            if t >= 1 { break }
            if it == 23 { hit = true } // grazing: stop at the safe t reached
        }
        if !hit { continue }
        if t < res.t - tie || !res.hit {
            res = {hit = true, t = max(t, 0), normal = n, point = c, start_dist = d0}
        } else if t <= res.t + tie && n.y > res.normal.y {
            res.normal = n; res.point = c; res.start_dist = d0
        }
    }
    if !res.hit { res.t = 1 }
    return
}

r3d_sweep_sphere :: proc(w: ^R3D_Collision_World, center: [3]f32, r: f32, motion: [3]f32, mask: u32, skin: f32 = 0.0) -> R3D_Sweep_Hit {
    return r3d_sweep_capsule(w, center, center, r, motion, mask, skin)
}

// Smallest distance from a capsule to the world (negative when the core
// touches geometry); max(f32) when nothing is within `reach`.
r3d_capsule_distance :: proc(w: ^R3D_Collision_World, p, q: [3]f32, r: f32, mask: u32, reach: f32 = 0.5) -> (d: f32, n: [3]f32) {
    d = max(f32)
    pad := [3]f32{r + reach, r + reach, r + reach}
    tris := r3d_world_gather(w, linalg.min(p, q) - pad, linalg.max(p, q) + pad, mask)
    for &tri in tris {
        dd, nn, _ := r3d_capsule_tri_distance(p, q, r, &tri)
        if dd < d { d, n = dd, nn }
    }
    return
}

r3d_sphere_overlaps :: proc(w: ^R3D_Collision_World, center: [3]f32, r: f32, mask: u32) -> bool {
    d, _ := r3d_capsule_distance(w, center, center, r, mask, 0.01)
    return d < 0
}

// Pushes a capsule out of any geometry it is closer to than `skin`
// (a few relaxation passes). Returns the correction applied.
r3d_capsule_depenetrate :: proc(w: ^R3D_Collision_World, p, q: [3]f32, r: f32, mask: u32, skin: f32 = 0.01) -> [3]f32 {
    total: [3]f32
    for _ in 0 ..< 4 {
        pad := [3]f32{r + skin, r + skin, r + skin}
        tris := r3d_world_gather(w, linalg.min(p, q) + total - pad, linalg.max(p, q) + total + pad, mask)
        worst: f32 = skin * 0.5
        push: [3]f32
        for &tri in tris {
            d, n, _ := r3d_capsule_tri_distance(p + total, q + total, r, &tri)
            if d < worst { worst = d; push = n * (skin - d) }
        }
        if worst >= skin * 0.5 { break }
        total += push
    }
    return total
}

// Two-sided ray / triangle (Moller-Trumbore); t in units of dir.
r3d_ray_tri :: proc "contextless" (o, dir: [3]f32, t: ^R3D_Tri) -> (tt: f32, ok: bool) {
    e1 := t.b - t.a; e2 := t.c - t.a
    h := linalg.cross(dir, e2)
    det := linalg.dot(e1, h)
    if abs(det) < 1e-12 { return }
    inv := 1 / det
    s := o - t.a
    u := linalg.dot(s, h) * inv
    if u < 0 || u > 1 { return }
    qv := linalg.cross(s, e1)
    v := linalg.dot(dir, qv) * inv
    if v < 0 || u + v > 1 { return }
    tt = linalg.dot(e2, qv) * inv
    return tt, tt >= 0
}

// First hit of the ray o + dir * t, t in [0, max_t] (dir need not be unit).
r3d_raycast :: proc(w: ^R3D_Collision_World, o, dir: [3]f32, max_t: f32, mask: u32) -> (t: f32, n: [3]f32, hit: bool) {
    e := o + dir * max_t
    tris := r3d_world_gather(w, linalg.min(o, e) - 0.001, linalg.max(o, e) + 0.001, mask)
    t = max_t
    for &tri in tris {
        if tt, ok := r3d_ray_tri(o, dir, &tri); ok && tt <= t {
            t, n, hit = tt, tri.n, true
        }
    }
    if hit && linalg.dot(n, dir) > 0 { n = -n }
    return
}

// Height of the highest surface below `point` (down to `depth`), -inf if none.
r3d_ground_below :: proc(w: ^R3D_Collision_World, point: [3]f32, mask: u32, depth: f32 = 500) -> f32 {
    t, _, hit := r3d_raycast(w, point, {0, -1, 0}, depth, mask)
    return hit ? point.y - t : math.inf_f32(-1)
}
