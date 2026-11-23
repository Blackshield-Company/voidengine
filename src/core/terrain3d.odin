package voidengine

// ============================================================================
// terrain3d — backend-agnostic heightfield terrain: a square grid of heights,
// bilinear height/normal queries, a chunk grid with geomipmap LOD selection,
// shared per-LOD index buffers with skirts (skirts hide the T-junction cracks
// between neighbouring LODs), and view-frustum culling of chunk AABBs.
//
// The GPU side (gl3d_terrain.odin) keeps one chunk-local vertex grid and
// one index buffer per LOD for every chunk; heights come from a texture, so a
// chunk is drawn with only its grid offset as a uniform.
// ============================================================================

import "core:math"
import "core:math/linalg"

R3D_Heightfield :: struct {
    size:    int,     // vertices per side
    spacing: f32,     // metres between vertices
    origin:  [2]f32,  // world (x, z) of vertex (0, 0)
    heights: []f32,   // size * size, row-major by z then x
}

r3d_heightfield_make :: proc(size: int, spacing: f32, origin: [2]f32, allocator := context.allocator) -> R3D_Heightfield {
    return {size = size, spacing = spacing, origin = origin, heights = make([]f32, size * size, allocator)}
}

r3d_heightfield_destroy :: proc(hf: ^R3D_Heightfield) {
    delete(hf.heights)
    hf^ = {}
}

// Height of vertex (i, j), clamped to the grid.
r3d_hf_at :: #force_inline proc "contextless" (hf: ^R3D_Heightfield, i, j: int) -> f32 {
    ii := math.clamp(i, 0, hf.size - 1)
    jj := math.clamp(j, 0, hf.size - 1)
    return hf.heights[jj * hf.size + ii]
}

// Extent covered by the grid in metres.
r3d_hf_extent :: proc "contextless" (hf: ^R3D_Heightfield) -> f32 { return f32(hf.size - 1) * hf.spacing }

// Bilinear height at world (x, z); clamped at the edges.
r3d_hf_height :: proc "contextless" (hf: ^R3D_Heightfield, x, z: f32) -> f32 {
    gx := (x - hf.origin[0]) / hf.spacing
    gz := (z - hf.origin[1]) / hf.spacing
    gx = math.clamp(gx, 0, f32(hf.size - 1))
    gz = math.clamp(gz, 0, f32(hf.size - 1))
    i := min(int(gx), hf.size - 2)
    j := min(int(gz), hf.size - 2)
    fx := gx - f32(i)
    fz := gz - f32(j)
    h00 := r3d_hf_at(hf, i, j)
    h10 := r3d_hf_at(hf, i + 1, j)
    h01 := r3d_hf_at(hf, i, j + 1)
    h11 := r3d_hf_at(hf, i + 1, j + 1)
    return math.lerp(math.lerp(h00, h10, fx), math.lerp(h01, h11, fx), fz)
}

// Unit surface normal at world (x, z) by central differences of the
// bilinear surface over one grid spacing.
r3d_hf_normal :: proc "contextless" (hf: ^R3D_Heightfield, x, z: f32) -> linalg.Vector3f32 {
    s := hf.spacing
    dx := r3d_hf_height(hf, x + s, z) - r3d_hf_height(hf, x - s, z)
    dz := r3d_hf_height(hf, x, z + s) - r3d_hf_height(hf, x, z - s)
    return linalg.normalize(linalg.Vector3f32{-dx, 2 * s, -dz})
}

// Slope in degrees at world (x, z).
r3d_hf_slope_deg :: proc "contextless" (hf: ^R3D_Heightfield, x, z: f32) -> f32 {
    n := r3d_hf_normal(hf, x, z)
    return math.to_degrees(math.acos(math.clamp(n.y, -1, 1)))
}

// ---------------------------------------------------------------------------
// Frustum
// ---------------------------------------------------------------------------

// Six planes (a, b, c, d) with a*x + b*y + c*z + d >= 0 inside; normalised.
R3D_Frustum :: struct { planes: [6][4]f32 }

// Planes from a view-projection matrix (Gribb-Hartmann). gl_depth: clip z
// in -w..w (OpenGL); otherwise 0..w (Vulkan / D3D).
r3d_frustum_from_matrix :: proc "contextless" (vp: linalg.Matrix4f32, gl_depth := true) -> (f: R3D_Frustum) {
    row :: #force_inline proc "contextless" (m: linalg.Matrix4f32, r: int) -> [4]f32 {
        return {m[r, 0], m[r, 1], m[r, 2], m[r, 3]}
    }
    r0, r1, r2, r3 := row(vp, 0), row(vp, 1), row(vp, 2), row(vp, 3)
    f.planes[0] = r3 + r0 // left
    f.planes[1] = r3 - r0 // right
    f.planes[2] = r3 + r1 // bottom
    f.planes[3] = r3 - r1 // top
    f.planes[4] = gl_depth ? r3 + r2 : r2 // near
    f.planes[5] = r3 - r2 // far
    for &p in f.planes {
        l := math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])
        if l > 0 { p /= l }
    }
    return
}

// False only when the box is completely outside one plane (conservative).
r3d_frustum_aabb :: proc "contextless" (f: ^R3D_Frustum, mn, mx: linalg.Vector3f32) -> bool {
    for p in f.planes {
        // the box corner furthest along the plane normal
        x := p[0] >= 0 ? mx.x : mn.x
        y := p[1] >= 0 ? mx.y : mn.y
        z := p[2] >= 0 ? mx.z : mn.z
        if p[0] * x + p[1] * y + p[2] * z + p[3] < 0 { return false }
    }
    return true
}

r3d_frustum_sphere :: proc "contextless" (f: ^R3D_Frustum, c: linalg.Vector3f32, radius: f32) -> bool {
    for p in f.planes {
        if p[0] * c.x + p[1] * c.y + p[2] * c.z + p[3] < -radius { return false }
    }
    return true
}

// ---------------------------------------------------------------------------
// Chunked geomipmap terrain
// ---------------------------------------------------------------------------

R3D_Terrain_Chunk :: struct {
    i0, j0:     int, // first vertex of the chunk in the heightfield
    min_y, max_y: f32,
    lod:        int,
    visible:    bool,
}

R3D_Terrain :: struct {
    hf:          ^R3D_Heightfield,
    chunk_quads: int, // quads per chunk side (power of two)
    chunks_x:    int, // chunks per side
    lod_count:   int, // LOD 0 .. lod_count-1; LOD k steps 2^k vertices
    lod_base:    f32, // LOD 0 out to this distance, doubling per LOD
    skirt:       f32, // skirt depth in metres at LOD 0 (scaled by 2^lod)
    chunks:      []R3D_Terrain_Chunk,
    // stats from the last update
    drawn:       int,
    drawn_lod:   [8]int,
}

// size - 1 must be a multiple of chunk_quads.
r3d_terrain_init :: proc(t: ^R3D_Terrain, hf: ^R3D_Heightfield, chunk_quads := 64, lod_base: f32 = 96) -> bool {
    if hf.size < 2 || (hf.size - 1) % chunk_quads != 0 { return false }
    t.hf = hf
    t.chunk_quads = chunk_quads
    t.chunks_x = (hf.size - 1) / chunk_quads
    t.lod_count = 0
    for q := chunk_quads; q >= 1 && t.lod_count < 8; q /= 2 { t.lod_count += 1 }
    t.lod_base = lod_base
    t.skirt = 2 * hf.spacing
    t.chunks = make([]R3D_Terrain_Chunk, t.chunks_x * t.chunks_x)
    for cj in 0 ..< t.chunks_x {
        for ci in 0 ..< t.chunks_x {
            c := &t.chunks[cj * t.chunks_x + ci]
            c.i0 = ci * chunk_quads
            c.j0 = cj * chunk_quads
            c.min_y = max(f32)
            c.max_y = min(f32)
            for j in c.j0 ..= c.j0 + chunk_quads {
                for i in c.i0 ..= c.i0 + chunk_quads {
                    h := r3d_hf_at(hf, i, j)
                    c.min_y = min(c.min_y, h)
                    c.max_y = max(c.max_y, h)
                }
            }
        }
    }
    return true
}

r3d_terrain_destroy :: proc(t: ^R3D_Terrain) {
    delete(t.chunks)
    t^ = {}
}

// World AABB of a chunk (skirt included).
r3d_terrain_chunk_aabb :: proc "contextless" (t: ^R3D_Terrain, c: ^R3D_Terrain_Chunk) -> (mn, mx: linalg.Vector3f32) {
    hf := t.hf
    x0 := hf.origin[0] + f32(c.i0) * hf.spacing
    z0 := hf.origin[1] + f32(c.j0) * hf.spacing
    w := f32(t.chunk_quads) * hf.spacing
    skirt := t.skirt * f32(int(1) << uint(t.lod_count - 1))
    return {x0, c.min_y - skirt, z0}, {x0 + w, c.max_y, z0 + w}
}

// LOD for a distance: 0 inside lod_base, +1 per doubling, clamped.
r3d_terrain_lod_for_distance :: proc "contextless" (t: ^R3D_Terrain, dist: f32) -> int {
    if dist <= t.lod_base { return 0 }
    lod := int(math.floor(math.log2(dist / t.lod_base))) + 1
    return math.clamp(lod, 0, t.lod_count - 1)
}

// Distance from p to the box (0 inside).
r3d_aabb_distance :: proc "contextless" (mn, mx, p: linalg.Vector3f32) -> f32 {
    d := linalg.Vector3f32{
        max(mn.x - p.x, 0, p.x - mx.x),
        max(mn.y - p.y, 0, p.y - mx.y),
        max(mn.z - p.z, 0, p.z - mx.z),
    }
    return linalg.length(d)
}

// Pick each chunk's LOD from the eye distance and cull it against the
// frustum (nil: no culling). Returns the number of visible chunks.
r3d_terrain_update :: proc(t: ^R3D_Terrain, eye: linalg.Vector3f32, frustum: ^R3D_Frustum, lod_bias := 0) -> int {
    t.drawn = 0
    t.drawn_lod = {}
    for &c in t.chunks {
        mn, mx := r3d_terrain_chunk_aabb(t, &c)
        c.visible = frustum == nil || r3d_frustum_aabb(frustum, mn, mx)
        c.lod = math.clamp(r3d_terrain_lod_for_distance(t, r3d_aabb_distance(mn, mx, eye)) + lod_bias, 0, t.lod_count - 1)
        if c.visible {
            t.drawn += 1
            t.drawn_lod[c.lod] += 1
        }
    }
    return t.drawn
}

// Chunk-local vertex grid shared by every chunk: (n+1)^2 surface vertices
// (i, j, 0) followed by 4*(n+1) skirt vertices (i, j, 1) along the border
// (bottom edge, right, top, left; each edge's own n+1 vertices).
r3d_terrain_grid_vertices :: proc(n: int, allocator := context.allocator) -> [][3]f32 {
    v := make([][3]f32, (n + 1) * (n + 1) + 4 * (n + 1), allocator)
    k := 0
    for j in 0 ..= n { for i in 0 ..= n { v[k] = {f32(i), f32(j), 0}; k += 1 } }
    for e in 0 ..< 4 {
        for s in 0 ..= n {
            i, j: int
            switch e {
            case 0: i, j = s, 0
            case 1: i, j = n, s
            case 2: i, j = n - s, n
            case 3: i, j = 0, n - s
            }
            v[k] = {f32(i), f32(j), 1}
            k += 1
        }
    }
    return v
}

// Triangle indices into r3d_terrain_grid_vertices for LOD `lod` (step 2^lod),
// counter-clockwise seen from above (+y), skirts facing outward.
r3d_terrain_lod_indices :: proc(n, lod: int, allocator := context.allocator) -> []u32 {
    step := 1 << uint(lod)
    q := n / step
    idx := make([dynamic]u32, 0, q * q * 6 + 4 * q * 6, allocator)
    vid :: #force_inline proc(n, i, j: int) -> u32 { return u32(j * (n + 1) + i) }
    for jj in 0 ..< q {
        for ii in 0 ..< q {
            i, j := ii * step, jj * step
            a := vid(n, i, j)
            b := vid(n, i + step, j)
            c := vid(n, i, j + step)
            d := vid(n, i + step, j + step)
            // +z is "down" the screen seen from above; CCW from +y is a, c, b
            append(&idx, a, c, b, b, c, d)
        }
    }
    base := (n + 1) * (n + 1)
    for e in 0 ..< 4 {
        for ss in 0 ..< q {
            s0, s1 := ss * step, (ss + 1) * step
            ti0 := u32(base + e * (n + 1) + s0)
            ti1 := u32(base + e * (n + 1) + s1)
            top :: proc(n, e, s: int) -> u32 {
                switch e {
                case 0: return u32(s)                          // (s, 0)
                case 1: return u32(s * (n + 1) + n)            // (n, s)
                case 2: return u32(n * (n + 1) + (n - s))      // (n - s, n)
                }
                return u32((n - s) * (n + 1))                  // (0, n - s)
            }
            v0 := top(n, e, s0)
            v1 := top(n, e, s1)
            // the border runs clockwise seen from above (x right, -z up), so
            // v0 -> v1 -> ti0 faces outward
            append(&idx, v0, v1, ti0, v1, ti1, ti0)
        }
    }
    return idx[:]
}
