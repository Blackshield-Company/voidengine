package templar_valley

// Templar Valley's world rules, ported from the Godot build (branch
// m1/terrain-pass at 1356930, src/world/valley_height.gd, valley_roads.gd,
// valley_ground.gd). The noise is the engine's bit-faithful FastNoiseLite
// port, so heights and ground classes match Godot's.

import "core:math"
import "core:math/linalg"
import ve "../../src/core"

HALF_EXTENT :: f32(1024)
HF_SPACING :: f32(2)            // Godot: Terrain3D vertex_spacing 2 m
HF_SIZE :: 1025                 // 2048 m / 2 m + 1
SPLAT_RES :: 2048               // 1 m per splat texel (Godot's control map: 2 m)

GRASS :: 0
ROCK :: 1
FOREST :: 2
DIRT :: 3
// Texture repeats per metre (ValleyGround.UV_SCALES)
UV_SCALES :: [4]f32{0.2, 0.12, 0.2, 0.25}

ROCK_START_DEG :: f32(16)
ROCK_FULL_DEG :: f32(26)
FOREST_BAND_X_M :: f32(560)
FOREST_EDGE_M :: f32(40)
PATCH_THRESHOLD :: f32(0.3)
VILLAGE_CLEAR_M :: f32(360)

ROAD_HALF_WIDTH :: f32(3)
ROAD_EDGE :: f32(2)
ROAD_RING_RADIUS :: f32(150)
ROAD_RING_POINTS :: 16

ROAD_MAIN := [?][2]f32{{0, -1010}, {16, -780}, {-8, -540}, {10, -300}, {-4, -150}, {0, 0}, {-12, 160}, {8, 420}, {-6, 660}, {6, 880}, {0, 1010}}
ROAD_HALL_SPOKE := [?][2]f32{{106, 106}, {170, 182}, {240, 258}}
ROAD_EAST_SPUR := [?][2]f32{{-8, -540}, {100, -562}, {240, -556}, {400, -532}, {560, -506}, {700, -498}, {820, -482}}

Valley :: struct {
    height_noise, edge_noise, patch_noise: ve.FNL_Noise,
    roads: [4][dynamic][2]f32,
}

valley_init :: proc(v: ^Valley) {
    v.height_noise = ve.fnl_godot(1165, 0.004, .FBm, 4)
    v.edge_noise = ve.fnl_godot(2207, 0.01, .None)
    v.patch_noise = ve.fnl_godot(3313, 0.006, .None)
    for p in ROAD_MAIN { append(&v.roads[0], p) }
    for i in 0 ..= ROAD_RING_POINTS {
        a := math.TAU * f32(i % ROAD_RING_POINTS) / f32(ROAD_RING_POINTS)
        append(&v.roads[1], [2]f32{math.cos(a), math.sin(a)} * ROAD_RING_RADIUS)
    }
    for p in ROAD_HALL_SPOKE { append(&v.roads[2], p) }
    for p in ROAD_EAST_SPUR { append(&v.roads[3], p) }
}

valley_destroy :: proc(v: ^Valley) {
    for &r in v.roads { delete(r) }
}

// ValleyHeight.height_at (GDScript doubles around the f32 noise).
valley_height :: proc(v: ^Valley, x, z: f32) -> f32 {
    n := ve.fnl_noise_2d(&v.height_noise, x, z)
    return f32(8.0 + 150.0 * math.pow(abs(f64(x)) / 1024.0, 1.5) + f64(n) * 18.0)
}

segment_distance :: proc(p, a, b: [2]f32) -> f32 {
    ab := b - a
    t := math.clamp(linalg.dot(p - a, ab) / max(linalg.dot(ab, ab), 1e-9), 0, 1)
    return linalg.length(p - (a + ab * t))
}

road_weight :: proc(d: f32) -> f32 {
    return math.clamp(1 - (d - ROAD_HALF_WIDTH) / ROAD_EDGE, 0, 1)
}

road_distance :: proc(v: ^Valley, x, z: f32) -> f32 {
    best := max(f32)
    for &line in v.roads {
        for i in 0 ..< len(line) - 1 { best = min(best, segment_distance({x, z}, line[i], line[i + 1])) }
    }
    return best
}

is_forest :: proc(v: ^Valley, x, z: f32) -> bool {
    edge := FOREST_BAND_X_M + FOREST_EDGE_M * ve.fnl_noise_1d(&v.edge_noise, z)
    if abs(x) > edge { return true }
    if x * x + z * z <= VILLAGE_CLEAR_M * VILLAGE_CLEAR_M { return false }
    return ve.fnl_noise_2d(&v.patch_noise, x, z) > PATCH_THRESHOLD
}

rock_weight :: proc(slope_deg: f32) -> f32 {
    return math.clamp((slope_deg - ROCK_START_DEG) / (ROCK_FULL_DEG - ROCK_START_DEG), 0, 1)
}

// ValleyGround.resolve -> (base, overlay, blend 0..255)
resolve :: proc(forest: bool, road, rock: f32) -> (base, overlay, blend: int) {
    b := forest ? FOREST : GRASS
    if rock > 0 && rock >= road {
        return road >= 1 ? DIRT : b, ROCK, blend_byte(rock)
    }
    if road > 0 { return b, DIRT, blend_byte(road) }
    return b, b, 0
}

blend_byte :: proc(w: f32) -> int { return clamp(int(math.round(w * 255)), 0, 255) }

// Heights at Godot's vertex positions: vertex (i, j) at (i*2 - 1024, j*2 - 1024).
build_heightfield :: proc(v: ^Valley) -> ve.R3D_Heightfield {
    hf := ve.r3d_heightfield_make(HF_SIZE, HF_SPACING, {-HALF_EXTENT, -HALF_EXTENT})
    for j in 0 ..< HF_SIZE {
        z := f32(j) * HF_SPACING - HALF_EXTENT
        for i in 0 ..< HF_SIZE {
            x := f32(i) * HF_SPACING - HALF_EXTENT
            hf.heights[j * HF_SIZE + i] = valley_height(v, x, z)
        }
    }
    return hf
}

// Splat map (RGBA = grass, rock, forest floor, dirt), 1 m texels over the
// 2 km square. Rock comes from the slope at the 2 m vertices (Godot's
// slope_deg_at: central differences), interpolated bilinearly.
build_splat :: proc(v: ^Valley, hf: ^ve.R3D_Heightfield) -> []u8 {
    n := HF_SIZE
    slope := make([]f32, n * n)
    defer delete(slope)
    for j in 0 ..< n {
        for i in 0 ..< n {
            i0, i1 := max(i - 1, 0), min(i + 1, n - 1)
            j0, j1 := max(j - 1, 0), min(j + 1, n - 1)
            dx := (ve.r3d_hf_at(hf, i1, j) - ve.r3d_hf_at(hf, i0, j)) / (f32(i1 - i0) * HF_SPACING)
            dz := (ve.r3d_hf_at(hf, i, j1) - ve.r3d_hf_at(hf, i, j0)) / (f32(j1 - j0) * HF_SPACING)
            slope[j * n + i] = math.to_degrees(math.atan(math.sqrt(dx * dx + dz * dz)))
        }
    }
    // road weights by rasterising each segment's neighbourhood
    road := make([]f32, SPLAT_RES * SPLAT_RES)
    defer delete(road)
    reach := ROAD_HALF_WIDTH + ROAD_EDGE
    for &line in v.roads {
        for k in 0 ..< len(line) - 1 {
            a, b := line[k], line[k + 1]
            x0 := max(int(math.floor(min(a.x, b.x) - reach + HALF_EXTENT)), 0)
            x1 := min(int(math.ceil(max(a.x, b.x) + reach + HALF_EXTENT)), SPLAT_RES - 1)
            z0 := max(int(math.floor(min(a.y, b.y) - reach + HALF_EXTENT)), 0)
            z1 := min(int(math.ceil(max(a.y, b.y) + reach + HALF_EXTENT)), SPLAT_RES - 1)
            for z in z0 ..= z1 {
                for x in x0 ..= x1 {
                    p := [2]f32{f32(x) + 0.5 - HALF_EXTENT, f32(z) + 0.5 - HALF_EXTENT}
                    w := road_weight(segment_distance(p, a, b))
                    if w > road[z * SPLAT_RES + x] { road[z * SPLAT_RES + x] = w }
                }
            }
        }
    }
    out := make([]u8, SPLAT_RES * SPLAT_RES * 4)
    for z in 0 ..< SPLAT_RES {
        wz := f32(z) + 0.5 - HALF_EXTENT
        edge := FOREST_BAND_X_M + FOREST_EDGE_M * ve.fnl_noise_1d(&v.edge_noise, wz)
        gz := (wz + HALF_EXTENT) / HF_SPACING
        for x in 0 ..< SPLAT_RES {
            wx := f32(x) + 0.5 - HALF_EXTENT
            forest := abs(wx) > edge || (wx * wx + wz * wz > VILLAGE_CLEAR_M * VILLAGE_CLEAR_M && ve.fnl_noise_2d(&v.patch_noise, wx, wz) > PATCH_THRESHOLD)
            gx := (wx + HALF_EXTENT) / HF_SPACING
            i := min(int(gx), n - 2)
            j := min(int(gz), n - 2)
            fx, fz := gx - f32(i), gz - f32(j)
            s := math.lerp(math.lerp(slope[j * n + i], slope[j * n + i + 1], fx), math.lerp(slope[(j + 1) * n + i], slope[(j + 1) * n + i + 1], fx), fz)
            base, over, blend := resolve(forest, road[z * SPLAT_RES + x], rock_weight(s))
            w: [4]f32
            bw := f32(blend) / 255
            w[base] += 1 - bw
            w[over] += bw
            o := (z * SPLAT_RES + x) * 4
            for c in 0 ..< 4 { out[o + c] = u8(clamp(int(math.round(w[c] * 255)), 0, 255)) }
        }
    }
    return out
}
