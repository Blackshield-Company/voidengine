package voidengine_test

// Tests for terrain3d.odin: heightfield sampling and normals, frustum
// planes and culling, LOD selection, and the per-LOD index buffers (counts,
// winding, skirts facing outward, every LOD covering the whole chunk).

import "core:math"
import "core:testing"
import "core:math/linalg"
import engine "../src/core"

@(private="file")
make_plane_hf :: proc(size: int, spacing: f32, a, b, c: f32) -> engine.R3D_Heightfield {
    hf := engine.r3d_heightfield_make(size, spacing, {-10, -20})
    for j in 0 ..< size {
        for i in 0 ..< size {
            x := hf.origin[0] + f32(i) * spacing
            z := hf.origin[1] + f32(j) * spacing
            hf.heights[j * size + i] = a * x + b * z + c
        }
    }
    return hf
}

@(test)
test_heightfield_bilinear_and_normal :: proc(t: ^testing.T) {
    hf := make_plane_hf(17, 2, 0.5, -0.25, 3)
    defer engine.r3d_heightfield_destroy(&hf)
    // a plane is reproduced exactly by bilinear interpolation, off-grid too
    for p in ([?][2]f32{{-10, -20}, {-3.3, -11.7}, {7.9, 3.1}, {21.99, 11.5}}) {
        h := engine.r3d_hf_height(&hf, p[0], p[1])
        testing.expectf(t, abs(h - (0.5 * p[0] - 0.25 * p[1] + 3)) < 1e-4, "h at %v = %v", p, h)
    }
    n := engine.r3d_hf_normal(&hf, 1, 1)
    want := linalg.normalize(linalg.Vector3f32{-0.5, 1, 0.25})
    testing.expect(t, linalg.dot(n, want) > 0.99999)
    slope := engine.r3d_hf_slope_deg(&hf, 1, 1)
    testing.expect(t, abs(slope - math.to_degrees(math.acos(want.y))) < 0.01)
    // clamped outside the grid
    testing.expect(t, abs(engine.r3d_hf_height(&hf, -100, -20) - engine.r3d_hf_height(&hf, -10, -20)) < 1e-5)
    testing.expect(t, engine.r3d_hf_extent(&hf) == 32)
}

@(test)
test_frustum_culls_boxes :: proc(t: ^testing.T) {
    cam := engine.R3D_Camera{position = {0, 2, 0}, yaw = 0, pitch = 0, fov_y = math.to_radians(f32(60)), near_z = 0.1, far_z = 100}
    vp := engine.r3d_camera_view_proj(&cam, 16.0 / 9.0, false)
    f := engine.r3d_frustum_from_matrix(vp, true)
    // camera looks down -z
    testing.expect(t, engine.r3d_frustum_aabb(&f, {-1, 1, -11}, {1, 3, -9}))       // ahead
    testing.expect(t, !engine.r3d_frustum_aabb(&f, {-1, 1, 9}, {1, 3, 11}))        // behind
    testing.expect(t, !engine.r3d_frustum_aabb(&f, {-1, 1, -211}, {1, 3, -209}))   // past far
    testing.expect(t, !engine.r3d_frustum_aabb(&f, {60, 1, -11}, {62, 3, -9}))     // far right
    testing.expect(t, engine.r3d_frustum_aabb(&f, {-500, -1, -500}, {500, 0, 500})) // straddles
    testing.expect(t, engine.r3d_frustum_sphere(&f, {0, 2, -50}, 1))
    testing.expect(t, !engine.r3d_frustum_sphere(&f, {0, 2, 50}, 1))
    // every corner of the view volume is inside (on) all planes
    inv := linalg.inverse(vp)
    for c in ([?][3]f32{{-1, -1, -1}, {1, 1, -1}, {1, -1, 0.99}, {-1, 1, 0.99}}) {
        p := inv * linalg.Vector4f32{c[0], c[1], c[2], 1}
        w := p.xyz / p.w
        for pl in f.planes { testing.expect(t, pl[0] * w.x + pl[1] * w.y + pl[2] * w.z + pl[3] > -1e-2) }
    }
}

@(test)
test_terrain_lod_and_culling :: proc(t: ^testing.T) {
    hf := make_plane_hf(257, 2, 0, 0, 5)
    defer engine.r3d_heightfield_destroy(&hf)
    ter: engine.R3D_Terrain
    testing.expect(t, engine.r3d_terrain_init(&ter, &hf, 32, 50))
    defer engine.r3d_terrain_destroy(&ter)
    testing.expect(t, ter.chunks_x == 8 && ter.lod_count == 6)
    testing.expect(t, engine.r3d_terrain_lod_for_distance(&ter, 0) == 0)
    testing.expect(t, engine.r3d_terrain_lod_for_distance(&ter, 50) == 0)
    testing.expect(t, engine.r3d_terrain_lod_for_distance(&ter, 51) == 1)
    testing.expect(t, engine.r3d_terrain_lod_for_distance(&ter, 150) == 2)
    testing.expect(t, engine.r3d_terrain_lod_for_distance(&ter, 1e6) == 5)
    prev := -1
    for d in 0 ..< 100 { // monotonic in distance
        l := engine.r3d_terrain_lod_for_distance(&ter, f32(d) * 10)
        testing.expect(t, l >= prev)
        prev = l
    }
    cam := engine.R3D_Camera{position = {0, 20, 0}, yaw = 0, pitch = -0.3, fov_y = 1.2, near_z = 0.1, far_z = 1000}
    vp := engine.r3d_camera_view_proj(&cam, 16.0 / 9.0, false)
    f := engine.r3d_frustum_from_matrix(vp, true)
    n := engine.r3d_terrain_update(&ter, cam.position, &f)
    testing.expect(t, n > 0 && n < len(ter.chunks)) // culled some, kept some
    all := engine.r3d_terrain_update(&ter, cam.position, nil)
    testing.expect(t, all == len(ter.chunks))
    // the chunk under the camera is LOD 0 and visible
    for &c in ter.chunks {
        mn, mx := engine.r3d_terrain_chunk_aabb(&ter, &c)
        if mn.x <= 0 && mx.x > 0 && mn.z <= 0 && mx.z > 0 { testing.expect(t, c.lod == 0) }
        testing.expect(t, mn.y < 5 && mx.y == 5)
    }
}

@(test)
test_terrain_lod_indices :: proc(t: ^testing.T) {
    n := 16
    verts := engine.r3d_terrain_grid_vertices(n)
    defer delete(verts)
    testing.expect(t, len(verts) == 17 * 17 + 4 * 17)
    for lod in 0 ..= 4 {
        idx := engine.r3d_terrain_lod_indices(n, lod)
        defer delete(idx)
        q := n >> uint(lod)
        testing.expectf(t, len(idx) == q * q * 6 + 4 * q * 6, "lod %v: %v indices", lod, len(idx))
        area: f32 = 0
        surface := q * q * 6
        pos :: proc(v: [3]f32) -> linalg.Vector3f32 { return {v[0], -v[2], v[1]} } // skirt drops 1
        for k := 0; k < len(idx); k += 3 {
            a, b, c := pos(verts[idx[k]]), pos(verts[idx[k + 1]]), pos(verts[idx[k + 2]])
            nrm := linalg.cross(b - a, c - a)
            if k < surface {
                testing.expect(t, nrm.y > 0) // CCW seen from above
                area += nrm.y * 0.5
                // only every step-th vertex is used
                for v in ([?][3]f32{verts[idx[k]], verts[idx[k + 1]], verts[idx[k + 2]]}) {
                    testing.expect(t, int(v[0]) % (1 << uint(lod)) == 0 && int(v[1]) % (1 << uint(lod)) == 0)
                }
            } else {
                mid := (a + b + c) / 3 - linalg.Vector3f32{f32(n) / 2, 0, f32(n) / 2}
                testing.expectf(t, linalg.dot(nrm, mid) > 0, "lod %v skirt tri %v faces inward", lod, k / 3)
            }
        }
        testing.expect(t, abs(area - f32(n * n)) < 1e-3) // covers the chunk exactly once
    }
}
