package voidengine_test

// Tests for collide3d.odin: BVH queries against brute force, closest-point
// primitives, ray casts on meshes and the heightfield, capsule sweeps (exact
// time of impact, no penetration, tangential slides not blocked), and
// depenetration.

import "core:math"
import "core:math/linalg"
import "core:math/rand"
import "core:testing"
import engine "../src/core"

@(private="file")
box_tris :: proc(out: ^[dynamic]engine.R3D_Tri, lo, hi: [3]f32, mask: u32 = engine.R3D_COL_PLAYER | engine.R3D_COL_CAMERA) {
    c := [8][3]f32{{lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z},
                   {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}}
    faces := [6][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 4, 7, 3}, {1, 2, 6, 5}, {3, 7, 6, 2}, {0, 1, 5, 4}}
    for f in faces {
        append(out, engine.r3d_tri_make(c[f[0]], c[f[1]], c[f[2]], mask), engine.r3d_tri_make(c[f[0]], c[f[2]], c[f[3]], mask))
    }
}

@(private="file")
floor_world :: proc(w: ^engine.R3D_Collision_World) {
    tris: [dynamic]engine.R3D_Tri
    box_tris(&tris, {-50, -1, -50}, {50, 0, 50})
    engine.r3d_world_add_mesh(w, tris)
}

@(test)
test_bvh_query_matches_brute_force :: proc(t: ^testing.T) {
    r := rand.create(7)
    context.random_generator = rand.default_random_generator(&r)
    tris: [dynamic]engine.R3D_Tri
    for _ in 0 ..< 500 {
        c := [3]f32{rand.float32_range(-40, 40), rand.float32_range(-5, 5), rand.float32_range(-40, 40)}
        o := proc() -> [3]f32 { return {rand.float32_range(-2, 2), rand.float32_range(-2, 2), rand.float32_range(-2, 2)} }
        append(&tris, engine.r3d_tri_make(c + o(), c + o(), c + o(), 1))
    }
    brute := make([]engine.R3D_Tri, len(tris)); defer delete(brute)
    copy(brute, tris[:])
    m := engine.r3d_tri_mesh_build(tris)
    defer engine.r3d_tri_mesh_destroy(&m)
    testing.expect(t, len(m.nodes) > 1)
    out: [dynamic]engine.R3D_Tri
    defer delete(out)
    for _ in 0 ..< 50 {
        c := [3]f32{rand.float32_range(-40, 40), 0, rand.float32_range(-40, 40)}
        bmin, bmax := c - {4, 3, 4}, c + {4, 3, 4}
        clear(&out)
        engine.r3d_tri_mesh_query(&m, bmin, bmax, 1, &out)
        n := 0
        for &tr in brute {
            a, b := engine.r3d_tri_bounds(&tr)
            if engine.r3d_aabb_overlap(bmin, bmax, a, b) { n += 1 }
        }
        testing.expect_value(t, len(out), n)
    }
}

@(test)
test_closest_points :: proc(t: ^testing.T) {
    tri := engine.r3d_tri_make({0, 0, 0}, {0, 0, 2}, {2, 0, 0}, 1)
    testing.expect(t, tri.n.y > 0.999)
    q := engine.r3d_closest_point_tri({0.5, 3, 0.5}, tri.a, tri.b, tri.c)
    testing.expect(t, linalg.length(q - {0.5, 0, 0.5}) < 1e-6)
    q = engine.r3d_closest_point_tri({-1, 1, -1}, tri.a, tri.b, tri.c)
    testing.expect(t, linalg.length(q) < 1e-6) // vertex region
    // vertical segment above the triangle interior
    s, c, d2 := engine.r3d_closest_seg_tri({0.5, 1, 0.5}, {0.5, 3, 0.5}, &tri)
    testing.expect(t, abs(math.sqrt(d2) - 1) < 1e-5)
    testing.expect(t, abs(s.y - 1) < 1e-5 && abs(c.y) < 1e-6)
    // segment piercing the triangle: distance 0
    _, _, d2 = engine.r3d_closest_seg_tri({0.5, -1, 0.5}, {0.5, 1, 0.5}, &tri)
    testing.expect(t, d2 == 0)
    // horizontal segment beside an edge (edge-edge case)
    _, _, d2 = engine.r3d_closest_seg_tri({-1, 0.5, 1}, {-1, 0.5, -1}, &tri)
    testing.expect(t, abs(math.sqrt(d2) - math.sqrt(f32(1.25))) < 1e-5)
}

@(test)
test_raycast_mesh_and_heightfield :: proc(t: ^testing.T) {
    w: engine.R3D_Collision_World
    defer engine.r3d_world_destroy(&w)
    tris: [dynamic]engine.R3D_Tri
    box_tris(&tris, {0, 0, 0}, {2, 3, 2})
    engine.r3d_world_add_mesh(&w, tris)
    tt, n, hit := engine.r3d_raycast(&w, {1, 10, 1}, {0, -1, 0}, 20, 1)
    testing.expect(t, hit && abs(tt - 7) < 1e-5 && n.y > 0.999)
    _, _, hit = engine.r3d_raycast(&w, {5, 10, 5}, {0, -1, 0}, 20, 1)
    testing.expect(t, !hit)
    // camera-only mask on a player-only mesh: no hit
    tris2: [dynamic]engine.R3D_Tri
    box_tris(&tris2, {10, 0, 10}, {12, 3, 12}, engine.R3D_COL_PLAYER)
    engine.r3d_world_add_mesh(&w, tris2)
    _, _, hit = engine.r3d_raycast(&w, {11, 10, 11}, {0, -1, 0}, 20, engine.R3D_COL_CAMERA)
    testing.expect(t, !hit)
    // heightfield collider = rendered triangulation
    hf := engine.r3d_heightfield_make(17, 2, {-16, -16})
    defer engine.r3d_heightfield_destroy(&hf)
    for j in 0 ..< 17 { for i in 0 ..< 17 { hf.heights[j * 17 + i] = math.sin(f32(i) * 0.7) * 2 + f32(j) * 0.3 } }
    w2: engine.R3D_Collision_World
    w2.hf = &hf
    w2.hf_mask = 1
    for k in 0 ..< 40 {
        x := -14 + f32(k) * 0.71
        z := 13 - f32(k) * 0.63
        g := engine.r3d_ground_below(&w2, {x, 50, z}, 1)
        testing.expectf(t, abs(g - engine.r3d_hf_height_tri(&hf, x, z)) < 1e-3, "hf ray %v %v: %v vs %v", x, z, g, engine.r3d_hf_height_tri(&hf, x, z))
    }
    // at grid vertices the triangulated and bilinear surfaces agree
    testing.expect(t, abs(engine.r3d_hf_height_tri(&hf, 2, 4) - engine.r3d_hf_height(&hf, 2, 4)) < 1e-5)
    w2.hf = nil
    engine.r3d_world_destroy(&w2)
}

@(test)
test_capsule_sweep_toi_and_slide :: proc(t: ^testing.T) {
    w: engine.R3D_Collision_World
    defer engine.r3d_world_destroy(&w)
    floor_world(&w)
    r: f32 = 0.4
    // fall 2 m onto the floor: stops at skin distance
    p := [3]f32{0, 1.5, 0}
    q := p + {0, 1, 0}
    h := engine.r3d_sweep_capsule(&w, p, q, r, {0, -3, 0}, 1, 0.01)
    testing.expect(t, h.hit)
    land := p.y - 3 * h.t
    testing.expectf(t, abs(land - (r + 0.01)) < 1e-3, "landing core bottom at %v", land)
    testing.expect(t, h.normal.y > 0.999)
    // resting in contact: a horizontal slide is not blocked, pressing down is
    p = {0, r + 0.01, 0}
    q = p + {0, 1, 0}
    h = engine.r3d_sweep_capsule(&w, p, q, r, {3, 0, 0}, 1, 0.01)
    testing.expect(t, !h.hit && h.t == 1)
    h = engine.r3d_sweep_capsule(&w, p, q, r, {0, -0.5, 0}, 1, 0.01)
    testing.expect(t, h.hit && h.t == 0)
    // moving away is never blocked
    h = engine.r3d_sweep_capsule(&w, p, q, r, {0, 0.5, 0}, 1, 0.01)
    testing.expect(t, !h.hit)
}

@(test)
test_sweep_never_penetrates :: proc(t: ^testing.T) {
    w: engine.R3D_Collision_World
    defer engine.r3d_world_destroy(&w)
    tris: [dynamic]engine.R3D_Tri
    box_tris(&tris, {-1, 0, -1}, {1, 2, 1})
    box_tris(&tris, {3, 0, -4}, {3.5, 3, 4})
    engine.r3d_world_add_mesh(&w, tris)
    rg := rand.create(11)
    context.random_generator = rand.default_random_generator(&rg)
    hits := 0
    for _ in 0 ..< 400 {
        a := [3]f32{rand.float32_range(-6, 6), rand.float32_range(-1, 4), rand.float32_range(-6, 6)}
        if d, _ := engine.r3d_capsule_distance(&w, a, a + {0, 0.8, 0}, 0.35, 1, 1); d < 0.02 { continue }
        m := [3]f32{rand.float32_range(-8, 8), rand.float32_range(-3, 3), rand.float32_range(-8, 8)}
        h := engine.r3d_sweep_capsule(&w, a, a + {0, 0.8, 0}, 0.35, m, 1, 0.01)
        e := a + m * h.t
        d, _ := engine.r3d_capsule_distance(&w, e, e + {0, 0.8, 0}, 0.35, 1, 1)
        testing.expectf(t, d >= 0.005, "capsule ended %v m from geometry", d)
        if h.hit { hits += 1; testing.expectf(t, d < 0.02, "hit but %v m away", d) }
    }
    testing.expect(t, hits > 50)
}

@(test)
test_sphere_sweep_wall_and_depenetrate :: proc(t: ^testing.T) {
    w: engine.R3D_Collision_World
    defer engine.r3d_world_destroy(&w)
    tris: [dynamic]engine.R3D_Tri
    box_tris(&tris, {5, -5, -5}, {6, 5, 5})
    engine.r3d_world_add_mesh(&w, tris)
    h := engine.r3d_sweep_sphere(&w, {0, 0, 0}, 0.35, {10, 0, 0}, 1, 0)
    testing.expect(t, h.hit)
    testing.expectf(t, abs(h.t * 10 - 4.65) < 1e-3, "sphere stops at %v", h.t * 10)
    testing.expect(t, h.normal.x < -0.999)
    testing.expect(t, engine.r3d_sphere_overlaps(&w, {4.8, 0, 0}, 0.35, 1))
    testing.expect(t, !engine.r3d_sphere_overlaps(&w, {4.5, 0, 0}, 0.35, 1))
    fix := engine.r3d_capsule_depenetrate(&w, {4.8, -0.5, 0}, {4.8, 0.5, 0}, 0.35, 1, 0.01)
    testing.expectf(t, abs((4.8 + fix.x) - (5 - 0.36)) < 2e-3 && abs(fix.y) < 1e-4, "depenetrate %v", fix)
}
