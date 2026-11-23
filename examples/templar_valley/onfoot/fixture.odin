package onfoot

// ============================================================================
// onfoot — Templar's on-foot test course and routes for Void Engine 3D-3,
// pure CPU (no GL), shared by `odin test` and `templar_valley --fixture`.
//
// The course is Godot Templar's OnFootFixture (src/world/onfoot_fixture.gd at
// 1356930), same local layout and the same ORIGIN floating 20 m over a flat
// heightfield, plus three additions the Godot build kept elsewhere:
//   * a village patch (house with walls and roof, a tree canopy, a market
//     awning, and a hall with a door and an interior volume) for the
//     occlusion / interior routes (Godot used the valley village);
//   * a 25 deg heightfield hillside for the F2 camera-clearance follow-up;
//   * a 50 deg heightfield bank (non-walkable terrain, not just a mesh).
// ============================================================================

import "core:math"
import "core:math/linalg"
import ve "../../../src/core"

ORIGIN :: [3]f32{96, 120, -480}
LANE_START :: [3]f32{-55, 0, 2}
COURSE_Z :: f32(18)
COURSE_START :: [3]f32{-55, 0, COURSE_Z}
GAP_FLOOR_Y :: f32(-6)
RIDGE_TOP :: f32(2.0096)  // 15 deg over 7.5 m
RAMP_RUN :: f32(6.4335)   // 3 m rise at 25 deg
WALL_X :: f32(44)         // end wall face (local)
HF_BASE :: f32(100)       // heightfield under the floating course
HILL_DEG :: f32(25)
HILL_Z0 :: f32(140)       // local z where the 25 deg hill starts (rises to +z)
HILL_RUN :: f32(60)
BANK_DEG :: f32(50)
BANK_Z0 :: f32(-140)      // local z where the 50 deg bank starts (rises to -z)
BANK_RUN :: f32(20)

world :: #force_inline proc "contextless" (local: [3]f32) -> [3]f32 { return ORIGIN + local }

Block :: struct {
    first, count: int,   // triangle range in Fixture.render_tris
    color:        [4]f32,
    occluder:     i32,   // occlusion id (-1 none)
    name:         string,
}

Fixture :: struct {
    hf:          ve.R3D_Heightfield,
    world:       ve.R3D_Collision_World,
    occ:         ve.R3D_Occlusion,
    render_tris: [dynamic]ve.R3D_Tri, // world space, per Block
    blocks:      [dynamic]Block,
    hall_group:  i32,
    house_group: i32,
}

MAT_A :: [4]f32{0.62, 0.6, 0.55, 1}
MAT_B :: [4]f32{0.5, 0.56, 0.62, 1}
MAT_OBSTACLE :: [4]f32{0.55, 0.35, 0.3, 1}
MAT_WALL :: [4]f32{0.78, 0.72, 0.6, 1}
MAT_ROOF :: [4]f32{0.55, 0.22, 0.16, 1}
MAT_TRUNK :: [4]f32{0.36, 0.25, 0.16, 1}
MAT_CANOPY :: [4]f32{0.24, 0.42, 0.2, 1}
MAT_AWNING :: [4]f32{0.75, 0.6, 0.25, 1}
MAT_GREEN :: [4]f32{0.45, 0.55, 0.38, 1}

ALL :: ve.R3D_COL_PLAYER | ve.R3D_COL_CAMERA

@(private="file")
Builder :: struct {
    f:    ^Fixture,
    tris: [dynamic]ve.R3D_Tri, // collision triangles (one mesh)
}

@(private="file")
add_hull :: proc(b: ^Builder, c: [8][3]f32, color: [4]f32, mask: u32, occ: i32 = -1, name := "") {
    faces := [6][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 4, 7, 3}, {1, 2, 6, 5}, {3, 7, 6, 2}, {0, 1, 5, 4}}
    centroid: [3]f32
    for p in c { centroid += p }
    centroid /= 8
    first := len(b.f.render_tris)
    for f in faces {
        for k in ([2][3]int{{0, 1, 2}, {0, 2, 3}}) {
            p0, p1, p2 := world(c[f[k[0]]]), world(c[f[k[1]]]), world(c[f[k[2]]])
            t := ve.r3d_tri_make(p0, p1, p2, mask)
            if linalg.length2(linalg.cross(p1 - p0, p2 - p0)) < 1e-10 { continue }
            if linalg.dot(t.n, (p0 + p1 + p2) / 3 - world(centroid)) < 0 { t = ve.r3d_tri_make(p0, p2, p1, mask) }
            append(&b.f.render_tris, t)
            if mask != 0 { append(&b.tris, t) }
        }
    }
    append(&b.f.blocks, Block{first = first, count = len(b.f.render_tris) - first, color = color, occluder = occ, name = name})
}

@(private="file")
add_box :: proc(b: ^Builder, lo, hi: [3]f32, color: [4]f32, mask: u32 = ALL, occ: i32 = -1, name := "") {
    add_hull(b, {{lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {lo.x, hi.y, lo.z},
                 {lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}}, color, mask, occ, name)
}

// Prism under a profile segment a -> b (x, top y) across z0..z1, bottom -2.
@(private="file")
prism_x :: proc(b: ^Builder, a, c: [2]f32, z0, z1: f32, color: [4]f32) {
    bot: f32 = -2
    add_hull(b, {{a.x, bot, z0}, {c.x, bot, z0}, {c.x, c.y, z0}, {a.x, a.y, z0},
                 {a.x, bot, z1}, {c.x, bot, z1}, {c.x, c.y, z1}, {a.x, a.y, z1}}, color, ALL)
}

// Prism under a profile segment a -> b (z, top y) across x0..x1.
@(private="file")
prism_z :: proc(b: ^Builder, a, c: [2]f32, x0, x1: f32, color: [4]f32) {
    bot: f32 = -2
    add_hull(b, {{x0, bot, a.x}, {x0, bot, c.x}, {x0, c.y, c.x}, {x0, a.y, a.x},
                 {x1, bot, a.x}, {x1, bot, c.x}, {x1, c.y, c.x}, {x1, a.y, a.x}}, color, ALL)
}

// Building: four walls (with an optional door gap on the +z wall), roof slab;
// walls and roof collide and are occluders of one group.
@(private="file")
add_building :: proc(b: ^Builder, lo, hi: [2]f32, wall_h, thick, roof_t: f32, door: bool, name: string) -> i32 {
    o := &b.f.occ
    g := i32(len(o.roof_of) + 100)
    wall :: proc(b: ^Builder, lo, hi: [3]f32, g: i32, name: string) {
        id := ve.r3d_occlusion_add(&b.f.occ, ve_world(lo), ve_world(hi), .Wall, g, i32(len(b.f.blocks)))
        add_box(b, lo, hi, MAT_WALL, ALL, i32(id), name)
    }
    wall(b, {lo.x, 0, lo.y}, {hi.x, wall_h, lo.y + thick}, g, name)                 // north (-z)
    wall(b, {lo.x, 0, lo.y + thick}, {lo.x + thick, wall_h, hi.y - thick}, g, name) // west
    wall(b, {hi.x - thick, 0, lo.y + thick}, {hi.x, wall_h, hi.y - thick}, g, name) // east
    if door {
        mid := (lo.x + hi.x) * 0.5
        wall(b, {lo.x, 0, hi.y - thick}, {mid - 1, wall_h, hi.y}, g, name)
        wall(b, {mid + 1, 0, hi.y - thick}, {hi.x, wall_h, hi.y}, g, name)
        wall(b, {mid - 1, 3, hi.y - thick}, {mid + 1, wall_h, hi.y}, g, name)
    } else {
        wall(b, {lo.x, 0, hi.y - thick}, {hi.x, wall_h, hi.y}, g, name)
    }
    rlo := [3]f32{lo.x - 0.4, wall_h, lo.y - 0.4}
    rhi := [3]f32{hi.x + 0.4, wall_h + roof_t, hi.y + 0.4}
    id := ve.r3d_occlusion_add(o, ve_world(rlo), ve_world(rhi), .Roof, g, i32(len(b.f.blocks)))
    add_box(b, rlo, rhi, MAT_ROOF, ALL, i32(id), name)
    return g
}

@(private="file")
ve_world :: proc "contextless" (p: [3]f32) -> [3]f32 { return world(p) }

hf_height_local :: proc "contextless" (lx, lz: f32) -> f32 {
    h := HF_BASE
    if lx >= -100 && lx <= 100 {
        if lz > HILL_Z0 { h += min(lz - HILL_Z0, HILL_RUN) * math.tan(math.to_radians(HILL_DEG)) }
        if lz < BANK_Z0 { h += min(BANK_Z0 - lz, BANK_RUN) * math.tan(math.to_radians(BANK_DEG)) }
    }
    return h
}

fixture_init :: proc(f: ^Fixture) {
    // heightfield: 512 m square at 2 m around the course
    f.hf = ve.r3d_heightfield_make(257, 2, {ORIGIN.x - 256, ORIGIN.z - 256})
    for j in 0 ..< f.hf.size {
        for i in 0 ..< f.hf.size {
            lx := f32(i) * 2 - 256
            lz := f32(j) * 2 - 256
            // smooth the x edge of the slopes over 8 m so no vertical cliff appears there
            h := hf_height_local(lx, lz)
            f.hf.heights[j * f.hf.size + i] = h
        }
    }
    f.world.hf = &f.hf
    f.world.hf_mask = ALL
    ve.r3d_occlusion_init(&f.occ)
    f.occ.hf = &f.hf

    b := Builder{f = f}
    // --- Godot OnFootFixture ---
    add_box(&b, {-60, -2, -14}, {0, 0, 10}, MAT_A)
    add_box(&b, {0, -2, -14}, {60, 0, 10}, MAT_B)
    profile := [?][2]f32{{-60, 0}, {-40, 0}, {-40 + RAMP_RUN, 3}, {-34 + RAMP_RUN, 3}, {-34 + 2 * RAMP_RUN, 0}, {-15, 0}, {-7.5, RIDGE_TOP}, {0, 0}, {6, 0}}
    for i in 0 ..< len(profile) - 1 {
        east := profile[i].x >= 0
        prism_x(&b, profile[i], profile[i + 1], 10, 26, east ? MAT_B : MAT_A)
    }
    add_box(&b, {16, -2, 10}, {46, 0, 26}, MAT_B)
    add_box(&b, {5, -0.4, COURSE_Z - 1.5}, {17, 0, COURSE_Z + 1.5}, MAT_OBSTACLE)  // bridge deck
    add_box(&b, {6, GAP_FLOOR_Y - 1, 10}, {16, GAP_FLOOR_Y, 26}, MAT_B)             // gap floor
    add_box(&b, {26, 3.2, 10}, {34, 3.8, 26}, MAT_OBSTACLE)                         // overhang
    add_box(&b, {44, 0, 10}, {45, 10, 26}, MAT_OBSTACLE)                            // end wall
    prism_z(&b, {26, 0}, {28.3094, 4}, 36, 44, MAT_OBSTACLE)                         // 60 deg slope
    prism_z(&b, {28.3094, 4}, {30, 4}, 36, 44, MAT_OBSTACLE)

    // --- village patch (occlusion / interior), local z 34..90 ---
    add_box(&b, {-60, -2, 34}, {60, 0, 90}, MAT_GREEN)
    f.house_group = add_building(&b, {-20, 50}, {-12, 56}, 3.5, 0.3, 0.5, false, "house")
    f.hall_group = add_building(&b, {20, 50}, {32, 60}, 4.0, 0.4, 0.5, true, "hall")
    {
        lo := [3]f32{20.4, 0, 50.4}
        hi := [3]f32{31.6, 4.0, 59.6}
        ve.r3d_occlusion_add_interior(&f.occ, world(lo), world(hi), f.hall_group)
    }
    // tree: trunk collides, canopy is an occluder only
    add_box(&b, {-0.25, 0, 47.75}, {0.25, 3, 48.25}, MAT_TRUNK, ALL, -1, "trunk")
    {
        lo := [3]f32{-2.5, 2.6, 45.5}
        hi := [3]f32{2.5, 6.5, 50.5}
        id := ve.r3d_occlusion_add(&f.occ, world(lo), world(hi), .Canopy, -1, i32(len(f.blocks)))
        add_box(&b, lo, hi, MAT_CANOPY, 0, i32(id), "canopy")
    }
    // market awning on four posts (posts collide, the cloth does not)
    for p in ([4][2]f32{{8, 46}, {14, 46}, {8, 52}, {14, 52}}) {
        add_box(&b, {p.x - 0.1, 0, p.y - 0.1}, {p.x + 0.1, 2.6, p.y + 0.1}, MAT_TRUNK, ALL, -1, "post")
    }
    {
        lo := [3]f32{7.6, 2.6, 45.6}
        hi := [3]f32{14.4, 2.85, 52.4}
        id := ve.r3d_occlusion_add(&f.occ, world(lo), world(hi), .Canopy, -1, i32(len(f.blocks)))
        add_box(&b, lo, hi, MAT_AWNING, 0, i32(id), "awning")
    }
    ve.r3d_world_add_mesh(&f.world, b.tris)
}

fixture_destroy :: proc(f: ^Fixture) {
    f.world.hf = nil
    ve.r3d_world_destroy(&f.world)
    ve.r3d_occlusion_destroy(&f.occ)
    ve.r3d_heightfield_destroy(&f.hf)
    delete(f.render_tris)
    delete(f.blocks)
    f^ = {}
}
