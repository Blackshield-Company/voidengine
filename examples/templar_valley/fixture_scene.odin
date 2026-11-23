package templar_valley

// --fixture-shots: GL render of the on-foot fixture (greybox course, village,
// 25 deg hill) at the routes' Shot ops. Each shot is saved twice: the player's
// follow-camera view (<name>.ppm) and a side view with the boom drawn
// (<name>-side.ppm; red = wanted boom, green = boom after the sweep).

import "core:fmt"
import "core:math"
import "core:math/linalg"
import ve "../../src/core"
import onfoot "onfoot"

Scene_Mesh :: struct {
    model: ve.R3D_Model,
    gl:    ve.GL3D_Model,
    pose:  ve.R3D_Model_Pose,
}

Fx_Scene :: struct {
    fx:      onfoot.Fixture,
    blocks:  [dynamic]^Scene_Mesh,
    ground, capsule, marker: ^Scene_Mesh,
    route:   string,
    shots:   int,
}

mesh_from_tris :: proc(tris: []ve.R3D_Tri, color: [4]f32, rough: f32 = 0.85) -> ^Scene_Mesh {
    m := new(Scene_Mesh)
    p: ve.R3D_Model_Prim
    p.node = -1
    p.skin = -1
    p.material = 0
    p.bounds_min = {max(f32), max(f32), max(f32)}
    p.bounds_max = -p.bounds_min
    for t in tris {
        for v in ([3][3]f32{t.a, t.b, t.c}) {
            append(&p.indices, u32(len(p.vertices)))
            append(&p.vertices, ve.R3D_Skin_Vertex{pos = v, normal = t.n, uv = {v.x, v.z} * 0.5, tangent = {1, 0, 0, 1}, weights = {1, 0, 0, 0}})
            p.bounds_min = linalg.min(p.bounds_min, v)
            p.bounds_max = linalg.max(p.bounds_max, v)
        }
    }
    append(&m.model.prims, p)
    mt := ve.r3d_default_material()
    mt.base_color = color
    mt.roughness = rough
    mt.double_sided = true
    append(&m.model.materials, mt)
    m.gl = ve.gl3d_model_upload(&m.model)
    ve.r3d_model_pose_init(&m.pose, &m.model)
    return m
}

mesh_destroy :: proc(m: ^Scene_Mesh) {
    ve.gl3d_model_destroy(&m.gl)
    ve.r3d_model_pose_destroy(&m.pose)
    ve.r3d_model_destroy(&m.model)
    free(m)
}

// Capsule with its feet at the origin (radius r, total height h).
capsule_tris :: proc(r, h: f32, allocator := context.temp_allocator) -> [dynamic]ve.R3D_Tri {
    out := make([dynamic]ve.R3D_Tri, allocator)
    SEG :: 20
    RINGS :: 8 // per hemisphere
    // profile: (radius, y) from bottom pole to top pole
    prof: [dynamic][2]f32
    defer delete(prof)
    for i in 0 ..= RINGS {
        a := -math.PI / 2 + f32(i) / RINGS * math.PI / 2
        append(&prof, [2]f32{r * math.cos(a), r + r * math.sin(a)})
    }
    for i in 0 ..= RINGS {
        a := f32(i) / RINGS * math.PI / 2
        append(&prof, [2]f32{r * math.cos(a), h - r + r * math.sin(a)})
    }
    for k in 0 ..< len(prof) - 1 {
        p0, p1 := prof[k], prof[k + 1]
        for s in 0 ..< SEG {
            a0 := f32(s) / SEG * math.TAU
            a1 := f32(s + 1) / SEG * math.TAU
            v00 := [3]f32{p0.x * math.cos(a0), p0.y, p0.x * math.sin(a0)}
            v01 := [3]f32{p0.x * math.cos(a1), p0.y, p0.x * math.sin(a1)}
            v10 := [3]f32{p1.x * math.cos(a0), p1.y, p1.x * math.sin(a0)}
            v11 := [3]f32{p1.x * math.cos(a1), p1.y, p1.x * math.sin(a1)}
            if p0.x > 1e-4 { append(&out, ve.r3d_tri_make(v00, v10, v01, 1)) }
            if p1.x > 1e-4 { append(&out, ve.r3d_tri_make(v01, v10, v11, 1)) }
        }
    }
    return out
}

fx_scene_init :: proc(g: ^Game) {
    sc := &g.fxs
    onfoot.fixture_init(&sc.fx)
    fx := &sc.fx
    for b in fx.blocks {
        append(&sc.blocks, mesh_from_tris(fx.render_tris[b.first:b.first + b.count], b.color))
    }
    // the heightfield, rendered diagonal (a,c,b),(b,c,d)
    hf := &fx.hf
    tris := make([dynamic]ve.R3D_Tri, context.temp_allocator)
    for j in 0 ..< hf.size - 1 {
        for i in 0 ..< hf.size - 1 {
            P :: proc(hf: ^ve.R3D_Heightfield, i, j: int) -> [3]f32 {
                return {hf.origin.x + f32(i) * hf.spacing, hf.heights[j * hf.size + i], hf.origin.y + f32(j) * hf.spacing}
            }
            a, b, c, d := P(hf, i, j), P(hf, i + 1, j), P(hf, i, j + 1), P(hf, i + 1, j + 1)
            append(&tris, ve.r3d_tri_make(a, c, b, 1), ve.r3d_tri_make(b, c, d, 1))
        }
    }
    sc.ground = mesh_from_tris(tris[:], {0.36, 0.45, 0.28, 1}, 0.95)
    k := ve.r3d_kcc_default()
    sc.capsule = mesh_from_tris(capsule_tris(k.radius, k.height)[:], {0.2, 0.45, 0.95, 1}, 0.5)
    sc.marker = mesh_from_tris(capsule_tris(0.18, 0.36)[:], {1.0, 0.85, 0.1, 1}, 0.4)
}

fx_scene_destroy :: proc(g: ^Game) {
    sc := &g.fxs
    for m in sc.blocks { mesh_destroy(m) }
    delete(sc.blocks)
    if sc.ground != nil { mesh_destroy(sc.ground) }
    if sc.capsule != nil { mesh_destroy(sc.capsule) }
    if sc.marker != nil { mesh_destroy(sc.marker) }
    onfoot.fixture_destroy(&sc.fx)
}

@(private="file")
block_alpha :: proc(sc: ^Fx_Scene, b: onfoot.Block) -> (alpha: f32, hidden: bool) {
    if b.occluder < 0 { return 1, false }
    return ve.r3d_occlusion_alpha(&sc.fx.occ, b.occluder)
}

draw_fx_scene :: proc(g: ^Game, r: ^onfoot.Run, side: bool) {
    sc := &g.fxs
    rr := g.r
    c := &r.cam
    k := &r.kcc
    ve.r3d_follow_apply(c, &g.cam)
    want := c.pivot + ve.r3d_follow_boom_direction(ve.r3d_follow_displayed_yaw(c), c.pitch_deg) * c.zoom_current
    actual := g.cam.position
    if side {
        // look at the player from the side of the boom, a little above
        boom := actual - c.pivot
        hz := linalg.normalize([2]f32{boom.x, boom.z} + {1e-4, 0})
        sidev := [2]f32{-hz.y, hz.x}
        focus := (c.pivot + want) * 0.5
        dist := max(linalg.length(want - c.pivot) * 0.9, 9)
        eye := focus + [3]f32{sidev.x, 0, sidev.y} * dist + {0, 2, 0}
        d := focus - eye
        g.cam.position = eye
        g.cam.yaw = math.atan2(d.x, -d.z)
        g.cam.pitch = math.atan2(d.y, linalg.length([2]f32{d.x, d.z}))
    }
    cm := linalg.matrix4_translate_f32(k.pos)
    mm := linalg.matrix4_translate_f32(actual - {0, 0.18, 0})
    ve.gl3d_env_update(rr, &g.env, &g.cam)
    for ci in 0 ..< ve.R3D_CSM_CASCADES {
        ve.gl3d_env_shadow_begin(&g.env, ci)
        ve.gl3d_pbr_shadow_begin_vp(&g.pbr, g.env.csm.light_vp[ci])
        ve.gl3d_pbr_draw_model_shadow(rr, &g.pbr, &sc.ground.gl, &sc.ground.pose, linalg.MATRIX4F32_IDENTITY)
        for m, i in sc.blocks {
            a, hidden := block_alpha(sc, sc.fx.blocks[i])
            if hidden || a < 0.99 { continue }
            ve.gl3d_pbr_draw_model_shadow(rr, &g.pbr, &m.gl, &m.pose, linalg.MATRIX4F32_IDENTITY)
        }
        ve.gl3d_pbr_draw_model_shadow(rr, &g.pbr, &sc.capsule.gl, &sc.capsule.pose, cm)
    }
    ve.gl3d_env_shadow_end(rr)
    ve.gl3d_set_camera(rr, &g.cam)
    ve.gl3d_begin_frame(rr)
    ve.gl3d_env_draw_sky(rr, &g.env)
    ve.gl3d_pbr_draw_model(rr, &g.pbr, &sc.ground.gl, &sc.ground.pose, linalg.MATRIX4F32_IDENTITY)
    for m, i in sc.blocks {
        a, hidden := block_alpha(sc, sc.fx.blocks[i])
        if hidden || a < 0.99 { continue }
        ve.gl3d_pbr_draw_model(rr, &g.pbr, &m.gl, &m.pose, linalg.MATRIX4F32_IDENTITY)
    }
    ve.gl3d_pbr_draw_model(rr, &g.pbr, &sc.capsule.gl, &sc.capsule.pose, cm)
    if side {
        ve.gl3d_pbr_draw_model(rr, &g.pbr, &sc.marker.gl, &sc.marker.pose, mm)
        ve.gl3d_debug_line(rr, c.pivot, want, {1, 0.15, 0.1, 1})
        ve.gl3d_debug_line(rr, c.pivot, actual, {0.1, 1, 0.2, 1})
    }
    for m, i in sc.blocks {
        a, hidden := block_alpha(sc, sc.fx.blocks[i])
        if hidden || a >= 0.99 { continue }
        draw_model_alpha(rr, &g.pbr, &m.gl, &m.pose, linalg.MATRIX4F32_IDENTITY, a)
    }
    ve.gl3d_end_frame(rr)
}

@(private="file")
shot_hook :: proc(r: ^onfoot.Run, op: ^onfoot.Op) -> bool {
    if op.kind != .Shot { return true }
    g := cast(^Game)r.user
    for side in ([]bool{false, true}) {
        draw_fx_scene(g, r, side)
        save_ppm(g, fmt.tprintf("%s/%s%s.ppm", g.fixture_shots, op.name, side ? "-side" : ""))
    }
    c := &r.cam
    fmt.printfln("templar_valley: %s route=%s feet=(%.2f, %.2f, %.2f) on_floor=%v floor_n=%v zoom=%.1f boom=%.2f m interior=%v faded/cut=%v",
        op.name, r.route, r.kcc.pos.x, r.kcc.pos.y, r.kcc.pos.z, r.kcc.on_floor, r.kcc.floor_n, c.zoom_current, c.effective_distance, c.interior_mode,
        ve.r3d_occlusion_cleared_counts(&r.fx.occ))
    g.fxs.shots += 1
    return true
}

// All three fixture routes, rendering at their Shot ops. Returns pass/fail.
fixture_shots_run :: proc(g: ^Game) -> bool {
    ok := true
    for name in ([]string{"fixture", "hill", "occlusion"}) {
        // fresh fixture state per route (geometry is the same as the meshes)
        onfoot.fixture_destroy(&g.fxs.fx)
        onfoot.fixture_init(&g.fxs.fx)
        r: onfoot.Run
        r.user = g
        onfoot.run_init(&r, &g.fxs.fx, name)
        r.on_step = shot_hook
        ops: [dynamic]onfoot.Op
        switch name {
        case "fixture": ops = onfoot.route_fixture(context.temp_allocator)
        case "hill": ops = onfoot.route_hill(context.temp_allocator)
        case "occlusion": ops = onfoot.route_occlusion(context.temp_allocator)
        }
        onfoot.run_ops(&r, ops[:])
        onfoot.judge_common(&r)
        switch name {
        case "fixture": onfoot.judge_fixture(&r)
        case "hill": onfoot.judge_hill(&r)
        case "occlusion": onfoot.judge_occlusion(&r)
        }
        onfoot.print_checks(&r)
        ok &= onfoot.all_ok(&r)
        onfoot.run_destroy(&r)
    }
    fmt.printfln("templar_valley: %d fixture shots", g.fxs.shots)
    return ok
}
