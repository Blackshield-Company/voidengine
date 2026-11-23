package templar_valley

// ============================================================================
// 3D-3 on foot: collision, character controller, follow camera, occlusion.
//
//   --fixture ROUTE|all  headless (no window/GL): run the deterministic
//                        on-foot routes (fixture, hill, occlusion, valley)
//                        and print the T-PARITY checks; exit 1 on any FAIL
//   --csv DIR            with --fixture: write per-step telemetry CSVs
//   --fixture-shots DIR  render the 3d3-* shots of the fixture routes (GL)
//   --onfoot             walk the valley: WASD/arrows move, Shift run,
//                        Q/E snap 45 deg, Z/X zoom, Esc quit
//   --bench-onfoot N     walk the valley trek for N frames (collision,
//                        follow camera and occluder fades on), print BENCH
// ============================================================================

import "core:fmt"
import "core:math"
import "core:math/linalg"
import "core:os"
import "core:slice"
import "core:time"
import gl "vendor:OpenGL"
import ve "../../src/core"
import onfoot "onfoot"

// Godot OnFootRoutes TREK (world x, z): the east-valley walk.
TREK := [?][2]f32{{100, -560}, {200, -560}, {330, -560}, {330, -400}, {240, -400}, {150, -400}}

Faded :: struct { prop: i32, alpha: f32 }

Onfoot :: struct {
    active:   bool,
    fx:       onfoot.Fixture, // valley: hf shares g.hf's heights
    prop_occ: [][2]i32,       // per prop: occluder ids (-1 = none)
    run:      onfoot.Run,
    ops:      [dynamic]onfoot.Op,
    faded:    [dynamic]Faded,
    face:     f32,            // heading of the player model
    acc:      f32,
    step_ms:  [dynamic]f64,
}

// --- valley colliders and occluders -----------------------------------------

@(private="file")
box_corners :: proc(m: linalg.Matrix4f32, bmin, bmax: [3]f32) -> (c: [8][3]f32, lo, hi: [3]f32) {
    lo = {max(f32), max(f32), max(f32)}
    hi = -lo
    for i in 0 ..< 8 {
        q := [3]f32{i & 1 != 0 ? bmax.x : bmin.x, i & 2 != 0 ? bmax.y : bmin.y, i & 4 != 0 ? bmax.z : bmin.z}
        c[i] = ve.r3d_mat4_transform_point(m, q)
        lo = linalg.min(lo, c[i]); hi = linalg.max(hi, c[i])
    }
    return
}

// 12 triangles of a (transformed) box hull.
@(private="file")
add_hull :: proc(tris: ^[dynamic]ve.R3D_Tri, c: [8][3]f32, mask: u32) {
    FACES :: [6][4]int{{0, 2, 6, 4}, {1, 5, 7, 3}, {0, 4, 5, 1}, {2, 3, 7, 6}, {0, 1, 3, 2}, {4, 6, 7, 5}}
    faces := FACES
    center: [3]f32
    for p in c { center += p / 8 }
    for f in faces {
        a, b, d, e := c[f[0]], c[f[1]], c[f[2]], c[f[3]]
        t0 := ve.r3d_tri_make(a, b, d, mask)
        // outward normals
        if linalg.dot(t0.n, (a + d) * 0.5 - center) < 0 {
            append(tris, ve.r3d_tri_make(a, d, b, mask), ve.r3d_tri_make(a, e, d, mask))
        } else {
            append(tris, t0, ve.r3d_tri_make(a, d, e, mask))
        }
    }
}

// Houses: box hull from the model bounds (player + camera), a wall and a
// roof occluder. Trees: a thin trunk box and a canopy fade occluder.
// Clutter: player-only hull and a small-prop occluder.
build_valley_onfoot :: proc(g: ^Game) {
    o := &g.of
    o.fx.hf = g.hf
    o.fx.world.hf = &o.fx.hf
    o.fx.world.hf_mask = ve.R3D_COL_PLAYER | ve.R3D_COL_CAMERA
    ve.r3d_occlusion_init(&o.fx.occ)
    o.fx.occ.hf = &o.fx.hf
    o.prop_occ = make([][2]i32, len(g.props))
    tris: [dynamic]ve.R3D_Tri // owned by the mesh
    t0 := time.tick_now()
    for &p, i in g.props {
        o.prop_occ[i] = {-1, -1}
        a := g.assets[p.asset]
        c, lo, hi := box_corners(p.mat, a.bmin, a.bmax)
        switch p.kind {
        case 'b':
            add_hull(&tris, c, ve.R3D_COL_PLAYER | ve.R3D_COL_CAMERA)
            split := lo.y + 0.62 * (hi.y - lo.y)
            o.prop_occ[i][0] = i32(ve.r3d_occlusion_add(&o.fx.occ, lo, {hi.x, split, hi.z}, .Wall, i32(i), i32(i)))
            o.prop_occ[i][1] = i32(ve.r3d_occlusion_add(&o.fx.occ, {lo.x, split, lo.z}, hi, .Roof, i32(i), i32(i)))
        case 't':
            base := ve.r3d_mat4_transform_point(p.mat, {0, 0, 0})
            T :: f32(0.3)
            tb := [8][3]f32{}
            for k in 0 ..< 8 {
                tb[k] = base + {k & 1 != 0 ? T : -T, k & 2 != 0 ? 4 : -0.5, k & 4 != 0 ? T : -T}
            }
            add_hull(&tris, tb, ve.R3D_COL_PLAYER | ve.R3D_COL_CAMERA)
            mid := (lo + hi) * 0.5
            ext := (hi - lo) * 0.5 * 0.8
            o.prop_occ[i][0] = i32(ve.r3d_occlusion_add(&o.fx.occ, {mid.x - ext.x, lo.y + 0.3 * (hi.y - lo.y), mid.z - ext.z}, {mid.x + ext.x, hi.y, mid.z + ext.z}, .Canopy, -1, i32(i)))
        case 'c':
            add_hull(&tris, c, ve.R3D_COL_PLAYER)
            o.prop_occ[i][0] = i32(ve.r3d_occlusion_add(&o.fx.occ, lo, hi, .Prop, -1, i32(i)))
        }
    }
    n_tris := len(tris)
    ve.r3d_world_add_mesh(&o.fx.world, tris)
    fmt.printfln("templar_valley: on-foot colliders %d tris, %d occluders in %.0f ms", n_tris, len(o.fx.occ.occluders), time.duration_milliseconds(time.tick_since(t0)))
}

destroy_valley_onfoot :: proc(g: ^Game) {
    o := &g.of
    o.fx.world.hf = nil
    ve.r3d_world_destroy(&o.fx.world)
    ve.r3d_occlusion_destroy(&o.fx.occ)
    delete(o.prop_occ)
    delete(o.ops)
    delete(o.faded)
    delete(o.step_ms)
    onfoot.run_destroy(&o.run)
}

prop_alpha :: proc(g: ^Game, pi: int) -> f32 {
    if !g.of.active || pi >= len(g.of.prop_occ) { return 1 }
    alpha: f32 = 1
    for id in g.of.prop_occ[pi] {
        if id < 0 { continue }
        a, hidden := ve.r3d_occlusion_alpha(&g.of.fx.occ, id)
        alpha = min(alpha, hidden ? 0.15 : a)
    }
    return alpha
}

// --- the valley route ---------------------------------------------------------

@(private="file")
slope_deg_at :: proc(hf: ^ve.R3D_Heightfield, x, z: f32) -> f32 {
    n := ve.r3d_hf_normal(hf, x, z)
    return math.to_degrees(math.acos(clamp(n.y, -1, 1)))
}

// Nearest ~25 deg patch to the trek start with no props within 10 m.
find_slope_spot :: proc(g: ^Game) -> (at: [2]f32, up: [2]f32, ok: bool) {
    best := max(f32)
    for z := f32(-900); z <= 900; z += 4 {
        for x := f32(-900); x <= 900; x += 4 {
            s := slope_deg_at(&g.hf, x, z)
            if s < 24.5 || s > 27 { continue }
            d := linalg.length([2]f32{x, z} - TREK[0])
            if d >= best { continue }
            good := true
            for k in 0 ..< 8 {
                a := f32(k) * math.TAU / 8
                for rr in ([]f32{3, 6}) {
                    ss := slope_deg_at(&g.hf, x + rr * math.cos(a), z + rr * math.sin(a))
                    if ss < 18 || ss > 34 { good = false }
                }
            }
            if !good { continue }
            for &p in g.props {
                if linalg.length([2]f32{p.center.x - x, p.center.z - z}) < 10 + p.radius { good = false; break }
            }
            if !good { continue }
            best = d
            at = {x, z}
            n := ve.r3d_hf_normal(&g.hf, x, z)
            up = -linalg.normalize([2]f32{n.x, n.z})
            ok = true
        }
    }
    return
}

route_valley :: proc(g: ^Game, ops: ^[dynamic]onfoot.Op) {
    gy :: proc(g: ^Game, xz: [2]f32) -> [3]f32 { return {xz.x, ve.r3d_hf_height(&g.hf, xz.x, xz.y), xz.y} }
    append(ops, onfoot.Op{kind = .Reset, at = gy(g, TREK[0]), yaw = 0, zoom = 14, seg = "reset"})
    append(ops, onfoot.idle(1.0, "settle"))
    for k in 1 ..< len(TREK) {
        append(ops, onfoot.Op{kind = .Go, to = TREK[k], run = k % 2 == 0, pad = k % 3 != 0, tol = 1.0, avoid = true, seg = "trek"})
    }
    at, up, ok := find_slope_spot(g)
    if ok {
        fmt.printfln("templar_valley: 25 deg valley slope at (%.0f, %.0f), %.1f deg", at.x, at.y, slope_deg_at(&g.hf, at.x, at.y))
        append(ops, onfoot.Op{kind = .Reset, at = gy(g, at), yaw = 0, zoom = 14, seg = "slope_reset"})
        append(ops, onfoot.idle(0.5, "slope_settle"))
        for z in ([]f32{14, 20}) {
            append(ops, onfoot.zoom(z, "slope_zoom"))
            for y in 0 ..< 8 { append(ops, onfoot.face(y, "slope_face"), onfoot.idle(0.4, "slope_hold")) }
        }
        side := [2]f32{-up.y, up.x}
        append(ops, onfoot.Op{kind = .Go, to = at + up * 6, run = false, pad = true, tol = 0.8, seg = "slope_up"})
        append(ops, onfoot.Op{kind = .Go, to = at + up * 6 + side * 8, run = false, pad = true, tol = 0.8, seg = "slope_across"})
        append(ops, onfoot.Op{kind = .Go, to = at + side * 8, run = true, pad = false, tol = 0.8, seg = "slope_down"})
    }
}

judge_valley :: proc(r: ^onfoot.Run) {
    onfoot.judge_common(r)
    onfoot.judge_readable(r)
    s := &r.sum
    onfoot.add_check(r, "trek_distance_m", s.dist_m >= 200, fmt.tprintf("%.1f", s.dist_m), fmt.tprint(">= 200"))
    slope_max: f32 = 0
    clear_min: f32 = max(f32)
    for x in r.rows {
        if len(x.seg) > 6 && x.seg[:6] == "slope_" && x.on_floor {
            slope_max = max(slope_max, x.slope_deg)
            if x.slope_deg > 20 { clear_min = min(clear_min, x.cam_clear) }
        }
    }
    onfoot.add_check(r, "valley_slope_reached_deg", slope_max >= 24, fmt.tprintf("%.2f", slope_max), fmt.tprint(">= 24"))
    onfoot.add_check(r, "valley_slope_clearance_m", clear_min >= 0.5, fmt.tprintf("%.3f", clear_min), fmt.tprint(">= 0.5 on > 20 deg ground (F2)"))
}

// Headless valley state: terrain + props, no GL.
valley_headless_init :: proc(g: ^Game) {
    g.headless = true
    valley_init(&g.valley)
    g.hf = build_heightfield(&g.valley)
    for n in ([]string{"pine_1", "pine_2"}) { append(&g.a_pine, load_asset(g, n, fmt.tprintf("%s/%s.glb", FETCHED, n))) }
    for n in ([]string{"common_tree_1", "common_tree_2"}) { append(&g.a_common, load_asset(g, n, fmt.tprintf("%s/%s.glb", FETCHED, n))) }
    for n in ([]string{"house_1", "house_2", "house_3"}) { append(&g.a_house, load_asset(g, n, fmt.tprintf("%s/%s.glb", FETCHED, n))) }
    for n in ([]string{"crate", "barrel", "hay", "rock_medium_1"}) { append(&g.a_clutter, load_asset(g, n, fmt.tprintf("%s/%s.glb", FETCHED, n))) }
    load_props(g)
}

// --fixture: returns the process exit code.
run_fixture_headless :: proc(g: ^Game, which: string, csv_dir: string) -> int {
    if len(csv_dir) > 0 { os.make_directory(csv_dir) }
    names := which == "all" ? []string{"fixture", "hill", "occlusion", "valley"} : []string{which}
    failed := 0
    for name in names {
        r: onfoot.Run
        t0 := time.tick_now()
        ok: bool
        if name == "valley" {
            if !os.exists(FETCHED) { fmt.println("ROUTE valley: SKIP (assets not fetched)"); continue }
            valley_headless_init(g)
            build_valley_onfoot(g)
            onfoot.run_init(&r, &g.of.fx, "valley")
            r.snap_every_s = 3
            r.zoom_every_s = 9
            route_valley(g, &g.of.ops)
            fmt.printfln("  valley setup (terrain, props, colliders, slope search) %.0f ms", time.duration_milliseconds(time.tick_since(t0)))
            t0 = time.tick_now()
            ok = onfoot.run_ops(&r, g.of.ops[:])
            if ok { judge_valley(&r) }
        } else {
            fx: onfoot.Fixture
            onfoot.fixture_init(&fx)
            ok = onfoot.run_route(&r, &fx, name)
            defer onfoot.fixture_destroy(&fx)
        }
        ms := time.duration_milliseconds(time.tick_since(t0))
        if !ok { fmt.eprintln("templar_valley: unknown route", name); failed += 1; continue }
        onfoot.print_checks(&r)
        fmt.printfln("  sim cost %.1f ms wall for %d steps (%.4f ms/step)", ms, len(r.rows), ms / f64(max(len(r.rows), 1)))
        if !onfoot.all_ok(&r) { failed += 1 }
        if len(csv_dir) > 0 { onfoot.write_csv(&r, fmt.tprintf("%s/onfoot-%s.csv", csv_dir, name)) }
        onfoot.run_destroy(&r)
        free_all(context.temp_allocator)
    }
    fmt.printfln("FIXTURE %s", failed == 0 ? "PASS" : "FAIL")
    return failed == 0 ? 0 : 1
}

// --- on foot in the valley (interactive and bench) ---------------------------

onfoot_start :: proc(g: ^Game) {
    build_valley_onfoot(g)
    g.of.active = true
    onfoot.run_init(&g.of.run, &g.of.fx, "valley")
    g.of.run.snap_every_s = 3
    g.of.run.zoom_every_s = 9
    if g.bench_onfoot > 0 {
        route_valley(g, &g.of.ops)
    } else {
        start := TREK[0]
        append(&g.of.ops, onfoot.Op{kind = .Reset, at = {start.x, ve.r3d_hf_height(&g.hf, start.x, start.y), start.y}, yaw = 0, zoom = 14})
        onfoot.run_ops(&g.of.run, g.of.ops[:])
    }
    g.mounted = false
    g.follow = true
    g.use_feet = true
}

// Sync the rider model and the render camera to the controller.
onfoot_pose :: proc(g: ^Game, dt: f32) {
    k := &g.of.run.kcc
    g.pos = {k.pos.x, k.pos.z}
    g.feet_y = k.pos.y
    v := [2]f32{k.vel.x, k.vel.z}
    sp := linalg.length(v)
    if sp > 0.2 {
        want := math.atan2(v.y, v.x)
        d := ve.r3d_wrap_pi(want - g.of.face)
        g.of.face += clamp(d, -12 * dt, 12 * dt)
    }
    g.heading = g.of.face
    g.gait = sp > 0.3 ? 1 : 0
    step_sim(g, dt)
    ve.r3d_follow_apply(&g.of.run.cam, &g.cam)
}

onfoot_interactive :: proc(g: ^Game, dt: f32) {
    in_ := &g.e.input
    mv: [2]f32
    if ve.input_is_key_held(in_, .W) || ve.input_is_key_held(in_, .UP) { mv.y += 1 }
    if ve.input_is_key_held(in_, .S) || ve.input_is_key_held(in_, .DOWN) { mv.y -= 1 }
    if ve.input_is_key_held(in_, .D) || ve.input_is_key_held(in_, .RIGHT) { mv.x += 1 }
    if ve.input_is_key_held(in_, .A) || ve.input_is_key_held(in_, .LEFT) { mv.x -= 1 }
    c := &g.of.run.cam
    if ve.input_is_key_pressed(in_, .Q) { ve.r3d_follow_request_snap(c, -1) }
    if ve.input_is_key_pressed(in_, .E) { ve.r3d_follow_request_snap(c, 1) }
    if ve.input_is_key_held(in_, .Z) { ve.r3d_follow_add_zoom(c, -10 * dt) }
    if ve.input_is_key_held(in_, .X) { ve.r3d_follow_add_zoom(c, 10 * dt) }
    run := ve.input_is_key_held(in_, .LSHIFT)
    op := onfoot.Op{kind = .Hold, seg = "play"}
    g.of.acc += dt
    for g.of.acc >= onfoot.DT {
        g.of.acc -= onfoot.DT
        onfoot.step(&g.of.run, &op, ve.r3d_radial_deadzone(mv, 0.2), run, mv != {})
        if len(g.of.run.rows) > 4096 { clear(&g.of.run.rows) }
    }
    onfoot_pose(g, dt)
}

// --bench-onfoot: one frame per 60 Hz sim step along the valley trek.
@(private="file")
bench_hook :: proc(r: ^onfoot.Run, op: ^onfoot.Op) -> bool {
    g := cast(^Game)r.user
    append(&g.of.step_ms, time.duration_milliseconds(time.tick_since(time.Tick{_nsec = g.of_t0})))
    onfoot_pose(g, onfoot.DT)
    t0 := time.tick_now()
    draw_world(g)
    append(&g.cpu_ms, time.duration_milliseconds(time.tick_since(t0)))
    gl.Finish()
    append(&g.frame_ms, time.duration_milliseconds(time.tick_since(t0)))
    g.frame += 1
    g.of_t0 = time.tick_now()._nsec
    return g.frame < g.bench_onfoot
}

onfoot_bench :: proc(g: ^Game) {
    r := &g.of.run
    r.user = g
    r.on_step = bench_hook
    g.of_t0 = time.tick_now()._nsec
    onfoot.run_ops(r, g.of.ops[:])
    ms := g.frame_ms[min(10, len(g.frame_ms)):]
    sorted := slice.clone(ms, context.temp_allocator)
    slice.sort(sorted)
    sum, csum, ssum: f64
    for v in ms { sum += v }
    for v in g.cpu_ms[min(10, len(g.cpu_ms)):] { csum += v }
    for v in g.of.step_ms { ssum += v }
    n := f64(max(len(ms), 1))
    pct :: proc(s: []f64, p: f64) -> f64 { return s[min(int(p * f64(len(s))), len(s) - 1)] }
    fmt.printfln("BENCH templar_valley_onfoot %dx%d frames=%d avg_ms=%.2f cpu_submit_ms=%.2f sim_step_ms=%.3f p50=%.2f p95=%.2f p99=%.2f max=%.2f fps_avg=%.1f dist_m=%.1f trees=%d buildings=%d",
        g.r.width, g.r.height, len(ms), sum / n, csum / n, ssum / f64(max(len(g.of.step_ms), 1)), pct(sorted, 0.5), pct(sorted, 0.95), pct(sorted, 0.99), sorted[len(sorted) - 1],
        1000.0 * n / sum, r.sum.dist_m, g.stats.trees, g.stats.buildings)
}

// Draw faded props (after the opaque instanced pass), alpha-blended.
draw_faded_props :: proc(g: ^Game) {
    for f in g.of.faded {
        p := &g.props[f.prop]
        a := g.assets[p.asset]
        draw_model_alpha(g.r, &g.pbr, &a.gl, &a.pose, p.mat, f.alpha)
    }
}

draw_model_alpha :: proc(r: ^ve.GL3D_Renderer, pbr: ^ve.GL3D_Pbr, gm: ^ve.GL3D_Model, pose: ^ve.R3D_Model_Pose, m: linalg.Matrix4f32, alpha: f32) {
    if alpha <= 0.01 { return }
    mats := gm.src.materials[:]
    Saved :: struct { mode: ve.R3D_Alpha_Mode, a: f32 }
    saved := make([]Saved, len(mats), context.temp_allocator)
    for &mt, i in mats {
        saved[i] = {mt.alpha_mode, mt.base_color.w}
        if alpha < 0.999 { mt.alpha_mode = .Blend; mt.base_color.w *= alpha }
    }
    ve.gl3d_pbr_draw_model(r, pbr, gm, pose, m)
    for &mt, i in mats { mt.alpha_mode = saved[i].mode; mt.base_color.w = saved[i].a }
}
