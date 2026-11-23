package onfoot

// Routes, driver, telemetry and checks (ports of Godot Templar's
// OnFootRoutes._fixture, OnFootDriver, OnFootTelemetry and the fixture /
// on-foot / occlusion parts of scripts/onfoot_check.py). Fixed 60 Hz sim
// time: the same route gives the same numbers on any machine.

import "core:fmt"
import "core:os"
import "core:math"
import "core:math/linalg"
import "core:slice"
import "core:strings"
import ve "../../../src/core"

DT :: f32(1.0 / 60.0)

Op_Kind :: enum { Reset, Idle, Hold, Push, Go, Snaps, Zoom, Face, Shot }

Op :: struct {
    kind:    Op_Kind,
    seg:     string,
    at:      [3]f32, // reset: world feet
    yaw:     int,    // reset / face: yaw index
    zoom:    f32,
    secs:    f32,
    move:    [2]f32, // hold: camera-relative raw input
    dir:     [2]f32, // push: world xz
    to:      [2]f32, // go: world xz
    tol:     f32,
    run:     bool,
    pad:     bool,   // via pad (analog) vs keyboard (digital)
    blocked: bool,   // intentional block: excluded from stuck detection
    n, sdir: int,
    gap:     f32,
    name:    string, // shot name
    avoid:   bool,   // go: whisker avoidance (Godot OnFootDriver._avoid)
}

Row :: struct {
    t:         f32,
    seg:       string,
    cmd, blocked: bool,
    pos:       [3]f32,
    speed_h:   f32,
    on_floor:  bool,
    air_s:     f32,
    pen:       f32,
    slope_deg: f32,
    yaw_deg:   f32,
    snapping:  bool,
    pending:   int,
    req_zoom, eff, safe: f32,
    cam:       [3]f32,
    cam_clear: f32,
    near_hit:  bool,
    readable, terrain_blocked: bool,
    interior:  bool,
    faded, cut: int,
    snap_index: int,
    valid:     bool,
}

Summary :: struct {
    duration_s, dist_m:  f32,
    pen_max_m:           f32,
    falls, airborne_events, stuck_events: int,
    air_max_s:           f32,
    cam_clear_min_m:     f32,
    clear_violations, near_hits, invalid_transforms: int,
    snaps:               int,
    snap_settle_max_ms:  f32,
    pending_max:         int,
    req_zoom_min, req_zoom_max: f32,
    eff_min_m:           f32,
    go_timeouts:         int,
    faded_max, cut_max:  int,
    interior_entries, interior_exits: int,
    interior_yaws:       [8]bool,
    wall_hits, steps_up: int,
    sim_ms:              f64, // wall-clock cost of the whole route
}

Check :: struct {
    name:      string,
    ok:        bool,
    value:     string,
    threshold: string,
}

Run :: struct {
    route:   string,
    fx:      ^Fixture,
    kcc:     ve.R3D_Kcc,
    cam:     ve.R3D_Follow_Camera,
    rows:    [dynamic]Row,
    sum:     Summary,
    checks:  [dynamic]Check,
    // driver
    t:       f32,
    fall_floor_y: f32,
    // telemetry state
    last_pos: [3]f32, have_last: bool,
    anchor: [3]f32, anchor_t: f32, have_anchor: bool,
    snap_start_t: f32, last_snaps: int,
    // background camera input while going (Godot trek settings)
    snap_every_s, zoom_every_s: f32,
    bg_snap_t, bg_zoom_t: f32,
    bg_zoom_i: int,
    avoid_side: f32,
    // optional per-step hook (rendering, shots); return false to stop
    on_step: proc(r: ^Run, op: ^Op) -> bool,
    user:    rawptr,
}

// ---------------------------------------------------------------------------
// Routes
// ---------------------------------------------------------------------------

reset_op :: proc(local: [3]f32, yaw: int, zoom: f32, seg := "reset") -> Op {
    return {kind = .Reset, at = world(local), yaw = yaw, zoom = zoom, seg = seg}
}
idle :: proc(secs: f32, seg := "idle") -> Op { return {kind = .Idle, secs = secs, seg = seg} }
go_local :: proc(lx, lz: f32, run: bool, seg: string, pad := true, tol: f32 = 0.6) -> Op {
    w := world({lx, 0, lz})
    return {kind = .Go, to = {w.x, w.z}, run = run, pad = pad, tol = tol, seg = seg}
}
snaps :: proc(n, dir: int, gap: f32, seg: string) -> Op { return {kind = .Snaps, n = n, sdir = dir, gap = gap, seg = seg} }
face :: proc(yaw: int, seg := "face") -> Op { return {kind = .Face, yaw = yaw, seg = seg} }
zoom :: proc(z: f32, seg := "zoom") -> Op { return {kind = .Zoom, zoom = z, seg = seg} }
push :: proc(dir: [2]f32, secs: f32, run: bool, blocked: bool, seg: string) -> Op {
    return {kind = .Push, dir = dir, secs = secs, run = run, blocked = blocked, seg = seg}
}
shot :: proc(name: string) -> Op { return {kind = .Shot, name = name, seg = "shot"} }

// OnFootRoutes._fixture (Godot), same order, timings and seg names.
route_fixture :: proc(allocator := context.allocator) -> [dynamic]Op {
    ops := make([dynamic]Op, allocator)
    Test :: struct { name: string, move: [2]f32, run: bool, pad: bool }
    tests := [?]Test{
        {"walk_kb", {1, 0}, false, false}, {"run_kb", {1, 0}, true, false},
        {"walk_pad", {1, 0}, false, true}, {"run_pad", {1, 0}, true, true},
        {"half_pad", {0.5, 0}, false, true}, {"half_run_pad", {0.5, 0}, true, true},
        {"diag_kb", {1, 1}, true, false}, {"diag_pad", {0.70710678, 0.70710678}, true, true},
    }
    for t in tests {
        secs: f32 = strings.has_prefix(t.name, "diag") ? 4 : 6
        append(&ops, reset_op(LANE_START, 0, 14))
        append(&ops, idle(0.5, "settle"))
        append(&ops, Op{kind = .Hold, move = t.move, run = t.run, pad = t.pad, secs = secs, seg = strings.concatenate({"move_", t.name}, allocator)})
        append(&ops, idle(1.5, strings.concatenate({"stop_", t.name}, allocator)))
    }
    append(&ops, reset_op(LANE_START, 0, 14))
    append(&ops, Op{kind = .Hold, move = {1, 0}, run = true, secs = 3, seg = "reversal_out"})
    append(&ops, Op{kind = .Hold, move = {-1, 0}, run = true, secs = 3, seg = "reversal_back"})
    append(&ops, idle(1, "stop_reversal"))
    // course at max zoom: ramp, plateau, ridge, seam, bridge, overhang
    append(&ops, reset_op(COURSE_START, 0, 20))
    Leg :: struct { x: f32, run: bool, name: string }
    legs := [?]Leg{{-42, false, "flat"}, {-30, true, "ramp_up"}, {-24, true, "plateau"}, {-17, true, "ramp_down"},
        {-7.5, false, "ridge_up"}, {3, false, "ridge_down_seam"}, {11, false, "bridge"}, {20, false, "bridge_off"},
        {30, false, "overhang"}, {42, false, "overhang_out"}}
    for l in legs { append(&ops, go_local(l.x, COURSE_Z, l.run, strings.concatenate({"course_", l.name}, allocator))) }
    // boom obstruction by the 10 m end wall: yaw 90 puts the boom over +x
    append(&ops, snaps(2, 1, 0.4, "turn_to_wall"))
    append(&ops, push({1, 0}, 3, false, true, "wall_push"))
    append(&ops, shot("3d3-boom-wall"))
    append(&ops, push({-1, 0}, 4, false, false, "wall_recover"))
    append(&ops, go_local(40, COURSE_Z + 6, false, "to_slope", true, 0.5))
    append(&ops, push({0, 1}, 3, true, true, "slope_push"))
    append(&ops, shot("3d3-capsule-slope"))
    append(&ops, snaps(2, -1, 0.4, "turn_back"))
    append(&ops, zoom(8))
    append(&ops, go_local(30, COURSE_Z, false, "overhang_min_zoom"))
    append(&ops, zoom(20))
    append(&ops, idle(1.5, "overhang_max_zoom"))
    append(&ops, go_local(20, COURSE_Z, false, "overhang_recover", false))
    append(&ops, snaps(8, 1, 0.5, "snap_tour"))
    append(&ops, idle(1, "end"))
    return ops
}

// F2 follow-up: the camera over a 25 deg heightfield hillside at all eight
// yaws, walking up, across and down it, then the 50 deg bank (non-walkable
// terrain) pushed into from below.
route_hill :: proc(allocator := context.allocator) -> [dynamic]Op {
    ops := make([dynamic]Op, allocator)
    base := HF_BASE - ORIGIN.y
    mid := HILL_Z0 + 20
    hill_y := base + 20 * math.tan(math.to_radians(HILL_DEG))
    for zm in ([2]f32{14, 20}) {
        append(&ops, reset_op({0, hill_y, mid}, 0, zm, "hill_reset"))
        append(&ops, idle(0.5, "hill_settle"))
        append(&ops, snaps(8, 1, 0.5, "hill_yaw_tour"))
        append(&ops, snaps(8, -1, 0.5, "hill_yaw_tour_back"))
    }
    append(&ops, reset_op({-20, base, HILL_Z0 - 6}, 0, 20, "hill_reset"))
    append(&ops, go_local(-20, HILL_Z0 + 40, true, "hill_up", true, 0.8))
    append(&ops, shot("3d3-hill-25deg"))
    for i in 0 ..< 8 {
        append(&ops, snaps(1, 1, 0.3, "hill_across_yaw"))
        append(&ops, go_local(i % 2 == 0 ? 20 : -20, HILL_Z0 + 40, i % 2 == 0, "hill_across", i % 2 == 0, 0.8))
    }
    append(&ops, face(0))
    append(&ops, go_local(-20, HILL_Z0 - 4, true, "hill_down", false, 0.8))
    // 50 deg bank: rises toward -z from local z -140
    bank_start := BANK_Z0 + 6
    append(&ops, reset_op({0, base, bank_start}, 0, 14, "bank_reset"))
    append(&ops, push({0, -1}, 3, true, true, "bank_push"))
    append(&ops, snaps(4, 1, 0.4, "bank_yaws"))
    append(&ops, push({0, -1}, 2, false, true, "bank_push_walk"))
    append(&ops, idle(1, "end"))
    return ops
}

// Occlusion + interior (Godot _occlusion / _interior, on the fixture village):
// behind the house at four yaws, under the canopy and the awning, an
// eight-yaw tour behind the house at max zoom, then hall entries at yaws
// 0, 4, 2, 6 and 1 with zooms 14, 20, 8, 14, 14.
route_occlusion :: proc(allocator := context.allocator) -> [dynamic]Op {
    ops := make([dynamic]Op, allocator)
    g :: proc(lx, lz: f32, seg: string, run := false, pad := false, tol: f32 = 0.6) -> Op { return go_local(lx, lz, run, seg, pad, tol) }
    append(&ops, reset_op({-16, 0, 66}, 0, 14), idle(1, "start"))
    // yaw 0 (camera south, +z): north of the house
    append(&ops, g(-8, 66, "to_house_e"), g(-8, 47, "to_north"), g(-16, 47, "behind_house_y0"), idle(1, "behind_house_y0_idle"))
    append(&ops, shot("3d3-occluder-fade"))
    append(&ops, g(-24, 47, "behind_house_y0", false, true))
    // yaw 4 (camera north): south of the house
    append(&ops, face(4), g(-24, 58.5, "to_south"), g(-8, 58.5, "behind_house_y4", false, true))
    // yaw 2 (camera east): west of the house
    append(&ops, face(2), g(-22.5, 58.5, "to_west"), g(-22.5, 47.5, "behind_house_y2"))
    // yaw 6 (camera west): east of the house
    append(&ops, face(6), g(-9.5, 47.5, "to_east"), g(-9.5, 58.5, "behind_house_y6", false, true))
    // under the tree canopy and the awning at yaw 0 (run) and yaw 1 (walk)
    append(&ops, face(0), g(-6, 48, "to_canopy"), g(6, 48, "under_canopy_y0", true), g(16, 49, "under_awning_y0", true, true))
    append(&ops, face(1), g(4, 49, "under_awning_y1"), g(-4, 48.5, "under_canopy_y1"))
    // eight-yaw tour north of the house at max zoom
    append(&ops, face(0), g(-16, 47, "to_tour"), zoom(20), snaps(8, 1, 0.8, "behind_house_tour"), zoom(14))
    // hall entries: door at x 26 on the south (+z) wall at z 60
    Entry :: struct { yaw: int, zoom: f32, tag: string }
    entries := [?]Entry{{0, 14, "y0"}, {4, 20, "y4"}, {2, 8, "y2"}, {6, 14, "y6"}, {1, 14, "y1"}}
    append(&ops, g(26, 66, "to_hall"))
    for e in entries {
        tag := e.tag
        append(&ops, face(e.yaw), zoom(e.zoom))
        append(&ops, g(26, 62, strings.concatenate({"hall_approach_", tag}, allocator), false, true, 0.5))
        append(&ops, g(26, 59.8, strings.concatenate({"hall_door_", tag}, allocator), false, true, 0.4))
        append(&ops, g(26, 55, strings.concatenate({"hall_inside_", tag}, allocator), false, true, 0.5))
        append(&ops, idle(1.5, strings.concatenate({"hall_inside_", tag}, allocator)))
        if tag == "y0" {
            append(&ops, shot("3d3-interior-mode"))
            append(&ops, g(23, 54, "hall_inside_y0", false, true, 0.5), snaps(4, 1, 0.8, "hall_inside_snaps"), face(0))
            append(&ops, g(26, 55, "hall_inside_y0", false, true, 0.5))
        }
        append(&ops, g(26, 59.8, strings.concatenate({"hall_exit_", tag}, allocator), false, true, 0.4))
        append(&ops, g(26, 62, strings.concatenate({"hall_exit_", tag}, allocator), false, true, 0.5))
        append(&ops, g(26, 66, strings.concatenate({"hall_out_", tag}, allocator), false, true, 0.6))
        append(&ops, idle(1, strings.concatenate({"hall_out_", tag}, allocator)))
    }
    append(&ops, idle(2, "end"))
    return ops
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

// Camera-relative input for a world xz direction at the displayed yaw.
to_input :: proc(dir: [2]f32, yaw: f32) -> [2]f32 {
    // inverse of r3d_move_direction: world = (c*x + s*z', -s*x + c*z'), z' = -y
    s, c := math.sin(yaw), math.cos(yaw)
    x := c * dir.x - s * dir.y
    zp := s * dir.x + c * dir.y
    return {x, -zp}
}

// Keyboard: eight digital directions.
quantize_kb :: proc(v: [2]f32) -> [2]f32 {
    if linalg.length(v) < 1e-6 { return {} }
    a := math.atan2(v.y, v.x)
    k := math.round(a / (math.PI / 4))
    q := [2]f32{math.cos(k * math.PI / 4), math.sin(k * math.PI / 4)}
    return {math.round(q.x), math.round(q.y)}
}

@(private="file")
raw_input :: proc(raw: [2]f32, pad: bool) -> [2]f32 {
    r := pad ? raw : [2]f32{math.sign(raw.x) * f32(abs(raw.x) > 0.5 ? 1 : 0), math.sign(raw.y) * f32(abs(raw.y) > 0.5 ? 1 : 0)}
    return ve.r3d_radial_deadzone(r, 0.2)
}

run_init :: proc(r: ^Run, fx: ^Fixture, route: string) {
    r.route = route
    r.fx = fx
    r.kcc = ve.r3d_kcc_default()
    r.cam = ve.r3d_follow_default()
    r.sum.cam_clear_min_m = max(f32)
    r.sum.req_zoom_min = max(f32)
    r.sum.req_zoom_max = min(f32)
    r.sum.eff_min_m = max(f32)
    r.snap_start_t = -1
    r.fall_floor_y = route == "fixture" ? ORIGIN.y - 3 : -1e9
}

run_destroy :: proc(r: ^Run) {
    delete(r.rows)
    delete(r.checks)
}

@(private="file")
do_reset :: proc(r: ^Run, op: ^Op) {
    ve.r3d_kcc_place(&r.kcc, &r.fx.world, op.at + {0, 0.6, 0})
    ve.r3d_follow_set_yaw_index(&r.cam, op.yaw)
    ve.r3d_follow_set_zoom(&r.cam, op.zoom)
    ve.r3d_follow_reset(&r.cam, &r.fx.world, r.kcc.pos)
    r.have_anchor = false
    r.have_last = false
}

// One fixed step of the player, camera, occlusion and telemetry.
step :: proc(r: ^Run, op: ^Op, input: [2]f32, running: bool, cmd: bool) {
    k := &r.kcc
    yaw := ve.r3d_follow_displayed_yaw(&r.cam)
    wish := ve.r3d_kcc_wish(k, input, running, yaw)
    ve.r3d_kcc_step(k, &r.fx.world, wish, DT)
    ve.r3d_follow_step(&r.cam, &r.fx.world, k.pos, DT)
    campos := ve.r3d_follow_camera_position(&r.cam)
    r.cam.interior_mode = ve.r3d_occlusion_step(&r.fx.occ, DT, campos, k.pos)
    r.t += DT
    telemetry(r, op, cmd)
}

@(private="file")
telemetry :: proc(r: ^Run, op: ^Op, cmd: bool) {
    k := &r.kcc
    w := &r.fx.world
    c := &r.cam
    o := &r.fx.occ
    p := k.pos
    s := &r.sum
    s.duration_s = r.t
    if r.have_last { s.dist_m += linalg.length([2]f32{p.x - r.last_pos.x, p.z - r.last_pos.z}) }
    r.last_pos = p; r.have_last = true
    ground := ve.r3d_ground_below(w, p + {0, 1, 0}, ve.R3D_COL_PLAYER, 4)
    terrain_y := ve.r3d_hf_height(&r.fx.hf, p.x, p.z)
    pen := max(0, ground - p.y, terrain_y - p.y)
    s.pen_max_m = max(s.pen_max_m, pen)
    if p.y < terrain_y - 1.5 || p.y < r.fall_floor_y { s.falls += 1 }
    if !k.on_floor && k.air_s >= 1 && k.air_s - DT < 1 { s.airborne_events += 1 }
    s.air_max_s = max(s.air_max_s, k.air_s)
    blocked := op.blocked
    if !cmd || blocked || !r.have_anchor || linalg.length(p - r.anchor) >= 0.5 {
        r.anchor = p; r.anchor_t = r.t; r.have_anchor = true
    } else if r.t - r.anchor_t >= 5 {
        s.stuck_events += 1
        r.anchor = p; r.anchor_t = r.t
    }
    campos := ve.r3d_follow_camera_position(c)
    clear := ve.r3d_follow_clearance_at(c, w, campos)
    s.cam_clear_min_m = min(s.cam_clear_min_m, clear)
    if clear < c.min_ground_clearance_m { s.clear_violations += 1 }
    near := ve.r3d_sphere_overlaps(w, campos, 0.2, ve.R3D_COL_CAMERA)
    if near { s.near_hits += 1 }
    valid := !(math.is_nan(p.x + p.y + p.z) || math.is_nan(campos.x + campos.y + campos.z) || math.is_nan(c.effective_distance))
    if !valid { s.invalid_transforms += 1 }
    if c.snaps_started != r.last_snaps {
        if r.snap_start_t >= 0 { close_snap(r) }
        r.last_snaps = c.snaps_started
        r.snap_start_t = r.t - DT
        s.snaps += 1
    }
    if r.snap_start_t >= 0 && !ve.r3d_follow_is_snapping(c) { close_snap(r) }
    s.pending_max = max(s.pending_max, c.pending != 0 ? 1 : 0)
    s.req_zoom_min = min(s.req_zoom_min, c.requested_zoom)
    s.req_zoom_max = max(s.req_zoom_max, c.requested_zoom)
    s.eff_min_m = min(s.eff_min_m, c.effective_distance)
    faded, cut := ve.r3d_occlusion_cleared_counts(o)
    s.faded_max = max(s.faded_max, faded)
    s.cut_max = max(s.cut_max, cut)
    interior := ve.r3d_follow_is_interior(c) && c.interior_mode
    if len(r.rows) > 0 && interior && !r.rows[len(r.rows) - 1].interior { s.interior_yaws[ve.r3d_follow_snap_index(c)] = true }
    s.interior_entries = o.interior_entries
    s.interior_exits = o.interior_exits
    s.wall_hits = k.wall_hits
    s.steps_up = k.steps_up
    slope := math.to_degrees(math.acos(math.clamp(k.floor_n.y, -1, 1)))
    append(&r.rows, Row{
        t = r.t, seg = op.seg, cmd = cmd, blocked = blocked, pos = p, speed_h = ve.r3d_kcc_horizontal_speed(k),
        on_floor = k.on_floor, air_s = k.air_s, pen = pen, slope_deg = slope,
        yaw_deg = math.to_degrees(ve.r3d_follow_displayed_yaw(c)), snapping = ve.r3d_follow_is_snapping(c), pending = c.pending != 0 ? 1 : 0,
        req_zoom = c.requested_zoom, eff = c.effective_distance, safe = c.safe_distance, cam = campos, cam_clear = clear,
        near_hit = near, readable = o.readable, terrain_blocked = o.terrain_blocked, interior = interior,
        faded = faded, cut = cut, snap_index = ve.r3d_follow_snap_index(c), valid = valid,
    })
}

@(private="file")
close_snap :: proc(r: ^Run) {
    ms := (r.t - r.snap_start_t) * 1000
    r.sum.snap_settle_max_ms = max(r.sum.snap_settle_max_ms, ms)
    r.snap_start_t = -1
}

// Runs every op; returns false if a hook stopped it.
run_ops :: proc(r: ^Run, ops: []Op) -> bool {
    for &op in ops {
        switch op.kind {
        case .Reset:
            do_reset(r, &op)
        case .Shot:
            if r.on_step != nil && !r.on_step(r, &op) { return false }
        case .Zoom:
            ve.r3d_follow_set_zoom(&r.cam, op.zoom)
        case .Idle:
            n := int(math.round(op.secs / DT))
            for _ in 0 ..< n { step(r, &op, {}, false, false); if !hook(r, &op) { return false } }
        case .Hold:
            n := int(math.round(op.secs / DT))
            in_ := raw_input(op.move, op.pad)
            for _ in 0 ..< n { step(r, &op, in_, op.run, true); if !hook(r, &op) { return false } }
        case .Push:
            n := int(math.round(op.secs / DT))
            for _ in 0 ..< n {
                v := to_input(op.dir, ve.r3d_follow_displayed_yaw(&r.cam))
                step(r, &op, raw_input(op.pad ? v : quantize_kb(v), op.pad), op.run, true)
                if !hook(r, &op) { return false }
            }
        case .Go:
            limit := int((linalg.length(op.to - [2]f32{r.kcc.pos.x, r.kcc.pos.z}) / (op.run ? 4.5 : 1.6) * 2 + 6) / DT)
            reached := false
            for _ in 0 ..< limit {
                d := op.to - [2]f32{r.kcc.pos.x, r.kcc.pos.z}
                if linalg.length(d) <= op.tol { reached = true; break }
                dir := linalg.normalize(d)
                if op.avoid { dir = avoid_dir(r, dir) }
                background(r)
                v := to_input(dir, ve.r3d_follow_displayed_yaw(&r.cam))
                step(r, &op, raw_input(op.pad ? v : quantize_kb(v), op.pad), op.run, true)
                if !hook(r, &op) { return false }
            }
            if !reached { r.sum.go_timeouts += 1; fmt.eprintfln("onfoot: go timeout in %s at %v", op.seg, r.kcc.pos) }
        case .Snaps:
            gap := int(math.round(op.gap / DT))
            for _ in 0 ..< op.n {
                ve.r3d_follow_request_snap(&r.cam, op.sdir)
                for _ in 0 ..< max(gap, 1) { step(r, &op, {}, false, false); if !hook(r, &op) { return false } }
            }
        case .Face:
            for guard := 0; ve.r3d_follow_snap_index(&r.cam) != op.yaw && guard < 8; guard += 1 {
                d := (op.yaw - ve.r3d_follow_snap_index(&r.cam) + 8) % 8
                ve.r3d_follow_request_snap(&r.cam, d <= 4 ? 1 : -1)
                for ve.r3d_follow_is_snapping(&r.cam) { step(r, &op, {}, false, false); if !hook(r, &op) { return false } }
            }
        }
    }
    return true
}

// Whisker avoidance: first clear heading among goal +-30/60/90/120 deg (a
// capsule swept 1.6 m from shin height; terrain excluded).
avoid_dir :: proc(r: ^Run, dir: [2]f32, reach: f32 = 1.6, radius: f32 = 0.38) -> [2]f32 {
    w := &r.fx.world
    saved := w.hf
    w.hf = nil
    defer w.hf = saved
    if r.avoid_side == 0 { r.avoid_side = 1 }
    p := r.kcc.pos + {0, 0.45 + radius, 0}
    q := p + {0, 0.8, 0}
    for k in ([]int{0, 1, -1, 2, -2, 3, -3, 4, -4}) {
        a := math.to_radians(f32(30 * k)) * (k != 0 ? r.avoid_side : 1)
        c, s := math.cos(a), math.sin(a)
        d := [2]f32{c * dir.x - s * dir.y, s * dir.x + c * dir.y}
        h := ve.r3d_sweep_capsule(w, p, q, radius, {d.x * reach, 0, d.y * reach}, ve.R3D_COL_PLAYER, 0)
        if !h.hit {
            if k != 0 { r.avoid_side = math.sign(a) }
            return d
        }
    }
    return dir
}

@(private="file")
background :: proc(r: ^Run) {
    if r.snap_every_s > 0 {
        r.bg_snap_t += DT
        if r.bg_snap_t >= r.snap_every_s { r.bg_snap_t = 0; ve.r3d_follow_request_snap(&r.cam, 1) }
    }
    if r.zoom_every_s > 0 {
        r.bg_zoom_t += DT
        if r.bg_zoom_t >= r.zoom_every_s {
            r.bg_zoom_t = 0
            r.bg_zoom_i += 1
            zooms := [3]f32{8, 14, 20}
            ve.r3d_follow_set_zoom(&r.cam, zooms[r.bg_zoom_i % 3])
        }
    }
}

@(private="file")
hook :: proc(r: ^Run, op: ^Op) -> bool { return r.on_step == nil || r.on_step(r, op) }

// ---------------------------------------------------------------------------
// Checks (scripts/onfoot_check.py). Check strings live in the temp allocator.
// ---------------------------------------------------------------------------

add_check :: proc(r: ^Run, name: string, ok: bool, value: string, threshold: string) {
    append(&r.checks, Check{name = name, ok = ok, value = value, threshold = threshold})
}

seg_runs :: proc(rows: []Row, name: string, allocator := context.temp_allocator) -> [dynamic][]Row {
    out := make([dynamic][]Row, allocator)
    i := 0
    for i < len(rows) {
        if rows[i].seg != name { i += 1; continue }
        j := i
        for j < len(rows) && rows[j].seg == name { j += 1 }
        append(&out, rows[i:j])
        i = j
    }
    return out
}

first_run :: proc(rows: []Row, name: string) -> []Row {
    runs := seg_runs(rows, name)
    return len(runs) > 0 ? runs[0] : nil
}

settled_speed :: proc(run: []Row, skip_s: f32 = 1) -> f32 {
    if len(run) == 0 { return 0 }
    vals := make([dynamic]f32, context.temp_allocator)
    for r in run { if r.t - run[0].t >= skip_s { append(&vals, r.speed_h) } }
    if len(vals) == 0 { return 0 }
    slice.sort(vals[:])
    n := len(vals)
    return n % 2 == 1 ? vals[n / 2] : (vals[n / 2 - 1] + vals[n / 2]) * 0.5
}

stop_time :: proc(run: []Row) -> f32 {
    if len(run) == 0 { return 99 }
    last: f32 = -1
    for r in run { if r.speed_h >= 0.05 { last = r.t } }
    if last < 0 { return 0 }
    return last - run[0].t + (len(run) > 1 ? run[1].t - run[0].t : 0)
}

trajectory_gap :: proc(a, b: []Row) -> f32 {
    n := min(len(a), len(b))
    if n == 0 { return max(f32) }
    worst: f32 = 0
    for i in 0 ..< n {
        da := a[i].pos - a[0].pos
        db := b[i].pos - b[0].pos
        worst = max(worst, linalg.length([2]f32{da.x - db.x, da.z - db.z}))
    }
    return worst
}

@(private="file")
f3 :: proc(v: f32) -> string { return fmt.tprintf("%.3f", v) }

judge_common :: proc(r: ^Run) {
    s := &r.sum
    add_check(r, "ground_penetration_m", s.pen_max_m <= 0.1, f3(s.pen_max_m), fmt.tprint("<= 0.1"))
    add_check(r, "falls", s.falls == 0, fmt.tprint(s.falls), fmt.tprint("== 0"))
    add_check(r, "airborne_over_1s", s.airborne_events == 0, fmt.tprintf("%d (max air %.2f s)", s.airborne_events, s.air_max_s), fmt.tprint("== 0"))
    add_check(r, "stuck_events", s.stuck_events == 0, fmt.tprint(s.stuck_events), fmt.tprint("== 0"))
    add_check(r, "go_timeouts", s.go_timeouts == 0, fmt.tprint(s.go_timeouts), fmt.tprint("== 0"))
    add_check(r, "clearance_violations", s.clear_violations == 0, fmt.tprintf("%d (min %.3f m)", s.clear_violations, s.cam_clear_min_m), fmt.tprint("== 0 (< 0.5 m)"))
    add_check(r, "near_plane_hits", s.near_hits == 0, fmt.tprint(s.near_hits), fmt.tprint("== 0"))
    add_check(r, "invalid_transforms", s.invalid_transforms == 0, fmt.tprint(s.invalid_transforms), fmt.tprint("== 0"))
    add_check(r, "snap_settle_max_ms", s.snaps == 0 || s.snap_settle_max_ms <= 250, fmt.tprintf("%.1f (%d snaps)", s.snap_settle_max_ms, s.snaps), fmt.tprint("<= 250"))
    add_check(r, "pending_snaps_max", s.pending_max <= 1, fmt.tprint(s.pending_max), fmt.tprint("<= 1"))
}

judge_fixture :: proc(r: ^Run) {
    rows := r.rows[:]
    WALK, RUN :: f32(1.6), f32(4.5)
    Sp :: struct { name: string, target: f32 }
    for t in ([]Sp{{"walk_kb", WALK}, {"run_kb", RUN}, {"walk_pad", WALK}, {"run_pad", RUN}, {"diag_kb", RUN}, {"diag_pad", RUN}}) {
        v := settled_speed(first_run(rows, fmt.tprint("move_", t.name, sep = "")))
        add_check(r, fmt.tprint("speed_", t.name, sep = ""), abs(v - t.target) <= 0.05 * t.target, f3(v), fmt.tprintf("%.1f +-5%%", t.target))
        st := stop_time(first_run(rows, fmt.tprint("stop_", t.name, sep = "")))
        add_check(r, fmt.tprint("stop_", t.name, sep = ""), st <= 0.3, f3(st), fmt.tprint("<= 0.3 s"))
    }
    for p in ([][2]string{{"walk_kb", "walk_pad"}, {"run_kb", "run_pad"}, {"diag_kb", "diag_pad"}}) {
        gap := trajectory_gap(first_run(rows, fmt.tprint("move_", p[0], sep = "")), first_run(rows, fmt.tprint("move_", p[1], sep = "")))
        add_check(r, fmt.tprint("kb_vs_pad_", strings.split(p[0], "_", context.temp_allocator)[0], "_m", sep = ""), gap <= 0.10, fmt.tprintf("%.4f", gap), fmt.tprint("<= 0.10"))
    }
    half := settled_speed(first_run(rows, "move_half_pad"))
    half_run := settled_speed(first_run(rows, "move_half_run_pad"))
    add_check(r, "half_stick_slower", 0.05 < half && half < 0.9 * WALK && 0.05 < half_run && half_run < 0.9 * RUN,
        fmt.tprintf("[%.3f, %.3f]", half, half_run), fmt.tprint("0 < half < 0.9 x full (walk, run)"))
    slope := first_run(rows, "slope_push")
    rise: f32 = 99
    if len(slope) > 0 { hi := slope[0].pos.y; for x in slope { hi = max(hi, x.pos.y) }; rise = hi - slope[0].pos.y }
    add_check(r, "nonwalkable_slope_rise_m", rise <= 0.5, f3(rise), fmt.tprint("<= 0.5"))
    wall := first_run(rows, "wall_push")
    wall_x: f32 = 99e3
    retract: f32 = 99
    if len(wall) > 0 { wall_x = min(f32); for x in wall { wall_x = max(wall_x, x.pos.x); retract = min(retract, x.eff) } }
    add_check(r, "wall_blocks", wall_x <= ORIGIN.x + WALL_X - 0.35, f3(wall_x), fmt.tprintf("<= %.2f", ORIGIN.x + WALL_X - 0.35))
    add_check(r, "wall_boom_retracted_below_8m", retract < 8, f3(retract), fmt.tprint("< 8 (below the requested minimum)"))
    over := first_run(rows, "overhang_max_zoom")
    over_eff: f32 = 99
    for x in over { over_eff = min(over_eff, x.eff) }
    add_check(r, "overhang_boom_retracted", over_eff < 19, f3(over_eff), fmt.tprint("< requested 20"))
    tour := first_run(rows, "snap_tour")
    gap: f32 = 99
    if len(tour) > 0 { e := tour[len(tour) - 1]; gap = abs(e.eff - e.req_zoom) }
    add_check(r, "boom_recovered_after_overhang", gap < 0.05, f3(gap), fmt.tprint("< 0.05 m from requested"))
    for name in ([]string{"course_ramp_up", "course_ridge_up", "course_bridge", "course_ridge_down_seam"}) {
        ran := len(first_run(rows, name)) > 0
        add_check(r, fmt.tprint("traversed_", name, sep = ""), ran, fmt.tprint(ran), fmt.tprint("segment ran"))
    }
}

judge_hill :: proc(r: ^Run) {
    rows := r.rows[:]
    slope_max: f32 = 0
    clear_min: f32 = max(f32)
    yaws: [8]bool
    for x in rows {
        if strings.has_prefix(x.seg, "hill_") && x.on_floor {
            slope_max = max(slope_max, x.slope_deg)
            if x.slope_deg > 20 { clear_min = min(clear_min, x.cam_clear); if !x.snapping { yaws[x.snap_index] = true } }
        }
    }
    ny := 0
    for y in yaws { if y { ny += 1 } }
    add_check(r, "hill_slope_reached_deg", slope_max >= HILL_DEG - 1, fmt.tprintf("%.2f", slope_max), fmt.tprintf(">= %.0f", HILL_DEG - 1))
    add_check(r, "hill_cam_clearance_min_m", clear_min >= 0.5, f3(clear_min), fmt.tprint(">= 0.5 on > 20 deg ground (F2)"))
    add_check(r, "hill_yaws_on_slope", ny == 8, fmt.tprint(ny), fmt.tprint("all 8"))
    bank := first_run(rows, "bank_push")
    rise: f32 = 99
    if len(bank) > 0 { hi := bank[0].pos.y; for x in bank { hi = max(hi, x.pos.y) }; rise = hi - bank[0].pos.y }
    for x in first_run(rows, "bank_push_walk") { rise = max(rise, x.pos.y - bank[0].pos.y) }
    add_check(r, "hf_nonwalkable_rise_m", rise <= 0.5, f3(rise), fmt.tprint("<= 0.5 (50 deg heightfield bank)"))
}

judge_occlusion :: proc(r: ^Run) {
    rows := r.rows[:]
    judge_readable(r)
    s := &r.sum
    add_check(r, "occluders_faded", s.faded_max >= 1, fmt.tprint(s.faded_max), fmt.tprint(">= 1"))
    add_check(r, "roofs_cut", s.cut_max >= 1, fmt.tprint(s.cut_max), fmt.tprint(">= 1"))
    ny := 0
    for y in s.interior_yaws { if y { ny += 1 } }
    ends_outside := len(rows) > 0 && !rows[len(rows) - 1].interior
    add_check(r, "interior_mode_entered_exited", s.interior_entries >= 3 && ny >= 3 && s.interior_exits >= 1 && ends_outside,
        fmt.tprintf("%d entries at %d yaws, %d exits, ends outside %v", s.interior_entries, ny, s.interior_exits, ends_outside), fmt.tprint(">= 3 entries at >= 3 yaws, ends outside"))
    add_check(r, "zoom_contract", s.req_zoom_min >= 8 - 1e-6 && s.req_zoom_max <= 20 + 1e-6, fmt.tprintf("[%.1f, %.1f]", s.req_zoom_min, s.req_zoom_max), fmt.tprint("8 <= requested <= 20"))
}

// Player readable (not hidden by a solid occluder or terrain) per step.
judge_readable :: proc(r: ^Run) {
    rows := r.rows[:]
    readable := 0
    longest, cur: f32 = 0, 0
    for x in rows {
        if x.readable && !x.terrain_blocked { readable += 1; cur = 0 } else { cur += DT; longest = max(longest, cur) }
    }
    pct := 100 * f32(readable) / f32(max(len(rows), 1))
    add_check(r, "readable_pct", pct >= 98, fmt.tprintf("%.2f", pct), fmt.tprint(">= 98"))
    add_check(r, "longest_hidden_s", longest <= 0.5, f3(longest), fmt.tprint("<= 0.5"))
}

// Builds the route, runs it and judges it.
run_route :: proc(r: ^Run, fx: ^Fixture, route: string) -> bool {
    run_init(r, fx, route)
    ops: [dynamic]Op
    switch route {
    case "fixture": ops = route_fixture(context.temp_allocator)
    case "hill": ops = route_hill(context.temp_allocator)
    case "occlusion": ops = route_occlusion(context.temp_allocator)
    case: return false
    }
    if !run_ops(r, ops[:]) { return false }
    judge_common(r)
    switch route {
    case "fixture": judge_fixture(r)
    case "hill": judge_hill(r)
    case "occlusion": judge_occlusion(r)
    }
    return true
}

all_ok :: proc(r: ^Run) -> bool {
    for c in r.checks { if !c.ok { return false } }
    return true
}

print_checks :: proc(r: ^Run) {
    fails := 0
    for c in r.checks { if !c.ok { fails += 1 } }
    fmt.printfln("ROUTE %s: %s (%d checks, %d failed) sim %.1f s, %d steps, %.1f m",
        r.route, fails == 0 ? "PASS" : "FAIL", len(r.checks), fails, r.sum.duration_s, len(r.rows), r.sum.dist_m)
    for c in r.checks { fmt.printfln("  %-4s %-34s %-36s %s", c.ok ? "ok" : "FAIL", c.name, c.value, c.threshold) }
}

// Telemetry CSV (column names follow Godot's OnFootTelemetry where they exist).
write_csv :: proc(r: ^Run, path: string) -> bool {
    sb := strings.builder_make()
    defer strings.builder_destroy(&sb)
    strings.write_string(&sb, "t_s,seg,cmd,blocked,x,y,z,speed_h,on_floor,air_s,pen_m,floor_deg,yaw_deg,snapping,pending,req_zoom,eff_m,safe_m,cam_x,cam_y,cam_z,cam_clear_m,near_hit,readable,terrain_blocked,interior,faded,cut,valid\n")
    b :: proc(v: bool) -> int { return v ? 1 : 0 }
    for x in r.rows {
        fmt.sbprintf(&sb, "%.4f,%s,%d,%d,%.3f,%.3f,%.3f,%.3f,%d,%.3f,%.3f,%.2f,%.2f,%d,%d,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d\n",
            x.t, x.seg, b(x.cmd), b(x.blocked), x.pos.x, x.pos.y, x.pos.z, x.speed_h, b(x.on_floor), x.air_s, x.pen, x.slope_deg, x.yaw_deg,
            b(x.snapping), x.pending, x.req_zoom, x.eff, x.safe, x.cam.x, x.cam.y, x.cam.z, x.cam_clear, b(x.near_hit),
            b(x.readable), b(x.terrain_blocked), b(x.interior), x.faded, x.cut, b(x.valid))
    }
    return os.write_entire_file(path, transmute([]u8)strings.to_string(sb)) == nil
}
