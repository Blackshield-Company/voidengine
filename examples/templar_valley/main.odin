package templar_valley

// ============================================================================
// templar_valley — Void Engine 3D-2 demo and the look test: Templar Valley's
// 2 x 2 km terrain rebuilt from the Godot build's own height and ground rules
// (bit-faithful noise), its four CC0 Poly Haven ground layers, the Godot
// prop layout (trees, houses, clutter) dressed with CC0 Quaternius models, a
// rider on a horse, sky, height fog and cascaded sun shadows. OpenGL backend.
//
// Usage:
//   templar_valley [--size WxH] [--ssaa N] [--shots DIR] [--bench N] [--bench-pitch DEG]
//                  [--onfoot] [--bench-onfoot N] [--fixture ROUTE|all [--csv DIR]]
//                  [--fixture-shots DIR]
//   --shots DIR   render the look-test shots (same camera poses as the Godot
//                 review shots) as PPM into DIR, then exit
//   --bench N     ride the east spur road for N frames with the riding camera,
//                 print frame-time stats (glFinish per frame), exit
//   --ssaa N      render N x the size and box-filter the shots down
//   3D-3 on foot (collision, controller, follow camera, occlusion): see
//   onfoot_play.odin and fixture_scene.odin for --onfoot, --bench-onfoot,
//   --fixture and --fixture-shots.
// Controls: mouse drag = orbit, Z/X = zoom, F = free-fly (WASD QE, Shift),
//           1/2/3 = horse halt/walk/trot (rides east), Esc = quit
// Assets: python3 examples/templar_valley/tools/fetch_assets.py then
//         python3 examples/templar_valley/tools/pack_ground.py (see ASSETS.md)
// ============================================================================

import "core:fmt"
import "core:math"
import "core:math/linalg"
import "core:os"
import "core:slice"
import "core:strconv"
import "core:strings"
import "core:time"
import SDL "vendor:sdl2"
import gl "vendor:OpenGL"
import ve "../../src/core"

ASSETS :: "examples/templar_valley/assets"
FETCHED :: ASSETS + "/fetched"
GENERATED :: ASSETS + "/generated"

RIDER_HEIGHT :: f32(1.85)
HORSE_BACK_HEIGHT :: f32(1.55)
TREE_RANGE :: f32(520)
BUILDING_RANGE :: f32(600)
CLUTTER_RANGE :: f32(140)
GRASS_RANGE :: f32(75)
GRASS_CELL :: f32(12)

// Godot FollowCamera (src/camera/follow_camera.gd) riding / on-foot numbers.
RIDING_PIVOT_HEIGHT :: f32(2.6)
ONFOOT_PIVOT_HEIGHT :: f32(1.6)
CAMERA_FOV_DEG :: f32(70)

Look_Shot :: struct {
    name:      string,
    ref:       string, // the Godot shot it matches (progress/runs/m1-terrain/shots-9830014/)
    at:        [2]f32, // rider/player x, z
    heading:   f32,    // radians, 0 = +x (east)
    mounted:   bool,
    yaw_deg:   f32,    // Godot camera yaw (snap index * 45)
    pitch_deg: f32,
    zoom:      f32,
    ahead:     f32,    // riding look-ahead (m, along heading)
    t:         f32,    // animation time
}

// _clip_terrain in src/telemetry/onfoot_routes.gd: on foot on the east ridge
// (yaw index 2, zoom 20); mounted trotting east on the spur road (yaw 0,
// zoom 8, pitch 55); then at the 25 deg riding pitch.
LOOK_SHOTS := [?]Look_Shot{
    {name = "look-mounted-55", ref = "after-mounted.png", at = {120, -562}, heading = 0, mounted = true, yaw_deg = 0, pitch_deg = 55, zoom = 8, ahead = 1.5, t = 2.3},
    {name = "look-mounted-25", ref = "after-mounted-horizon-25deg.png", at = {150, -560}, heading = 0, mounted = true, yaw_deg = 0, pitch_deg = 25, zoom = 8, ahead = 1.5, t = 3.1},
    {name = "look-onfoot", ref = "after-onfoot.png", at = {790, -480}, heading = math.PI, mounted = false, yaw_deg = 90, pitch_deg = 55, zoom = 20, ahead = 0, t = 1.0},
    {name = "rider-close", ref = "", at = {120, -562}, heading = 0, mounted = true, yaw_deg = -60, pitch_deg = 12, zoom = 5, ahead = 0, t = 2.3},
    {name = "meadow-close", ref = "", at = {60, -470}, heading = 0, mounted = false, yaw_deg = 30, pitch_deg = 20, zoom = 6, ahead = 0, t = 1.0},
    {name = "valley-north-low", ref = "", at = {6, -300}, heading = -math.PI / 2, mounted = true, yaw_deg = 180, pitch_deg = 14, zoom = 11, ahead = 1.5, t = 0.7},
}

Asset :: struct {
    name:   string,
    model:  ve.R3D_Model,
    gl:     ve.GL3D_Model,
    pose:   ve.R3D_Model_Pose,
    bmin, bmax: [3]f32,
    inst:   [dynamic]linalg.Matrix4f32,
}

Prop :: struct {
    asset:  int,
    mat:    linalg.Matrix4f32,
    center: [3]f32,
    radius: f32,
    kind:   u8,
}

Game :: struct {
    e:      ^ve.Engine,
    r:      ^ve.GL3D_Renderer,
    pbr:    ve.GL3D_Pbr,
    env:    ve.GL3D_Env,
    valley: Valley,
    hf:     ve.R3D_Heightfield,
    ter:    ve.R3D_Terrain,
    gter:   ve.GL3D_Terrain,
    splat:  []u8,

    assets: [dynamic]^Asset, // heap: GL3D_Model keeps a pointer to its R3D_Model
    props:  [dynamic]Prop,
    cells:  []Prop_Cell,      // PROP_GRID^2 buckets for culling
    shadow_lists: [dynamic][dynamic]linalg.Matrix4f32, // per asset, reused per cascade
    a_pine, a_common, a_house, a_clutter, a_grass: [dynamic]int,

    // grass cells: key -> instances
    grass_cells: map[[2]i32][dynamic]Prop,

    // rider + horse
    king, horse: int, // asset indices
    king_pose, horse_pose: ve.R3D_Model_Pose,
    king_player, horse_player: ve.R3D_Anim_Player,
    king_scale, horse_scale: f32,
    king_hips: [3]f32,
    horse_back_node: int,
    clip_horse_idle, clip_horse_walk, clip_horse_gallop: int,
    clip_king_idle, clip_king_walk: int,
    pos:     [2]f32,
    heading: f32,
    mounted: bool,
    gait:    int, // 0 halt, 1 walk, 2 trot
    anim_t:  f32,

    cam:     ve.R3D_Camera,
    orbit:   ve.R3D_Orbit_Camera,
    fly:     ve.R3D_Fly_Camera,
    use_fly: bool,
    look:    Look_Shot,
    follow:  bool, // Godot follow-camera pose (shots/bench)

    shots_dir: string,
    next_shot: int,
    ssaa:      int,
    out_w, out_h: int,
    bench:     int,
    bench_pitch: f32,
    frame_ms:  [dynamic]f64, // draw_world + glFinish
    cpu_ms:    [dynamic]f64, // draw_world submission only
    frame:     int,
    deterministic: bool,
    last_ns:   i64,
    stats:     Draw_Stats,

    // 3D-3 on foot
    headless:      bool, // CPU only (--fixture): no GL uploads
    of:            Onfoot,
    of_t0:         i64,
    onfoot_mode:   bool,
    bench_onfoot:  int,
    use_feet:      bool, // rider model at feet_y (controller) not the terrain
    feet_y:        f32,
    fixture_shots: string,
    fxs:           Fx_Scene,
    exit_code:     int,
}

PROP_CELL :: f32(64)
PROP_GRID :: 32 // 2048 m / 64 m

Prop_Cell :: struct {
    props:      [dynamic]i32,
    bmin, bmax: [3]f32,
}

build_prop_grid :: proc(g: ^Game) {
    g.cells = make([]Prop_Cell, PROP_GRID * PROP_GRID)
    for &c in g.cells { c.bmin = {max(f32), max(f32), max(f32)}; c.bmax = {min(f32), min(f32), min(f32)} }
    for &p, i in g.props {
        cx := clamp(int((p.center.x + HALF_EXTENT) / PROP_CELL), 0, PROP_GRID - 1)
        cz := clamp(int((p.center.z + HALF_EXTENT) / PROP_CELL), 0, PROP_GRID - 1)
        c := &g.cells[cz * PROP_GRID + cx]
        append(&c.props, i32(i))
        c.bmin = linalg.min(c.bmin, p.center - p.radius)
        c.bmax = linalg.max(c.bmax, p.center + p.radius)
    }
}

aabb_distance :: proc(p, bmin, bmax: [3]f32) -> f32 {
    q := linalg.clamp(p, bmin, bmax)
    return linalg.length(p - q)
}

Draw_Stats :: struct { chunks, trees, buildings, clutter, grass: int }

load_asset :: proc(g: ^Game, name: string, path: string) -> int {
    a := new(Asset)
    a.name = name
    ok: bool
    a.model, ok = ve.r3d_model_load(path)
    if !ok { fmt.eprintln("templar_valley: cannot load", path, "(run tools/fetch_assets.py)"); os.exit(1) }
    if !g.headless { a.gl = ve.gl3d_model_upload(&a.model) }
    ve.r3d_model_pose_init(&a.pose, &a.model)
    a.bmin = {max(f32), max(f32), max(f32)}
    a.bmax = {min(f32), min(f32), min(f32)}
    for &p in a.model.prims {
        m := linalg.MATRIX4F32_IDENTITY
        if p.skin < 0 && p.node >= 0 { m = a.pose.global[p.node] }
        for c in 0 ..< 8 {
            q := [3]f32{c & 1 != 0 ? p.bounds_max.x : p.bounds_min.x, c & 2 != 0 ? p.bounds_max.y : p.bounds_min.y, c & 4 != 0 ? p.bounds_max.z : p.bounds_min.z}
            w := ve.r3d_mat4_transform_point(m, q)
            a.bmin = linalg.min(a.bmin, w)
            a.bmax = linalg.max(a.bmax, w)
        }
    }
    if !g.headless { ve.gl3d_model_enable_instancing(&g.pbr, &a.gl) }
    append(&g.assets, a)
    return len(g.assets) - 1
}

hash32 :: proc(a: u32) -> u32 {
    x := a
    x ~= x >> 16; x *= 0x7feb352d
    x ~= x >> 15; x *= 0x846ca68b
    x ~= x >> 16
    return x
}
hash01 :: proc(a: u32) -> f32 { return f32(hash32(a) & 0xffffff) / f32(0x1000000) }

yaw_scale_matrix :: proc(pos: [3]f32, yaw: f32, s: [3]f32) -> linalg.Matrix4f32 {
    return ve.r3d_trs_matrix(ve.R3D_Trs{t = pos, r = ve.r3d_quat_axis_angle({0, 1, 0}, yaw), s = s})
}

// Props from the Godot bake (tools/bake_valley.gd): same positions/sizes,
// greybox boxes replaced by CC0 models.
load_props :: proc(g: ^Game) {
    data, err := os.read_entire_file_from_path(ASSETS + "/valley_props.csv", context.allocator)
    if err != nil { fmt.eprintln("templar_valley: no valley_props.csv"); os.exit(1) }
    defer delete(data)
    text := string(data)
    line_no := 0
    for line in strings.split_lines_iterator(&text) {
        line_no += 1
        if line_no == 1 || len(line) < 3 { continue }
        f := strings.split(line, ",", context.temp_allocator)
        if len(f) < 7 { continue }
        v: [6]f32
        for k in 0 ..< 6 { x, _ := strconv.parse_f32(f[k + 1]); v[k] = x }
        pos := [3]f32{v[0], v[1], v[2]}
        size := [3]f32{v[3], v[4], v[5]}
        id := u32(line_no) * 2654435761
        p: Prop
        p.kind = f[0][0]
        switch p.kind {
        case 't':
            forest := is_forest(&g.valley, pos.x, pos.z)
            list := forest || hash01(id) < 0.3 ? g.a_pine[:] : g.a_common[:]
            p.asset = list[int(hash32(id + 1)) % len(list)]
            a := g.assets[p.asset]
            hgt := a.bmax.y - a.bmin.y
            s := size.y / max(hgt, 0.01)
            pos.y = ve.r3d_hf_height(&g.hf, pos.x, pos.z) - 0.1
            p.mat = yaw_scale_matrix(pos, hash01(id + 2) * math.TAU, {s, s * (0.92 + 0.16 * hash01(id + 3)), s})
            p.radius = max(a.bmax.x - a.bmin.x, a.bmax.z - a.bmin.z) * s * 0.5 + hgt * s * 0.5
            p.center = pos + {0, hgt * s * 0.5, 0}
        case 'b':
            p.asset = g.a_house[int(hash32(id + 1)) % len(g.a_house)]
            a := g.assets[p.asset]
            fx, fz := a.bmax.x - a.bmin.x, a.bmax.z - a.bmin.z
            s := math.clamp(math.sqrt((size.x * size.z) / max(fx * fz, 0.01)), 0.5, 2.0)
            yaw := f32(int(hash32(id + 2)) % 4) * math.PI / 2
            // sit on the lowest corner so no side floats on a slope
            lo := max(f32)
            for c in 0 ..< 4 {
                cx := pos.x + (c & 1 != 0 ? 1 : -1) * fx * s * 0.5
                cz := pos.z + (c & 2 != 0 ? 1 : -1) * fz * s * 0.5
                lo = min(lo, ve.r3d_hf_height(&g.hf, cx, cz))
            }
            pos.y = lo - a.bmin.y * s - 0.05
            p.mat = yaw_scale_matrix(pos, yaw, {s, s, s})
            p.radius = linalg.length(a.bmax - a.bmin) * s * 0.5
            p.center = pos + {0, (a.bmax.y - a.bmin.y) * s * 0.5, 0}
        case 'c':
            p.asset = g.a_clutter[int(hash32(id + 1)) % len(g.a_clutter)]
            a := g.assets[p.asset]
            ext := max(a.bmax.x - a.bmin.x, a.bmax.y - a.bmin.y, a.bmax.z - a.bmin.z)
            s := 1.0 / max(ext, 0.01)
            pos.y = ve.r3d_hf_height(&g.hf, pos.x, pos.z) - a.bmin.y * s - 0.03
            p.mat = yaw_scale_matrix(pos, hash01(id + 2) * math.TAU, {s, s, s})
            p.radius = 0.9
            p.center = pos + {0, 0.5, 0}
        case:
            continue
        }
        append(&g.props, p)
    }
    fmt.printfln("templar_valley: %d props from the Godot bake", len(g.props))
}

splat_at :: proc(g: ^Game, x, z: f32) -> [4]f32 {
    i := clamp(int(x + HALF_EXTENT), 0, SPLAT_RES - 1)
    j := clamp(int(z + HALF_EXTENT), 0, SPLAT_RES - 1)
    o := (j * SPLAT_RES + i) * 4
    return {f32(g.splat[o]), f32(g.splat[o + 1]), f32(g.splat[o + 2]), f32(g.splat[o + 3])} / 255
}

// Grass and small bushes for one cell, deterministic.
grass_cell :: proc(g: ^Game, key: [2]i32) -> [dynamic]Prop {
    out: [dynamic]Prop
    step: f32 = 0.85
    n := int(GRASS_CELL / step)
    seed := u32(key.x) * 73856093 ~ u32(key.y) * 19349663
    for j in 0 ..< n {
        for i in 0 ..< n {
            id := seed + u32(j * n + i) * 2246822519
            x := f32(key.x) * GRASS_CELL + (f32(i) + hash01(id)) * step
            z := f32(key.y) * GRASS_CELL + (f32(j) + hash01(id + 1)) * step
            w := splat_at(g, x, z)
            green := w[GRASS] + w[FOREST]
            if green < 0.7 || w[DIRT] > 0.12 || w[ROCK] > 0.2 { continue }
            // clumps: low-frequency density so the meadow is patchy
            dens := 0.35 + 0.65 * math.clamp(ve.fnl_noise_2d(&g.valley.patch_noise, x * 7, z * 7) * 1.4 + 0.5, 0, 1)
            if hash01(id + 2) > dens * green { continue }
            p: Prop
            p.asset = g.a_grass[int(hash32(id + 4)) % len(g.a_grass)]
            a := g.assets[p.asset]
            target := 0.35 + 0.35 * hash01(id + 5)
            s := target / max(a.bmax.y - a.bmin.y, 0.01)
            y := ve.r3d_hf_height(&g.hf, x, z) - 0.02
            p.mat = yaw_scale_matrix({x, y, z}, hash01(id + 6) * math.TAU, {s, s, s})
            p.center = {x, y + target * 0.5, z}
            p.radius = max(a.bmax.x - a.bmin.x, a.bmax.z - a.bmin.z) * s * 0.5 + target * 0.5
            p.kind = 'g'
            append(&out, p)
        }
    }
    return out
}

update_grass :: proc(g: ^Game, focus: [3]f32) {
    r := GRASS_RANGE
    c0 := [2]i32{i32(math.floor((focus.x - r) / GRASS_CELL)), i32(math.floor((focus.z - r) / GRASS_CELL))}
    c1 := [2]i32{i32(math.floor((focus.x + r) / GRASS_CELL)), i32(math.floor((focus.z + r) / GRASS_CELL))}
    for cz in c0.y ..= c1.y {
        for cx in c0.x ..= c1.x {
            k := [2]i32{cx, cz}
            if k in g.grass_cells { continue }
            g.grass_cells[k] = grass_cell(g, k)
        }
    }
    // drop far cells
    drop: [dynamic][2]i32
    defer delete(drop)
    for k in g.grass_cells {
        if k.x < c0.x - 2 || k.x > c1.x + 2 || k.y < c0.y - 2 || k.y > c1.y + 2 { append(&drop, k) }
    }
    for k in drop {
        v := g.grass_cells[k]
        delete(v)
        delete_key(&g.grass_cells, k)
    }
}

game_init :: proc(e: ^ve.Engine) {
    g := cast(^Game)e.user_data
    g.r = ve.gl3d_init(e.config.width, e.config.height)
    if g.r == nil { fmt.eprintln("templar_valley: gl3d_init failed"); os.exit(1) }
    fmt.printfln("templar_valley: GL renderer %s | %s", gl.GetString(gl.RENDERER), gl.GetString(gl.VERSION))
    ok: bool
    g.pbr, ok = ve.gl3d_pbr_init()
    if !ok { os.exit(1) }
    if !ve.gl3d_env_init(&g.env, 2048) { os.exit(1) }
    ve.gl3d_pbr_use_env(&g.pbr, &g.env)
    if len(g.fixture_shots) > 0 {
        setup_lighting(g)
        g.cam = ve.R3D_Camera{fov_y = math.to_radians(CAMERA_FOV_DEG), near_z = 0.2, far_z = 3000}
        fx_scene_init(g)
        return
    }

    t0 := time.tick_now()
    valley_init(&g.valley)
    g.hf = build_heightfield(&g.valley)
    if !ve.r3d_terrain_init(&g.ter, &g.hf, 64, 80) { fmt.eprintln("terrain init failed"); os.exit(1) }
    if !ve.gl3d_terrain_init(&g.gter, &g.ter) { os.exit(1) }
    g.splat = build_splat(&g.valley, &g.hf)
    ve.gl3d_terrain_set_splat(&g.gter, g.splat, SPLAT_RES, SPLAT_RES, {-HALF_EXTENT, -HALF_EXTENT}, {2 * HALF_EXTENT, 2 * HALF_EXTENT})
    fmt.printfln("templar_valley: terrain %dx%d @ %.0f m, %d chunks, %d LODs, splat %d^2 built in %.0f ms",
        HF_SIZE, HF_SIZE, HF_SPACING, len(g.ter.chunks), g.ter.lod_count, SPLAT_RES, time.duration_milliseconds(time.tick_since(t0)))

    // ground layers (Templar's slots: grass, rock, forest floor, dirt)
    {
        LW :: 1024
        albedo := make([]u8, LW * LW * 4 * 4); defer delete(albedo)
        normal := make([]u8, LW * LW * 4 * 4); defer delete(normal)
        orm := make([]u8, LW * LW * 4 * 4); defer delete(orm)
        for i in 0 ..< 4 {
            for kind, k in ([]string{"albedo", "normal", "orm"}) {
                path := fmt.tprintf("%s/layer%d_%s.png", GENERATED, i, kind)
                bytes, rerr := os.read_entire_file_from_path(path, context.allocator)
                if rerr != nil { fmt.eprintln("templar_valley: missing", path, "(run tools/pack_ground.py)"); os.exit(1) }
                px, w, h := ve.r3d_decode_rgba8(bytes)
                delete(bytes)
                if w != LW || h != LW { fmt.eprintln("bad layer size", path); os.exit(1) }
                dst := k == 0 ? albedo : (k == 1 ? normal : orm)
                copy(dst[i * LW * LW * 4:], px)
                delete(px)
            }
        }
        ve.gl3d_terrain_set_layers(&g.gter, albedo, normal, orm, LW, LW, 4)
        // Terrain3D samples at world_xz * uv_scale * 0.5 (its shader halves
        // the asset scale), so the effective repeat per metre is half.
        g.gter.uv_scale = UV_SCALES * 0.5
        g.gter.layer_tint = {{1.0, 1.02, 0.95, 1}, {0.95, 0.95, 0.95, 1}, {1.0, 1.0, 1.0, 1}, {1.0, 0.97, 0.93, 1}}
        g.gter.macro = 0.55
        g.gter.normal_strength = 1.35
    }

    // models
    g.king = load_asset(g, "king", FETCHED + "/king.glb")
    g.horse = load_asset(g, "horse", FETCHED + "/horse.glb")
    for n in ([]string{"pine_1", "pine_2"}) { append(&g.a_pine, load_asset(g, n, fmt.tprintf("%s/%s.glb", FETCHED, n))) }
    for n in ([]string{"common_tree_1", "common_tree_2"}) { append(&g.a_common, load_asset(g, n, fmt.tprintf("%s/%s.glb", FETCHED, n))) }
    for n in ([]string{"house_1", "house_2", "house_3"}) { append(&g.a_house, load_asset(g, n, fmt.tprintf("%s/%s.glb", FETCHED, n))) }
    for n in ([]string{"crate", "barrel", "hay", "rock_medium_1"}) { append(&g.a_clutter, load_asset(g, n, fmt.tprintf("%s/%s.glb", FETCHED, n))) }
    for n in ([]string{"grass_common_short", "grass_wispy_short"}) { append(&g.a_grass, load_asset(g, n, fmt.tprintf("%s/%s.glb", FETCHED, n))) }
    // The kit's grass atlas is bright stylised yellow-green; pull it toward
    // the Poly Haven meadow so the blades sit in the photo ground.
    for ai in g.a_grass { for &mt in g.assets[ai].model.materials { mt.base_color = {0.62, 0.74, 0.46, 1} } }
    for a in g.assets { fmt.printfln("  asset %-20s bounds %v .. %v", a.name, a.bmin, a.bmax) }
    load_props(g)
    build_prop_grid(g)

    // rider (Quaternius King) and horse
    king := g.assets[g.king]
    g.king_scale = RIDER_HEIGHT / max(king.bmax.y - king.bmin.y, 0.01)
    ve.r3d_model_pose_init(&g.king_pose, &king.model)
    g.clip_king_idle = ve.r3d_anim_find_clip(king.model.clips[:], "Idle")
    g.clip_king_walk = ve.r3d_anim_find_clip(king.model.clips[:], "Walk")
    ve.r3d_anim_player_init(&g.king_player, g.clip_king_idle)
    hips := ve.r3d_skeleton_find(&king.model.skeleton, "Hips")
    if hips >= 0 { g.king_hips = ve.r3d_mat4_transform_point(g.king_pose.global[hips], {0, 0, 0}) }
    horse := g.assets[g.horse]
    ve.r3d_model_pose_init(&g.horse_pose, &horse.model)
    g.horse_back_node = ve.r3d_skeleton_find(&horse.model.skeleton, "Back")
    back_y: f32 = horse.bmax.y * 0.6
    if g.horse_back_node >= 0 { back_y = ve.r3d_mat4_transform_point(g.horse_pose.global[g.horse_back_node], {0, 0, 0}).y }
    g.horse_scale = HORSE_BACK_HEIGHT / max(back_y, 0.01)
    g.clip_horse_idle = ve.r3d_anim_find_clip(horse.model.clips[:], "Idle")
    g.clip_horse_walk = ve.r3d_anim_find_clip(horse.model.clips[:], "Walk")
    g.clip_horse_gallop = ve.r3d_anim_find_clip(horse.model.clips[:], "Gallop")
    ve.r3d_anim_player_init(&g.horse_player, g.clip_horse_walk)
    fmt.printfln("templar_valley: rider scale %.3f (hips %v), horse scale %.3f (back %.2f)", g.king_scale, g.king_hips, g.horse_scale, back_y)

    setup_lighting(g)

    g.orbit = ve.r3d_orbit_camera_default()
    g.orbit.want_dist = 12
    g.orbit.want_pitch = math.to_radians(f32(30))
    g.fly = ve.r3d_fly_camera_default()
    g.cam = ve.R3D_Camera{fov_y = math.to_radians(CAMERA_FOV_DEG), near_z = 0.2, far_z = 3000}
    g.pos = {100, -562}
    g.mounted = true
    g.gait = 2
    g.follow = g.deterministic
    if g.onfoot_mode || g.bench_onfoot > 0 { onfoot_start(g) }
}

setup_lighting :: proc(g: ^Game) {
    // light: Godot's Sun, rotation_degrees (-55, -30, 0): travels along
    // Basis(euler YXZ) * -Z = (0.287, -0.819, -0.497), 55 deg up.
    ve.gl3d_set_sun(g.r, ve.R3D_Sun{
        direction = linalg.normalize([3]f32{0.2868, -0.8192, -0.4967}),
        color = {2.05, 1.95, 1.78},
        enabled = true, cast_shadows = true,
    })
    g.r.ambient = {0.02, 0.022, 0.026}
    s := &g.env.settings
    s.sky_zenith = {0.13, 0.27, 0.56}
    s.sky_horizon = {0.62, 0.70, 0.79}
    s.sky_ground = {0.30, 0.30, 0.24}
    s.sky_curve = 0.45
    s.ambient_sky = 0.8
    s.ambient_ground = 0.45
    s.fog_density = 0.0021
    s.fog_falloff = 0.010
    s.fog_base_height = 0
    s.fog_start = 40
    s.fog_max = 0.9
    s.fog_sun_scatter = 0.25
    g.r.bloom_threshold = 1.6
    g.r.bloom_strength = 0.12
    g.r.tonemap = 1
    g.r.exposure = 0.62
    g.r.vignette = 0.18
    g.r.saturation = 1.12
    g.r.grain = 0.0
}

apply_model_rotation :: proc(m: ^ve.R3D_Model, pose: ^ve.R3D_Model_Pose, joint: int, axis: [3]f32, angle: f32) {
    if joint < 0 { return }
    ve.r3d_pose_global(&m.skeleton, pose.local[:], pose.global[:])
    parent := m.skeleton.parents[joint]
    pq := ve.R3D_QUAT_IDENTITY
    if parent >= 0 { pq = ve.r3d_trs_from_matrix(pose.global[parent]).r }
    pinv := ve.R3D_Quat{-pq.x, -pq.y, -pq.z, pq.w}
    R := ve.r3d_quat_axis_angle(axis, angle)
    l := &pose.local[joint]
    l.r = ve.r3d_quat_normalize(ve.r3d_quat_mul(ve.r3d_quat_mul(pinv, R), ve.r3d_quat_mul(pq, l.r)))
}

// Seated riding pose over the sampled idle: thighs forward and apart, knees bent.
seat_pose :: proc(g: ^Game) {
    king := &g.assets[g.king].model
    sk := &king.skeleton
    for side in ([]string{"L", "R"}) {
        ul := ve.r3d_skeleton_find(sk, fmt.tprintf("UpperLeg.%s", side))
        ll := ve.r3d_skeleton_find(sk, fmt.tprintf("LowerLeg.%s", side))
        if ul < 0 || ll < 0 { continue }
        ve.r3d_pose_global(sk, g.king_pose.local[:], g.king_pose.global[:])
        x := ve.r3d_mat4_transform_point(g.king_pose.global[ul], {0, 0, 0}).x
        out: f32 = x > 0 ? 1 : -1
        apply_model_rotation(king, &g.king_pose, ul, {0, 0, 1}, out * math.to_radians(f32(24)))
        apply_model_rotation(king, &g.king_pose, ul, {1, 0, 0}, -math.to_radians(f32(72)))
        apply_model_rotation(king, &g.king_pose, ll, {1, 0, 0}, math.to_radians(f32(78)))
    }
    for side in ([]string{"L", "R"}) {
        ua := ve.r3d_skeleton_find(sk, fmt.tprintf("UpperArm.%s", side))
        la := ve.r3d_skeleton_find(sk, fmt.tprintf("LowerArm.%s", side))
        if ua < 0 || la < 0 { continue }
        apply_model_rotation(king, &g.king_pose, ua, {1, 0, 0}, -math.to_radians(f32(25)))
        apply_model_rotation(king, &g.king_pose, la, {1, 0, 0}, -math.to_radians(f32(45)))
    }
    ve.r3d_model_pose_finish(&g.king_pose, king)
}

ground_pos :: proc(g: ^Game, xz: [2]f32) -> [3]f32 { return {xz.x, ve.r3d_hf_height(&g.hf, xz.x, xz.y), xz.y} }

horse_matrix :: proc(g: ^Game) -> linalg.Matrix4f32 {
    // model faces +z; heading 0 = +x
    return yaw_scale_matrix(ground_pos(g, g.pos), math.PI / 2 - g.heading, {g.horse_scale, g.horse_scale, g.horse_scale})
}

rider_matrix :: proc(g: ^Game) -> linalg.Matrix4f32 {
    hm := horse_matrix(g)
    if !g.mounted {
        feet := g.use_feet ? [3]f32{g.pos.x, g.feet_y, g.pos.y} : ground_pos(g, g.pos)
        return yaw_scale_matrix(feet, math.PI / 2 - g.heading, {g.king_scale, g.king_scale, g.king_scale})
    }
    back := [3]f32{0, HORSE_BACK_HEIGHT / g.horse_scale, 0}
    if g.horse_back_node >= 0 { back = ve.r3d_mat4_transform_point(g.horse_pose.global[g.horse_back_node], {0, 0, 0}) }
    seat := ve.r3d_mat4_transform_point(hm, back + {0, 0, -0.15 / g.horse_scale})
    s := g.king_scale
    pos := seat - [3]f32{0, g.king_hips.y * s - 0.12, 0}
    return yaw_scale_matrix(pos, math.PI / 2 - g.heading, {s, s, s})
}

step_sim :: proc(g: ^Game, dt: f32) {
    g.anim_t += dt
    speeds := [3]f32{0, 1.8, 4.0} // Templar gait speeds (halt, walk, trot)
    horse := &g.assets[g.horse].model
    clip := g.gait == 0 ? g.clip_horse_idle : g.clip_horse_walk
    rate: f32 = g.gait == 2 ? 1.7 : 1.0
    ve.r3d_anim_play(&g.horse_player, horse.clips[:], clip, 0.3, true, rate)
    ve.r3d_anim_update(&g.horse_player, dt)
    ve.r3d_anim_evaluate(&g.horse_player, &horse.skeleton, horse.clips[:], g.horse_pose.local[:])
    ve.r3d_model_pose_finish(&g.horse_pose, horse)
    king := &g.assets[g.king].model
    kclip := g.mounted || g.gait == 0 ? g.clip_king_idle : g.clip_king_walk
    ve.r3d_anim_play(&g.king_player, king.clips[:], kclip, 0.25)
    ve.r3d_anim_update(&g.king_player, dt)
    ve.r3d_anim_evaluate(&g.king_player, &king.skeleton, king.clips[:], g.king_pose.local[:])
    if g.mounted { seat_pose(g) } else { ve.r3d_model_pose_finish(&g.king_pose, king) }
    if !g.follow {
        g.pos += [2]f32{math.cos(g.heading), math.sin(g.heading)} * speeds[g.gait] * dt
    }
}

// Godot FollowCamera pose: pivot over the target + boom at yaw/pitch/zoom.
follow_camera :: proc(g: ^Game, s: ^Look_Shot) {
    pivot_h := s.mounted ? RIDING_PIVOT_HEIGHT : ONFOOT_PIVOT_HEIGHT
    base := ground_pos(g, g.pos)
    ahead := [3]f32{math.cos(g.heading), 0, math.sin(g.heading)} * s.ahead
    pivot := base + {0, pivot_h, 0} + ahead
    yaw := math.to_radians(s.yaw_deg)
    p := math.to_radians(s.pitch_deg)
    // boom_direction: Basis(UP, yaw) * (0, sin p, cos p)
    boom := [3]f32{math.cos(p) * math.sin(yaw), math.sin(p), math.cos(p) * math.cos(yaw)}
    g.cam.position = pivot + boom * s.zoom
    g.cam.pitch = -p
    // camera looks along -boom horizontally: (sin y, -cos y) == (-boom.x, -boom.z)
    g.cam.yaw = math.atan2(-boom.x, boom.z)
}

draw_world :: proc(g: ^Game) {
    r := g.r
    cam := &g.cam
    aspect := f32(r.width) / f32(max(r.height, 1))
    vp := ve.r3d_camera_view_proj(cam, aspect, false)
    fr := ve.r3d_frustum_from_matrix(vp, true)
    focus := ground_pos(g, g.pos)
    update_grass(g, focus)
    clear(&g.of.faded)

    // gather instances for the camera
    for a in g.assets { clear(&a.inst) }
    g.stats = {}
    eye := cam.position
    for &cell in g.cells {
        if len(cell.props) == 0 || aabb_distance(eye, cell.bmin, cell.bmax) > BUILDING_RANGE { continue }
        if !ve.r3d_frustum_aabb(&fr, cell.bmin, cell.bmax) { continue }
        for pi in cell.props {
            p := &g.props[pi]
            d := linalg.length(p.center - eye) - p.radius
            rng := p.kind == 't' ? TREE_RANGE : (p.kind == 'b' ? BUILDING_RANGE : CLUTTER_RANGE)
            if d > rng || !ve.r3d_frustum_sphere(&fr, p.center, p.radius) { continue }
            if g.of.active {
                if a := prop_alpha(g, int(pi)); a < 0.99 { append(&g.of.faded, Faded{pi, a}); continue }
            }
            append(&g.assets[p.asset].inst, p.mat)
            switch p.kind { case 't': g.stats.trees += 1; case 'b': g.stats.buildings += 1; case 'c': g.stats.clutter += 1 }
        }
    }
    for _, &cell in g.grass_cells {
        for &p in cell {
            if linalg.length(p.center - eye) > GRASS_RANGE + 10 || !ve.r3d_frustum_sphere(&fr, p.center, p.radius) { continue }
            append(&g.assets[p.asset].inst, p.mat)
            g.stats.grass += 1
        }
    }

    // shadows: cascades
    ve.gl3d_env_update(r, &g.env, cam)
    hm := horse_matrix(g)
    rm := rider_matrix(g)
    if len(g.shadow_lists) != len(g.assets) { resize(&g.shadow_lists, len(g.assets)) }
    for c in 0 ..< ve.R3D_CSM_CASCADES {
        lvp := g.env.csm.light_vp[c]
        lf := ve.r3d_frustum_from_matrix(lvp, true)
        ve.gl3d_env_shadow_begin(&g.env, c)
        ve.gl3d_terrain_draw_shadow(&g.gter, &g.ter, lvp, eye)
        ve.gl3d_pbr_shadow_begin_vp(&g.pbr, lvp)
        for &l in g.shadow_lists { clear(&l) }
        reach := g.env.csm.splits[c + 1] + 40
        for &cell in g.cells {
            if len(cell.props) == 0 || aabb_distance(eye, cell.bmin, cell.bmax) > reach { continue }
            if !ve.r3d_frustum_aabb(&lf, cell.bmin, cell.bmax) { continue }
            for pi in cell.props {
                p := &g.props[pi]
                if linalg.length(p.center - eye) - p.radius > reach { continue }
                if !ve.r3d_frustum_sphere(&lf, p.center, p.radius) { continue }
                append(&g.shadow_lists[p.asset], p.mat)
            }
        }
        if c == 0 {
            for _, &cell in g.grass_cells {
                for &p in cell {
                    if !ve.r3d_frustum_sphere(&lf, p.center, p.radius) { continue }
                    append(&g.shadow_lists[p.asset], p.mat)
                }
            }
        }
        for ai in 0 ..< len(g.assets) {
            if ai == g.king || ai == g.horse { continue }
            ve.gl3d_pbr_draw_model_shadow_instanced(&g.pbr, &g.assets[ai].gl, &g.assets[ai].pose, g.shadow_lists[ai][:])
        }
        if g.mounted { ve.gl3d_pbr_draw_model_shadow(r, &g.pbr, &g.assets[g.horse].gl, &g.horse_pose, hm) }
        ve.gl3d_pbr_draw_model_shadow(r, &g.pbr, &g.assets[g.king].gl, &g.king_pose, rm)
    }
    ve.gl3d_env_shadow_end(r)

    ve.r3d_terrain_update(&g.ter, eye, &fr)
    g.stats.chunks = g.ter.drawn
    ve.gl3d_set_camera(r, cam)
    ve.gl3d_begin_frame(r)
    ve.gl3d_env_draw_sky(r, &g.env)
    ve.gl3d_terrain_draw(r, &g.gter, &g.ter, &g.env)
    for ai in 0 ..< len(g.assets) {
        if ai == g.king || ai == g.horse { continue }
        a := g.assets[ai]
        ve.gl3d_pbr_draw_model_instanced(r, &g.pbr, &a.gl, &a.pose, a.inst[:])
    }
    if g.mounted { ve.gl3d_pbr_draw_model(r, &g.pbr, &g.assets[g.horse].gl, &g.horse_pose, hm) }
    ve.gl3d_pbr_draw_model(r, &g.pbr, &g.assets[g.king].gl, &g.king_pose, rm)
    draw_faded_props(g)
    ve.gl3d_end_frame(r)
}

game_update :: proc(e: ^ve.Engine, dt: f64) {}

game_render :: proc(e: ^ve.Engine, _: ^SDL.Renderer) {
    g := cast(^Game)e.user_data
    in_ := &e.input
    dt: f32 = 1.0 / 60
    if len(g.fixture_shots) > 0 {
        if !fixture_shots_run(g) { g.exit_code = 1 }
        e.running = false
        return
    }
    if g.bench_onfoot > 0 {
        onfoot_bench(g)
        e.running = false
        return
    }
    if !g.deterministic {
        now := time.tick_now()._nsec
        if g.last_ns != 0 { dt = clamp(f32(now - g.last_ns) / 1e9, 0, 0.1) }
        g.last_ns = now
        if ve.input_is_key_pressed(in_, .ESCAPE) { e.running = false }
        if ve.input_is_key_pressed(in_, .F) { g.use_fly = !g.use_fly
            if g.use_fly { g.fly.position = g.cam.position; g.fly.yaw = g.cam.yaw; g.fly.pitch = g.cam.pitch } }
        for k, i in ([]SDL.Scancode{.NUM1, .NUM2, .NUM3}) { if ve.input_is_key_pressed(in_, k) { g.gait = i } }
        drag := in_.mouse_buttons[1] || in_.mouse_buttons[3]
        if g.use_fly {
            mv: [3]f32
            if ve.input_is_key_held(in_, .W) { mv.z += 1 }
            if ve.input_is_key_held(in_, .S) { mv.z -= 1 }
            if ve.input_is_key_held(in_, .D) { mv.x += 1 }
            if ve.input_is_key_held(in_, .A) { mv.x -= 1 }
            if ve.input_is_key_held(in_, .E) { mv.y += 1 }
            if ve.input_is_key_held(in_, .Q) { mv.y -= 1 }
            look := drag ? [2]f32{f32(in_.mouse_dx), f32(in_.mouse_dy)} : {}
            ve.r3d_fly_camera_update(&g.fly, mv, look, ve.input_is_key_held(in_, .LSHIFT), dt)
        } else {
            if drag { ve.r3d_orbit_camera_input(&g.orbit, f32(in_.mouse_dx) * 0.005, f32(in_.mouse_dy) * 0.004, 0) }
            if ve.input_is_key_held(in_, .Z) { ve.r3d_orbit_camera_input(&g.orbit, 0, 0, -10 * dt) }
            if ve.input_is_key_held(in_, .X) { ve.r3d_orbit_camera_input(&g.orbit, 0, 0, 10 * dt) }
        }
    }

    shot := -1
    if len(g.shots_dir) > 0 {
        shot = g.next_shot
        s := LOOK_SHOTS[shot]
        g.pos = s.at
        g.heading = s.heading
        g.mounted = s.mounted
        g.gait = s.mounted ? 2 : 0
        g.follow = true
        // settle the animation at the shot's time
        g.horse_player = {}; g.king_player = {}
        ve.r3d_anim_player_init(&g.horse_player, g.clip_horse_walk)
        ve.r3d_anim_player_init(&g.king_player, g.clip_king_idle)
        step_sim(g, s.t)
        follow_camera(g, &LOOK_SHOTS[shot])
        // the grass streams around the target; build it fully for the shot
        update_grass(g, ground_pos(g, g.pos))
    } else if g.bench > 0 {
        // ride east along the spur road at trot speed
        t := f32(g.frame) / 60
        a := [2]f32{100, -562}
        b := [2]f32{240, -556}
        c := [2]f32{400, -532}
        d := t * 4.0
        l1 := linalg.length(b - a)
        if d < l1 { g.pos = a + (b - a) * (d / l1); g.heading = math.atan2(b.y - a.y, b.x - a.x) }
        else { u := math.min((d - l1) / linalg.length(c - b), 1); g.pos = b + (c - b) * u; g.heading = math.atan2(c.y - b.y, c.x - b.x) }
        g.mounted = true
        g.gait = 2
        step_sim(g, dt)
        ls := Look_Shot{mounted = true, yaw_deg = 0, pitch_deg = g.bench_pitch, zoom = 8, ahead = 1.5}
        follow_camera(g, &ls)
    } else if g.of.active {
        onfoot_interactive(g, dt)
    } else {
        g.follow = false
        step_sim(g, dt)
        g.orbit.target = ground_pos(g, g.pos) + {0, g.mounted ? RIDING_PIVOT_HEIGHT : ONFOOT_PIVOT_HEIGHT, 0}
        g.orbit.ground_y = ve.r3d_hf_height(&g.hf, g.cam.position.x, g.cam.position.z)
        ve.r3d_orbit_camera_update(&g.orbit, dt)
        if g.use_fly { ve.r3d_fly_camera_apply(&g.fly, &g.cam) } else { ve.r3d_orbit_camera_apply(&g.orbit, &g.cam) }
    }

    t0 := time.tick_now()
    draw_world(g)
    if g.bench > 0 {
        append(&g.cpu_ms, time.duration_milliseconds(time.tick_since(t0)))
        gl.Finish()
        append(&g.frame_ms, time.duration_milliseconds(time.tick_since(t0)))
    }
    g.frame += 1

    if shot >= 0 {
        path := fmt.tprintf("%s/%s.ppm", g.shots_dir, LOOK_SHOTS[shot].name)
        save_ppm(g, path)
        fmt.printfln("templar_valley: %s stats chunks=%d trees=%d buildings=%d clutter=%d grass=%d", LOOK_SHOTS[shot].name,
            g.stats.chunks, g.stats.trees, g.stats.buildings, g.stats.clutter, g.stats.grass)
        g.next_shot += 1
        if g.next_shot >= len(LOOK_SHOTS) { e.running = false }
    }
    if g.bench > 0 && g.frame >= g.bench {
        ms := g.frame_ms[min(10, len(g.frame_ms)):] // skip warm-up
        sorted := slice.clone(ms, context.temp_allocator)
        slice.sort(sorted)
        sum: f64 = 0
        for v in ms { sum += v }
        csum: f64 = 0
        cms := g.cpu_ms[min(10, len(g.cpu_ms)):]
        for v in cms { csum += v }
        pct :: proc(s: []f64, p: f64) -> f64 { return s[min(int(p * f64(len(s))), len(s) - 1)] }
        fmt.printfln("BENCH templar_valley %dx%d pitch=%.0f frames=%d avg_ms=%.2f cpu_submit_ms=%.2f p50=%.2f p95=%.2f p99=%.2f max=%.2f fps_avg=%.1f chunks=%d trees=%d buildings=%d grass=%d",
            g.r.width, g.r.height, g.bench_pitch, len(ms), sum / f64(len(ms)), csum / f64(max(len(cms), 1)), pct(sorted, 0.5), pct(sorted, 0.95), pct(sorted, 0.99), sorted[len(sorted) - 1],
            1000.0 * f64(len(ms)) / sum, g.stats.chunks, g.stats.trees, g.stats.buildings, g.stats.grass)
        e.running = false
    }
}

save_ppm :: proc(g: ^Game, path: string) {
    r := g.r
    w, h := int(r.width), int(r.height)
    pixels := make([]u8, w * h * 4)
    defer delete(pixels)
    ve.gl3d_read_screen(r, pixels)
    k := max(g.ssaa, 1)
    ow, oh := w / k, h / k
    sb := strings.builder_make()
    defer strings.builder_destroy(&sb)
    fmt.sbprintf(&sb, "P6\n%d %d\n255\n", ow, oh)
    out := make([dynamic]u8, 0, ow * oh * 3 + 32)
    defer delete(out)
    append(&out, ..transmute([]u8)strings.to_string(sb))
    for y in 0 ..< oh {
        for x in 0 ..< ow {
            acc: [3]int
            for dy in 0 ..< k {
                sy := h - 1 - (y * k + dy)
                for dx in 0 ..< k {
                    i := (sy * w + x * k + dx) * 4
                    acc += {int(pixels[i]), int(pixels[i + 1]), int(pixels[i + 2])}
                }
            }
            n := k * k
            append(&out, u8(acc[0] / n), u8(acc[1] / n), u8(acc[2] / n))
        }
    }
    if os.write_entire_file(path, out[:]) != nil { fmt.eprintln("templar_valley: cannot write", path); return }
    fmt.println("templar_valley: shot ->", path)
}

game_shutdown :: proc(e: ^ve.Engine) {
    g := cast(^Game)e.user_data
    if len(g.fixture_shots) > 0 {
        fx_scene_destroy(g)
        ve.gl3d_env_destroy(&g.env)
        ve.gl3d_pbr_destroy(&g.pbr)
        ve.gl3d_shutdown(g.r)
        return
    }
    if g.of.active { destroy_valley_onfoot(g) }
    for a in g.assets {
        ve.gl3d_model_destroy(&a.gl)
        ve.r3d_model_pose_destroy(&a.pose)
        ve.r3d_model_destroy(&a.model)
        delete(a.inst)
        free(a)
    }
    for _, v in g.grass_cells { delete(v) }
    for &c in g.cells { delete(c.props) }
    delete(g.cells)
    for l in g.shadow_lists { delete(l) }
    delete(g.shadow_lists)
    delete(g.grass_cells)
    ve.r3d_model_pose_destroy(&g.king_pose)
    ve.r3d_model_pose_destroy(&g.horse_pose)
    ve.gl3d_terrain_destroy(&g.gter)
    ve.r3d_terrain_destroy(&g.ter)
    ve.r3d_heightfield_destroy(&g.hf)
    delete(g.splat)
    valley_destroy(&g.valley)
    ve.gl3d_env_destroy(&g.env)
    ve.gl3d_pbr_destroy(&g.pbr)
    ve.gl3d_shutdown(g.r)
}

game_handle_event :: proc(e: ^ve.Engine, event: ^SDL.Event) -> bool {
    g := cast(^Game)e.user_data
    if event.type == .WINDOWEVENT && event.window.event == .RESIZED && g.r != nil {
        w, h := event.window.data1, event.window.data2
        if w > 0 && h > 0 { ve.gl3d_resize(g.r, w, h) }
    }
    return true
}

main :: proc() {
    g := new(Game)
    g.ssaa = 1
    g.bench_pitch = 25
    width, height: i32 = 1280, 720
    fixture_route, csv_dir: string
    args := os.args
    for i := 1; i < len(args); i += 1 {
        switch {
        case args[i] == "--shots" && i + 1 < len(args):
            i += 1; g.shots_dir = args[i]; g.deterministic = true
        case args[i] == "--bench" && i + 1 < len(args):
            i += 1; g.bench, _ = strconv.parse_int(args[i]); g.deterministic = true
        case args[i] == "--bench-pitch" && i + 1 < len(args):
            i += 1; g.bench_pitch, _ = strconv.parse_f32(args[i])
        case args[i] == "--fixture" && i + 1 < len(args):
            i += 1; fixture_route = args[i]
        case args[i] == "--csv" && i + 1 < len(args):
            i += 1; csv_dir = args[i]
        case args[i] == "--fixture-shots" && i + 1 < len(args):
            i += 1; g.fixture_shots = args[i]; g.deterministic = true
        case args[i] == "--onfoot":
            g.onfoot_mode = true
        case args[i] == "--bench-onfoot" && i + 1 < len(args):
            i += 1; g.bench_onfoot, _ = strconv.parse_int(args[i]); g.deterministic = true
        case args[i] == "--ssaa" && i + 1 < len(args):
            i += 1; g.ssaa, _ = strconv.parse_int(args[i]); g.ssaa = clamp(g.ssaa, 1, 4)
        case args[i] == "--size" && i + 1 < len(args):
            i += 1
            parts := strings.split(args[i], "x", context.temp_allocator)
            if len(parts) == 2 {
                w, _ := strconv.parse_int(parts[0]); h, _ := strconv.parse_int(parts[1])
                width, height = i32(w), i32(h)
            }
        }
    }
    if len(fixture_route) > 0 { os.exit(run_fixture_headless(g, fixture_route, csv_dir)) }
    if len(g.shots_dir) > 0 { os.make_directory(g.shots_dir) }
    else if len(g.fixture_shots) > 0 { os.make_directory(g.fixture_shots) }
    else { g.ssaa = 1 }
    width *= i32(g.ssaa)
    height *= i32(g.ssaa)
    config := ve.EngineConfig{
        title = "templar_valley — Void Engine 3D-3 (terrain, sky, CSM, on foot)",
        width = width, height = height,
        target_fps = g.deterministic ? 1000.0 : 240.0,
        enable_hot_reload = false,
        asset_path = "assets",
        app_id = "voidengine-templar-valley",
        use_opengl = true,
        gl_vsync = g.deterministic ? 0 : 1,
    }
    e := ve.engine_init(config)
    defer ve.engine_shutdown(e)
    g.e = e
    e.user_data = g
    e.game_api = ve.GameAPI{
        init = game_init, update = game_update, render = game_render,
        shutdown = game_shutdown, handle_event = game_handle_event,
    }
    ve.engine_run(e)
    if g.exit_code != 0 { os.exit(g.exit_code) }
}
