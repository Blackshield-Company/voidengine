package templar3d

// ============================================================================
// templar3d — Void Engine 3D-1 demo: a CC0 skinned knight (KayKit, Kay
// Lousberg) idles, walks and runs a loop on a textured ground (CC0 Poly
// Haven, from Templar Valley's terrain pass) under a shadowing sun with fog.
//
// Exercises: textured glTF (base color / normal / ORM, sRGB correct), GPU
// skinning, clip crossfades with phase sync, joint-attached props, the
// orbit and free-fly cameras, skinned sun shadows. OpenGL backend.
//
// Usage:
//   templar3d [--size WxH] [--shots DIR] [--bench N]
//   --shots DIR  deterministic sim (1/60 s per frame); writes PPM shots of
//                idle, crossfade, walk, run and a wide view into DIR, then exits
//   --bench N    run N frames (deterministic sim), print FPS stats, exit
// Controls: mouse drag = orbit, Z/X = zoom, F = toggle free-fly camera
//           (WASD + Q/E, Shift = fast), 1/2/3 = idle/walk/run override,
//           0 = back to the scripted loop, Esc = quit
// ============================================================================

import "core:fmt"
import "core:math"
import "core:math/linalg"
import "core:os"
import "core:strconv"
import "core:strings"
import "core:time"
import SDL "vendor:sdl2"
import gl "vendor:OpenGL"
import ve "../../src/core"

ASSETS :: "examples/templar3d/assets"

// Templar tuning (wt-m1-terrain onfoot_check.py: WALK, RUN m/s)
WALK_SPEED :: f32(1.6)
RUN_SPEED  :: f32(4.5)
KNIGHT_HEIGHT :: f32(1.85)
LOOP_RADIUS :: f32(6)
FADE_S :: f32(0.25)

Gait :: enum { Idle, Walk, Run }

Shot :: struct {
    t:     f32,     // sim time
    name:  string,
    yaw:   f32,     // orbit yaw relative to the knight's heading (0 = behind)
    pitch: f32,     // degrees
    dist:  f32,
}

SHOTS := [?]Shot{
    {t = 1.20, name = "01-idle-front",       yaw = 200, pitch = 12, dist = 4.2},
    {t = 2.125, name = "02-crossfade-idle-walk", yaw = 110, pitch = 10, dist = 4.5},
    {t = 4.00, name = "03-walk-side",        yaw = 90,  pitch = 8,  dist = 4.5},
    {t = 9.40, name = "04-run-three-quarter", yaw = 145, pitch = 14, dist = 5.5},
    {t = 6.00, name = "05-wide-sun-shadow-fog", yaw = 160, pitch = 24, dist = 16},
    {t = 14.5, name = "06-closeup-armor-ground", yaw = 215, pitch = 28, dist = 2.6},
}

Game :: struct {
    e:          ^ve.Engine,
    r:          ^ve.GL3D_Renderer,
    pbr:        ve.GL3D_Pbr,
    knight:     ve.R3D_Model,
    knight_gl:  ve.GL3D_Model,
    pose:       ve.R3D_Model_Pose,
    player:     ve.R3D_Anim_Player,
    clip_idle, clip_walk, clip_run: int,
    knight_scale: f32,
    ground:     ve.R3D_Model,
    ground_gl:  ve.GL3D_Model,

    // knight motion
    sim_t:      f32,
    angle:      f32, // position on the loop (radians)
    pos:        [3]f32,
    heading:    f32, // radians, 0 = facing +Z
    gait:       Gait,
    override:   int, // 0 = scripted loop, 1..3 = idle/walk/run

    // cameras
    orbit:      ve.R3D_Orbit_Camera,
    fly:        ve.R3D_Fly_Camera,
    use_fly:    bool,
    cam:        ve.R3D_Camera,

    // modes
    shots_dir:  string,
    shot_order: [len(SHOTS)]int,
    next_shot:  int,
    bench:      int,
    max_shots:  int,
    frame:      int,
    deterministic: bool,
    last_ns:    i64,
    bench_start: time.Tick,
}

scripted_gait :: proc(t: f32) -> Gait {
    lt := math.mod(t, 16)
    switch {
    case lt < 2:  return .Idle
    case lt < 8:  return .Walk
    case lt < 11: return .Run
    case lt < 13: return .Walk
    }
    return .Idle
}

gait_speed :: proc(g: Gait) -> f32 {
    switch g {
    case .Idle: return 0
    case .Walk: return WALK_SPEED
    case .Run:  return RUN_SPEED
    }
    return 0
}

game_init :: proc(e: ^ve.Engine) {
    g := cast(^Game)e.user_data
    g.r = ve.gl3d_init(e.config.width, e.config.height)
    if g.r == nil { fmt.eprintln("templar3d: gl3d_init failed"); os.exit(1) }
    fmt.printfln("templar3d: GL renderer %s | %s", gl.GetString(gl.RENDERER), gl.GetString(gl.VERSION))
    ok: bool
    g.pbr, ok = ve.gl3d_pbr_init()
    if !ok { os.exit(1) }

    // --- knight ---
    g.knight, ok = ve.r3d_model_load(ASSETS + "/kaykit_knight.glb")
    if !ok { fmt.eprintln("templar3d: cannot load knight"); os.exit(1) }
    g.knight_gl = ve.gl3d_model_upload(&g.knight)
    // debug: TEMPLAR3D_HIDE=name,name hides more primitives
    if hide := os.get_env("TEMPLAR3D_HIDE", context.temp_allocator); len(hide) > 0 {
        for name in strings.split(hide, ",", context.temp_allocator) { ve.gl3d_model_set_visible(&g.knight_gl, name, false) }
    }
    // one sword and the round shield; hide the pack's spare gear
    for name in ([]string{"1H_Sword_Offhand", "Badge_Shield", "Rectangle_Shield", "Spike_Shield", "2H_Sword"}) {
        ve.gl3d_model_set_visible(&g.knight_gl, name, false)
    }
    g.clip_idle = ve.r3d_anim_find_clip(g.knight.clips[:], "Idle")
    g.clip_walk = ve.r3d_anim_find_clip(g.knight.clips[:], "Walking_A")
    g.clip_run  = ve.r3d_anim_find_clip(g.knight.clips[:], "Running_A")
    ve.r3d_model_pose_init(&g.pose, &g.knight)
    ve.r3d_anim_player_init(&g.player, g.clip_idle)
    g.player.sync_phase = true
    top: f32 = 0
    for &p in g.knight.prims { if p.skin >= 0 { top = max(top, p.bounds_max.y) } }
    g.knight_scale = top > 0 ? KNIGHT_HEIGHT / top : 1
    fmt.printfln("templar3d: knight %d prims, %d joints, %d clips, height %.2f -> scale %.3f",
        len(g.knight.prims), len(g.knight.skins[0].joints), len(g.knight.clips), top, g.knight_scale)

    // --- ground: grass field + dirt road (CC0 Poly Haven via Templar) ---
    grass_mat := ve.r3d_default_material()
    grass_mat.base_color_tex  = ve.r3d_model_add_image_file(&g.ground, ASSETS + "/ground_grass_albedo.png")
    grass_mat.normal_tex      = ve.r3d_model_add_image_file(&g.ground, ASSETS + "/ground_grass_normal.png")
    grass_mat.metal_rough_tex = ve.r3d_model_add_image_file(&g.ground, ASSETS + "/ground_grass_orm.png")
    grass_mat.occlusion_tex   = grass_mat.metal_rough_tex
    grass_mat.uv_scale = {40, 40} // 160 m plane: 4 m per tile
    dirt_mat := grass_mat
    dirt_mat.base_color_tex  = ve.r3d_model_add_image_file(&g.ground, ASSETS + "/ground_dirt_albedo.png")
    dirt_mat.normal_tex      = ve.r3d_model_add_image_file(&g.ground, ASSETS + "/ground_dirt_normal.png")
    dirt_mat.metal_rough_tex = ve.r3d_model_add_image_file(&g.ground, ASSETS + "/ground_dirt_orm.png")
    dirt_mat.occlusion_tex   = dirt_mat.metal_rough_tex
    dirt_mat.uv_scale = {1, 1}
    append(&g.ground.materials, grass_mat, dirt_mat)
    plane := ve.r3d_mesh_plane(160, 32)
    ve.r3d_model_add_mesh(&g.ground, "grass", &plane, 0)
    ve.r3d_mesh_data_destroy(&plane)
    road := road_ring(LOOP_RADIUS, 1.6, 96)
    ve.r3d_model_add_mesh(&g.ground, "road", &road, 1)
    ve.r3d_mesh_data_destroy(&road)
    g.ground_gl = ve.gl3d_model_upload(&g.ground)

    // --- light, sky, post ---
    ve.gl3d_set_sun(g.r, ve.R3D_Sun{
        direction = linalg.normalize([3]f32{-0.55, -0.62, -0.42}),
        color = {3.3, 2.9, 2.35},
        enabled = true, cast_shadows = true,
        shadow_radius = 9, shadow_bias = 0.0012,
    })
    ve.gl3d_set_atmosphere(g.r, ve.R3D_Atmosphere{
        sky_zenith = {0.22, 0.40, 0.78, 1},
        sky_horizon = {0.78, 0.80, 0.82, 1},
        fog_color = {0.66, 0.70, 0.74, 1},
        fog_density = 0.011,
        sun_halo = 1.0,
    })
    g.r.ambient = {0.42, 0.46, 0.55}
    g.r.bloom_threshold = 1.2
    g.r.bloom_strength = 0.25
    g.r.exposure = 1.0
    g.r.vignette = 0.25
    g.r.saturation = 1.05
    if os.get_env("TEMPLAR3D_NOBLOOM", context.temp_allocator) == "1" { g.r.bloom = false }

    // --- cameras ---
    g.orbit = ve.r3d_orbit_camera_default()
    g.orbit.ground_y = 0
    g.orbit.want_dist = 5
    g.orbit.want_pitch = math.to_radians(f32(15))
    g.fly = ve.r3d_fly_camera_default()
    g.fly.position = {0, 2, 10}
    g.cam = ve.R3D_Camera{fov_y = math.to_radians(f32(55)), near_z = 0.1, far_z = 400}

    // shots in time order
    for i in 0 ..< len(SHOTS) { g.shot_order[i] = i }
    for i in 1 ..< len(SHOTS) {
        j := i
        for j > 0 && SHOTS[g.shot_order[j - 1]].t > SHOTS[g.shot_order[j]].t {
            g.shot_order[j - 1], g.shot_order[j] = g.shot_order[j], g.shot_order[j - 1]
            j -= 1
        }
    }
    g.bench_start = time.tick_now()
}

// Flat ring strip (the dirt path the knight walks), uv u across, v along.
road_ring :: proc(radius, width: f32, segs: int) -> (m: ve.R3D_Mesh_Data) {
    for i in 0 ..= segs {
        a := f32(i) / f32(segs) * math.TAU
        c, s := math.cos(a), math.sin(a)
        v := f32(i) / f32(segs) * (math.TAU * radius / 2.5) // 2.5 m per texture repeat
        r0, r1 := radius - width / 2, radius + width / 2
        append(&m.vertices,
            ve.R3D_Vertex{pos = {r0 * s, 0.015, r0 * c}, normal = {0, 1, 0}, uv = {0, v}},
            ve.R3D_Vertex{pos = {r1 * s, 0.015, r1 * c}, normal = {0, 1, 0}, uv = {width / 2.5, v}},
        )
    }
    for i in 0 ..< segs {
        b := u32(i * 2)
        append(&m.indices, b, b + 1, b + 3, b, b + 3, b + 2)
    }
    return
}

step_sim :: proc(g: ^Game, dt: f32) {
    g.sim_t += dt
    want := scripted_gait(g.sim_t)
    if g.override > 0 { want = Gait(g.override - 1) }
    if want != g.gait {
        g.gait = want
        clip := g.clip_idle
        switch want {
        case .Idle: clip = g.clip_idle
        case .Walk: clip = g.clip_walk
        case .Run:  clip = g.clip_run
        }
        ve.r3d_anim_play(&g.player, g.knight.clips[:], clip, FADE_S)
    }
    // ground speed eases with the crossfade so the feet don't skate
    speed := gait_speed(g.gait)
    if g.player.fading && g.player.prev.clip >= 0 {
        prev := g.player.prev.clip == g.clip_run ? RUN_SPEED : (g.player.prev.clip == g.clip_walk ? WALK_SPEED : 0)
        w := ve.r3d_anim_fade_weight(&g.player)
        speed = prev + (speed - prev) * w
    }
    g.angle += speed * dt / LOOP_RADIUS
    g.pos = {LOOP_RADIUS * math.sin(g.angle), 0, LOOP_RADIUS * math.cos(g.angle)}
    g.heading = g.angle + math.PI / 2 // tangent of the loop (counter-clockwise seen from above)

    ve.r3d_anim_update(&g.player, dt)
    ve.r3d_anim_evaluate(&g.player, &g.knight.skeleton, g.knight.clips[:], g.pose.local[:])
    ve.r3d_model_pose_finish(&g.pose, &g.knight)
}

knight_matrix :: proc(g: ^Game) -> linalg.Matrix4f32 {
    x := ve.R3D_Trs{
        t = g.pos,
        r = ve.r3d_quat_axis_angle({0, 1, 0}, g.heading),
        s = {g.knight_scale, g.knight_scale, g.knight_scale},
    }
    return ve.r3d_trs_matrix(x)
}

// orbit yaw that puts the camera `rel` degrees around the knight from behind
camera_yaw_for :: proc(g: ^Game, rel_deg: f32) -> f32 {
    // knight forward in world: (sin h, 0, cos h). Camera yaw y looks along
    // (sin y, 0, -cos y); "behind" means looking along the knight's forward.
    behind := math.atan2(math.sin(g.heading), -math.cos(g.heading))
    return behind + math.to_radians(rel_deg)
}

game_update :: proc(e: ^ve.Engine, dt: f64) {}

game_render :: proc(e: ^ve.Engine, _: ^SDL.Renderer) {
    g := cast(^Game)e.user_data
    in_ := &e.input
    dt: f32 = 1.0 / 60
    if !g.deterministic {
        now := time.tick_now()._nsec
        if g.last_ns != 0 { dt = clamp(f32(now - g.last_ns) / 1e9, 0, 0.1) }
        g.last_ns = now
    }

    // input (interactive only)
    if !g.deterministic {
        if ve.input_is_key_pressed(in_, .ESCAPE) { e.running = false }
        if ve.input_is_key_pressed(in_, .F) { g.use_fly = !g.use_fly
            if g.use_fly { g.fly.position = g.cam.position; g.fly.yaw = g.cam.yaw; g.fly.pitch = g.cam.pitch } }
        for k, i in ([]SDL.Scancode{.NUM0, .NUM1, .NUM2, .NUM3}) {
            if ve.input_is_key_pressed(in_, k) { g.override = i }
        }
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
            if ve.input_is_key_held(in_, .Z) { ve.r3d_orbit_camera_input(&g.orbit, 0, 0, -6 * dt) }
            if ve.input_is_key_held(in_, .X) { ve.r3d_orbit_camera_input(&g.orbit, 0, 0, 6 * dt) }
        }
    }

    step_sim(g, dt)

    // camera
    shot_idx := -1
    if len(g.shots_dir) > 0 && g.next_shot < len(SHOTS) {
        si := g.shot_order[g.next_shot]
        if g.sim_t + 1e-4 >= SHOTS[si].t { shot_idx = si }
    }
    g.orbit.target = g.pos + {0, 1.15, 0}
    if shot_idx >= 0 {
        s := SHOTS[shot_idx]
        g.orbit.damping = 0
        g.orbit.want_yaw = camera_yaw_for(g, s.yaw)
        g.orbit.want_pitch = math.to_radians(s.pitch)
        g.orbit.want_dist = s.dist
        if s.dist > 10 { g.orbit.target = g.pos + {0, 0.9, 0} }
    } else if g.deterministic {
        g.orbit.damping = 0
        g.orbit.want_yaw = camera_yaw_for(g, 135)
        g.orbit.want_pitch = math.to_radians(f32(15))
        g.orbit.want_dist = 5
    }
    ve.r3d_orbit_camera_update(&g.orbit, dt)
    if g.use_fly { ve.r3d_fly_camera_apply(&g.fly, &g.cam) } else { ve.r3d_orbit_camera_apply(&g.orbit, &g.cam) }

    // draw
    r := g.r
    ve.gl3d_set_camera(r, &g.cam)
    km := knight_matrix(g)
    ve.gl3d_shadow_pass_begin(r, g.pos + {0, 1, 0})
    ve.gl3d_pbr_shadow_begin(r, &g.pbr)
    ve.gl3d_pbr_draw_model_shadow(r, &g.pbr, &g.knight_gl, &g.pose, km)
    ve.gl3d_shadow_pass_end(r)
    ve.gl3d_begin_frame(r)
    ve.gl3d_pbr_draw_model(r, &g.pbr, &g.ground_gl, nil, linalg.MATRIX4F32_IDENTITY)
    ve.gl3d_pbr_draw_model(r, &g.pbr, &g.knight_gl, &g.pose, km)
    ve.gl3d_end_frame(r)
    g.frame += 1

    if shot_idx >= 0 {
        path := fmt.tprintf("%s/%s.ppm", g.shots_dir, SHOTS[shot_idx].name)
        save_ppm(r, path)
        g.next_shot += 1
        if g.next_shot >= min(len(SHOTS), g.max_shots) { e.running = false }
    }
    if g.bench > 0 && g.frame >= g.bench {
        secs := time.duration_seconds(time.tick_since(g.bench_start))
        fmt.printfln("BENCH templar3d frames=%d seconds=%.2f avg_fps=%.1f", g.frame, secs, f64(g.frame) / secs)
        e.running = false
    }
}

save_ppm :: proc(r: ^ve.GL3D_Renderer, path: string) {
    w, h := int(r.width), int(r.height)
    pixels := make([]u8, w * h * 4)
    defer delete(pixels)
    ve.gl3d_read_screen(r, pixels)
    sb := strings.builder_make()
    defer strings.builder_destroy(&sb)
    fmt.sbprintf(&sb, "P6\n%d %d\n255\n", w, h)
    out := make([dynamic]u8, 0, w * h * 3 + 32)
    defer delete(out)
    append(&out, ..transmute([]u8)strings.to_string(sb))
    for y in 0 ..< h {
        sy := h - 1 - y
        for x in 0 ..< w {
            i := (sy * w + x) * 4
            append(&out, pixels[i], pixels[i + 1], pixels[i + 2])
        }
    }
    if os.write_entire_file(path, out[:]) != nil {
        fmt.eprintln("templar3d: cannot write", path)
        return
    }
    fmt.println("templar3d: shot ->", path)
}

game_shutdown :: proc(e: ^ve.Engine) {
    g := cast(^Game)e.user_data
    ve.gl3d_model_destroy(&g.knight_gl)
    ve.gl3d_model_destroy(&g.ground_gl)
    ve.r3d_model_pose_destroy(&g.pose)
    ve.r3d_anim_player_destroy(&g.player)
    ve.r3d_model_destroy(&g.knight)
    ve.r3d_model_destroy(&g.ground)
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
    g.max_shots = len(SHOTS)
    width, height: i32 = 1280, 720
    args := os.args
    for i := 1; i < len(args); i += 1 {
        switch {
        case args[i] == "--shots" && i + 1 < len(args):
            i += 1; g.shots_dir = args[i]; g.deterministic = true
        case args[i] == "--max-shots" && i + 1 < len(args):
            i += 1; g.max_shots, _ = strconv.parse_int(args[i])
        case args[i] == "--bench" && i + 1 < len(args):
            i += 1; g.bench, _ = strconv.parse_int(args[i]); g.deterministic = true
        case args[i] == "--size" && i + 1 < len(args):
            i += 1
            parts := strings.split(args[i], "x", context.temp_allocator)
            if len(parts) == 2 {
                w, _ := strconv.parse_int(parts[0]); h, _ := strconv.parse_int(parts[1])
                width, height = i32(w), i32(h)
            }
        }
    }
    if len(g.shots_dir) > 0 { os.make_directory(g.shots_dir) }

    config := ve.EngineConfig{
        title = "templar3d — Void Engine 3D-1 (knight, PBR, skinning)",
        width = width, height = height,
        target_fps = g.deterministic ? 1000.0 : 240.0,
        enable_hot_reload = false,
        asset_path = "assets",
        app_id = "voidengine-templar3d",
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
}
