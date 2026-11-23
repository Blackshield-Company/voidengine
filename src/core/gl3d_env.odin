package voidengine

// ============================================================================
// gl3d_env — outdoor environment for the GL 3D path: cascaded sun shadows
// (csm3d.odin), an analytic sky, height fog with aerial perspective that
// fades into the sky colour at the horizon, and the shared GLSL lighting
// code used by gl3d_terrain and gl3d_pbr when an environment is bound.
//
// Bindings: UBO "Env" at binding 2, the cascade depth array at texture unit 6.
// Frame order:
//   gl3d_env_update(r, env, &csm, &cam)          // matrices + UBO
//   for c in 0..<3 { gl3d_env_shadow_begin(env, c); draw casters with
//                    env.csm.light_vp[c]; }  gl3d_env_shadow_end(r)
//   gl3d_begin_frame(r); gl3d_env_draw_sky(r, env); draw scene; gl3d_end_frame(r)
// ============================================================================

import "core:fmt"
import "core:math/linalg"
import gl "vendor:OpenGL"

GL3D_ENV_BINDING :: 2
GL3D_ENV_CSM_UNIT :: 6

// Artist-facing environment settings (all linear colours).
R3D_Env_Settings :: struct {
    sky_zenith:      linalg.Vector3f32,
    sky_horizon:     linalg.Vector3f32,
    sky_ground:      linalg.Vector3f32,
    sky_curve:       f32, // horizon -> zenith gradient exponent (< 1: blue starts low)
    sun_disc_cos:    f32, // cos of the sun disc's angular radius
    sun_disc_power:  f32, // disc brightness x sun colour
    mie_strength:    f32, // forward-scatter glow around the sun
    mie_g:           f32, // Henyey-Greenstein g (0.7..0.9)
    fog_density:     f32, // extinction per metre at fog_base_height
    fog_falloff:     f32, // per metre of height (0 = uniform)
    fog_base_height: f32,
    fog_start:       f32, // metres of clear air in front of the camera
    fog_max:         f32, // max opacity (1 = fully hides at range)
    fog_sun_scatter: f32, // sun-coloured in-scatter toward the sun
    fog_sun_power:   f32,
    ambient_sky:     f32, // hemisphere ambient from the sky colours
    ambient_ground:  f32, // ... and from the ground bounce
    shadow_normal_offset: f32, // texels
    shadow_bias:     f32,      // depth units (0..1 range)
    shadow_blend:    f32,      // fraction of a cascade blended into the next
    shadow_pcf:      f32,      // PCF tap spacing in texels
}

r3d_env_default :: proc "contextless" () -> R3D_Env_Settings {
    return {
        sky_zenith = {0.16, 0.32, 0.62}, sky_horizon = {0.58, 0.66, 0.76}, sky_ground = {0.26, 0.28, 0.25},
        sky_curve = 0.5, sun_disc_cos = 0.99988, sun_disc_power = 40, mie_strength = 0.12, mie_g = 0.8,
        fog_density = 0.0016, fog_falloff = 0.012, fog_base_height = 0, fog_start = 30, fog_max = 0.92,
        fog_sun_scatter = 0.35, fog_sun_power = 6,
        ambient_sky = 1.0, ambient_ground = 0.35,
        shadow_normal_offset = 1.5, shadow_bias = 0.0004, shadow_blend = 0.15, shadow_pcf = 1.0,
    }
}

// std140 mirror of the GLSL Env block (matrices first: Odin aligns them to 32).
GL3D_Env_Uniforms :: struct {
    csm_vp:      [R3D_CSM_CASCADES]linalg.Matrix4f32,
    csm_splits:  linalg.Vector4f32, // xyz cascade far depths, w = shadows on
    csm_params:  linalg.Vector4f32, // x normal offset (texels), y bias, z blend band, w pcf spacing
    csm_texel:   linalg.Vector4f32, // xyz metres per texel per cascade
    cam_fwd:     linalg.Vector4f32,
    fog:         linalg.Vector4f32, // density, falloff, base height, start
    fog2:        linalg.Vector4f32, // max, sun scatter, sun power, -
    sky:         linalg.Vector4f32, // disc cos, disc power, mie strength, mie g
    sky_ground:  linalg.Vector4f32, // rgb, w = ambient ground
    sky_zenith:  linalg.Vector4f32, // rgb, w = curve
    sky_horizon: linalg.Vector4f32, // rgb, w = ambient sky
}

GL3D_ENV_BLOCK :: `
layout(std140) uniform Env {
    mat4 e_csm_vp[3];
    vec4 e_csm_splits;
    vec4 e_csm_params;
    vec4 e_csm_texel;
    vec4 e_cam_fwd;
    vec4 e_fog;
    vec4 e_fog2;
    vec4 e_sky;
    vec4 e_sky_ground;
    vec4 e_sky_zenith;
    vec4 e_sky_horizon;
};
uniform sampler2DArrayShadow u_csm;
`

// Needs GL3D_PBR_FRAME_BLOCK and GL3D_ENV_BLOCK before it.
GL3D_ENV_FUNCS :: `
const float ENV_PI = 3.14159265;

vec3 env_sky(vec3 dir, bool with_sun) {
    vec3 L = normalize(-u_sun_dir.xyz);
    float e = dir.y;
    float up = clamp(e, 0.0, 1.0);
    vec3 col = mix(e_sky_horizon.rgb, e_sky_zenith.rgb, pow(up, e_sky_zenith.w));
    if (e < 0.0) col = mix(e_sky_horizon.rgb, e_sky_ground.rgb, 1.0 - exp(e * 10.0));
    float mu = dot(dir, L);
    float g = e_sky.w;
    float hg = (1.0 - g * g) / (4.0 * ENV_PI * pow(max(1.0 + g * g - 2.0 * g * mu, 1e-4), 1.5));
    col += u_sun_color.rgb * e_sky.z * hg * (0.35 + 0.65 * exp(-up * 4.0));
    if (with_sun) {
        float disc = smoothstep(e_sky.x, e_sky.x + (1.0 - e_sky.x) * 0.35, mu);
        col += u_sun_color.rgb * disc * e_sky.y;
    }
    return col;
}

// Height fog: optical depth of an exponential-height medium along the ray.
float env_fog_amount(vec3 world, out vec3 rd) {
    vec3 ro = u_camera_pos.xyz;
    vec3 d3 = world - ro;
    float dist = length(d3);
    rd = d3 / max(dist, 1e-4);
    float d = max(dist - e_fog.w, 0.0);
    float a = e_fog.y;
    float dens = e_fog.x * exp(-a * (ro.y - e_fog.z));
    float dy = rd.y * d;
    float k = abs(a * dy) > 1e-4 ? (1.0 - exp(-a * dy)) / (a * dy) : 1.0;
    return min(1.0 - exp(-dens * d * k), e_fog2.x);
}

vec3 env_apply_fog(vec3 col, vec3 world) {
    vec3 rd;
    float f = env_fog_amount(world, rd);
    vec3 L = normalize(-u_sun_dir.xyz);
    vec3 fc = env_sky(normalize(vec3(rd.x, max(rd.y, 0.0) * 0.35, rd.z)), false);
    fc += u_sun_color.rgb * e_fog2.y * pow(max(dot(rd, L), 0.0), e_fog2.z);
    return mix(col, fc, f);
}

float env_csm_tap(int c, vec3 world, vec3 n, float ndl) {
    float texel = e_csm_texel[c];
    vec3 p = world + n * texel * e_csm_params.x * (1.0 + 2.0 * (1.0 - ndl));
    vec4 lp = e_csm_vp[c] * vec4(p, 1.0);
    vec3 q = lp.xyz / lp.w * 0.5 + 0.5;
    if (q.x <= 0.0 || q.x >= 1.0 || q.y <= 0.0 || q.y >= 1.0 || q.z >= 1.0) return 1.0;
    float ref = q.z - e_csm_params.y;
    vec2 ts = e_csm_params.w / vec2(textureSize(u_csm, 0).xy);
    float sum = 0.0;
    for (int x = -1; x <= 1; ++x)
        for (int y = -1; y <= 1; ++y)
            sum += texture(u_csm, vec4(q.xy + vec2(x, y) * ts, float(c), ref));
    return sum / 9.0;
}

// Sun visibility 0..1 (geometric normal n, n.L = ndl).
float env_shadow(vec3 world, vec3 n, float ndl) {
    if (e_csm_splits.w < 0.5) return 1.0;
    float depth = dot(world - u_camera_pos.xyz, e_cam_fwd.xyz);
    int c = depth < e_csm_splits.x ? 0 : (depth < e_csm_splits.y ? 1 : (depth < e_csm_splits.z ? 2 : 3));
    if (c > 2) return 1.0;
    float s = env_csm_tap(c, world, n, ndl);
    float far_d = e_csm_splits[c];
    float near_d = c == 0 ? 0.0 : e_csm_splits[c - 1];
    float band = (far_d - near_d) * e_csm_params.z;
    float t = (depth - (far_d - band)) / max(band, 1e-3);
    if (t > 0.0) {
        float s2 = c < 2 ? env_csm_tap(c + 1, world, n, ndl) : 1.0;
        s = mix(s, s2, clamp(t, 0.0, 1.0));
    }
    return s;
}

float env_D_ggx(float ndh, float a) {
    float a2 = a * a;
    float d = ndh * ndh * (a2 - 1.0) + 1.0;
    return a2 / (ENV_PI * d * d + 1e-7);
}
float env_G_smith(float ndv, float ndl, float rough) {
    float k = (rough + 1.0) * (rough + 1.0) / 8.0;
    return (ndv / (ndv * (1.0 - k) + k)) * (ndl / (ndl * (1.0 - k) + k));
}
vec3 env_F_schlick(float c, vec3 f0) {
    return f0 + (1.0 - f0) * pow(clamp(1.0 - c, 0.0, 1.0), 5.0);
}

// Full outdoor lighting: shadowed sun (GGX), hemisphere sky/ground ambient
// with a sky-tinted specular term, AO, fog. Ng = geometric normal.
vec3 env_light(vec3 N, vec3 Ng, vec3 V, vec3 albedo, float metal, float rough, float ao, vec3 world) {
    vec3 f0 = mix(vec3(0.04), albedo, metal);
    vec3 L = normalize(-u_sun_dir.xyz);
    float ndl = max(dot(N, L), 0.0);
    vec3 lit = vec3(0.0);
    float sh = env_shadow(world, Ng, max(dot(Ng, L), 0.0));
    if (ndl > 0.0) {
        vec3 hv = L + V;
        vec3 H = hv / max(length(hv), 1e-5);
        float ndv = max(dot(N, V), 1e-4);
        float ndh = max(dot(N, H), 0.0);
        float vdh = clamp(dot(V, H), 0.0, 1.0);
        vec3 F = env_F_schlick(vdh, f0);
        vec3 spec = env_D_ggx(ndh, rough * rough) * env_G_smith(ndv, ndl, rough) * F / (4.0 * ndv * ndl + 1e-4);
        vec3 kd = (1.0 - F) * (1.0 - metal);
        lit += (kd * albedo / ENV_PI + spec) * ndl * ENV_PI * u_sun_color.rgb * sh;
    }
    float hemi = N.y * 0.5 + 0.5;
    vec3 sky_amb = mix(e_sky_horizon.rgb, e_sky_zenith.rgb, 0.5) * e_sky_horizon.w;
    vec3 gnd_amb = e_sky_ground.rgb * e_sky_ground.w + u_sun_color.rgb * e_sky_ground.w * 0.08;
    vec3 amb = mix(gnd_amb, sky_amb, hemi) + u_ambient.rgb;
    vec3 R = reflect(-V, N);
    vec3 env_spec = env_sky(normalize(vec3(R.x, abs(R.y), R.z)), false);
    float ndv = max(dot(N, V), 0.0);
    vec3 Fa = f0 + (max(vec3(1.0 - rough), f0) - f0) * pow(1.0 - ndv, 5.0);
    vec3 kd = (1.0 - Fa) * (1.0 - metal);
    // occlude ambient a bit where the sun is shadowed (cheap sky occlusion)
    float occ = ao * mix(0.75, 1.0, sh);
    lit += (kd * albedo * amb + Fa * env_spec * (1.0 - rough) * 0.6) * occ;
    return env_apply_fog(lit, world);
}
`

GL3D_ENV_SKY_VERT :: `#version 330 core
out vec2 v_ndc;
void main() {
    vec2 p = vec2(gl_VertexID == 1 ? 3.0 : -1.0, gl_VertexID == 2 ? 3.0 : -1.0);
    v_ndc = p;
    gl_Position = vec4(p, 1.0, 1.0);
}
`

GL3D_ENV_SKY_FRAG :: "#version 330 core\n" + GL3D_PBR_FRAME_BLOCK + GL3D_ENV_BLOCK + GL3D_ENV_FUNCS + `
in vec2 v_ndc;
uniform mat4 u_inv_vp;
out vec4 frag_color;
void main() {
    vec4 far_pt = u_inv_vp * vec4(v_ndc, 1.0, 1.0);
    vec3 dir = normalize(far_pt.xyz / far_pt.w - u_camera_pos.xyz);
    frag_color = vec4(env_sky(dir, true), 1.0);
}
`

GL3D_Env :: struct {
    settings:   R3D_Env_Settings,
    csm:        R3D_Csm,
    ubo:        u32,
    csm_tex:    u32,
    csm_fbo:    u32,
    sky_prog:   u32,
    sky_vao:    u32,
    loc_inv_vp: i32,
    shadows:    bool,
    uniforms:   GL3D_Env_Uniforms,
}

// Bind the Env block and the cascade sampler of a program that includes
// GL3D_ENV_BLOCK (no-op for programs without them).
gl3d_env_bind_program :: proc(prog: u32) {
    bi := gl.GetUniformBlockIndex(prog, "Env")
    if bi != gl.INVALID_INDEX { gl.UniformBlockBinding(prog, bi, GL3D_ENV_BINDING) }
    fi := gl.GetUniformBlockIndex(prog, "Frame")
    if fi != gl.INVALID_INDEX { gl.UniformBlockBinding(prog, fi, 0) }
    loc := gl.GetUniformLocation(prog, "u_csm")
    if loc >= 0 {
        gl.UseProgram(prog)
        gl.Uniform1i(loc, GL3D_ENV_CSM_UNIT)
        gl.UseProgram(0)
    }
}

gl3d_env_init :: proc(env: ^GL3D_Env, csm_res := 2048) -> bool {
    env.settings = r3d_env_default()
    env.csm = r3d_csm_default()
    env.csm.res = csm_res
    env.shadows = true

    ok: bool
    env.sky_prog, ok = gl3d_build_program(GL3D_ENV_SKY_VERT, GL3D_ENV_SKY_FRAG)
    if !ok { fmt.eprintln("gl3d_env: sky program failed"); return false }
    gl3d_env_bind_program(env.sky_prog)
    env.loc_inv_vp = gl.GetUniformLocation(env.sky_prog, "u_inv_vp")
    gl.GenVertexArrays(1, &env.sky_vao)

    gl.GenBuffers(1, &env.ubo)
    gl.BindBuffer(gl.UNIFORM_BUFFER, env.ubo)
    gl.BufferData(gl.UNIFORM_BUFFER, size_of(GL3D_Env_Uniforms), nil, gl.DYNAMIC_DRAW)
    gl.BindBuffer(gl.UNIFORM_BUFFER, 0)

    res := i32(csm_res)
    gl.GenTextures(1, &env.csm_tex)
    gl.BindTexture(gl.TEXTURE_2D_ARRAY, env.csm_tex)
    gl.TexImage3D(gl.TEXTURE_2D_ARRAY, 0, gl.DEPTH_COMPONENT24, res, res, R3D_CSM_CASCADES, 0, gl.DEPTH_COMPONENT, gl.FLOAT, nil)
    gl.TexParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_MIN_FILTER, gl.LINEAR)
    gl.TexParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_MAG_FILTER, gl.LINEAR)
    gl.TexParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE)
    gl.TexParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE)
    gl.TexParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_COMPARE_MODE, gl.COMPARE_REF_TO_TEXTURE)
    gl.TexParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_COMPARE_FUNC, gl.LEQUAL)
    gl.BindTexture(gl.TEXTURE_2D_ARRAY, 0)
    gl.GenFramebuffers(1, &env.csm_fbo)
    gl.BindFramebuffer(gl.FRAMEBUFFER, env.csm_fbo)
    gl.FramebufferTextureLayer(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, env.csm_tex, 0, 0)
    gl.DrawBuffer(gl.NONE)
    gl.ReadBuffer(gl.NONE)
    status := gl.CheckFramebufferStatus(gl.FRAMEBUFFER)
    gl.BindFramebuffer(gl.FRAMEBUFFER, 0)
    if status != gl.FRAMEBUFFER_COMPLETE { fmt.eprintln("gl3d_env: cascade framebuffer incomplete", status); return false }
    return true
}

gl3d_env_destroy :: proc(env: ^GL3D_Env) {
    gl.DeleteProgram(env.sky_prog)
    gl.DeleteVertexArrays(1, &env.sky_vao)
    gl.DeleteBuffers(1, &env.ubo)
    gl.DeleteTextures(1, &env.csm_tex)
    gl.DeleteFramebuffers(1, &env.csm_fbo)
    env^ = {}
}

// Fit the cascades to the camera and upload the Env block (and the Frame
// block, so the sun and camera match this frame). Binds both.
gl3d_env_update :: proc(r: ^GL3D_Renderer, env: ^GL3D_Env, cam: ^R3D_Camera) {
    aspect := f32(r.width) / f32(max(r.height, 1))
    r3d_csm_update(&env.csm, cam, aspect, r.sun.direction)
    s := &env.settings
    u := &env.uniforms
    for i in 0 ..< R3D_CSM_CASCADES { u.csm_vp[i] = env.csm.light_vp[i] }
    u.csm_splits = {env.csm.splits[1], env.csm.splits[2], env.csm.splits[3], env.shadows ? 1 : 0}
    u.csm_params = {s.shadow_normal_offset, s.shadow_bias, s.shadow_blend, s.shadow_pcf}
    u.csm_texel = {env.csm.texel_world[0], env.csm.texel_world[1], env.csm.texel_world[2], 0}
    _, _, fwd := r3d_camera_basis(cam)
    u.cam_fwd = {fwd.x, fwd.y, fwd.z, 0}
    u.fog = {s.fog_density, s.fog_falloff, s.fog_base_height, s.fog_start}
    u.fog2 = {s.fog_max, s.fog_sun_scatter, s.fog_sun_power, 0}
    u.sky = {s.sun_disc_cos, s.sun_disc_power, s.mie_strength, s.mie_g}
    u.sky_ground = {s.sky_ground.x, s.sky_ground.y, s.sky_ground.z, s.ambient_ground}
    u.sky_zenith = {s.sky_zenith.x, s.sky_zenith.y, s.sky_zenith.z, s.sky_curve}
    u.sky_horizon = {s.sky_horizon.x, s.sky_horizon.y, s.sky_horizon.z, s.ambient_sky}
    gl.BindBuffer(gl.UNIFORM_BUFFER, env.ubo)
    gl.BufferSubData(gl.UNIFORM_BUFFER, 0, size_of(GL3D_Env_Uniforms), u)
    gl.BindBufferBase(gl.UNIFORM_BUFFER, GL3D_ENV_BINDING, env.ubo)
    gl3d_env_upload_frame(r)
}

// Upload the renderer's Frame block for its current camera and sun.
gl3d_env_upload_frame :: proc(r: ^GL3D_Renderer) {
    aspect := f32(r.width) / f32(max(r.height, 1))
    frame := r3d_make_frame_uniforms(&r.camera, aspect, false, r.ambient, r.lights[:], &r.sun, &r.atmosphere)
    gl.BindBuffer(gl.UNIFORM_BUFFER, r.ubo)
    gl.BufferSubData(gl.UNIFORM_BUFFER, 0, size_of(R3D_Frame_Uniforms), &frame)
    gl.BindBufferBase(gl.UNIFORM_BUFFER, 0, r.ubo)
}

// Bind the cascade array and Env block for drawing lit geometry.
gl3d_env_bind :: proc(env: ^GL3D_Env) {
    gl.BindBufferBase(gl.UNIFORM_BUFFER, GL3D_ENV_BINDING, env.ubo)
    gl.ActiveTexture(gl.TEXTURE0 + GL3D_ENV_CSM_UNIT)
    gl.BindTexture(gl.TEXTURE_2D_ARRAY, env.csm_tex)
    gl.ActiveTexture(gl.TEXTURE0)
}

// Render target for cascade c; draw casters with env.csm.light_vp[c].
gl3d_env_shadow_begin :: proc(env: ^GL3D_Env, c: int) {
    gl.BindFramebuffer(gl.FRAMEBUFFER, env.csm_fbo)
    gl.FramebufferTextureLayer(gl.FRAMEBUFFER, gl.DEPTH_ATTACHMENT, env.csm_tex, 0, i32(c))
    res := i32(env.csm.res)
    gl.Viewport(0, 0, res, res)
    gl.Enable(gl.DEPTH_TEST)
    gl.DepthFunc(gl.LESS)
    gl.DepthMask(true)
    gl.Disable(gl.BLEND)
    gl.Disable(gl.CULL_FACE)
    gl.Clear(gl.DEPTH_BUFFER_BIT)
    gl.Enable(gl.POLYGON_OFFSET_FILL)
    gl.PolygonOffset(1.5, 2.0)
}

gl3d_env_shadow_end :: proc(r: ^GL3D_Renderer) {
    gl.Disable(gl.POLYGON_OFFSET_FILL)
    gl.BindFramebuffer(gl.FRAMEBUFFER, 0)
}

// Draw the sky behind everything. Call right after gl3d_begin_frame.
gl3d_env_draw_sky :: proc(r: ^GL3D_Renderer, env: ^GL3D_Env) {
    gl3d_env_upload_frame(r)
    gl3d_env_bind(env)
    aspect := f32(r.width) / f32(max(r.height, 1))
    vp := r3d_camera_view_proj(&r.camera, aspect, false)
    inv := linalg.inverse(vp)
    gl.DepthMask(false)
    gl.Disable(gl.DEPTH_TEST)
    gl.Disable(gl.BLEND)
    gl.UseProgram(env.sky_prog)
    gl.UniformMatrix4fv(env.loc_inv_vp, 1, false, &inv[0][0])
    gl.BindVertexArray(env.sky_vao)
    gl.DrawArrays(gl.TRIANGLES, 0, 3)
    gl.BindVertexArray(0)
    gl.Enable(gl.DEPTH_TEST)
    gl.DepthMask(true)
    gl.Enable(gl.BLEND)
}
