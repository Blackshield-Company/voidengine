package voidengine

// ============================================================================
// gl3d_pbr — PBR-lite + GPU skinning for R3D_Model on the OpenGL 3.3 backend.
//
// Additive to gl3d.odin: it reuses the renderer's Frame UBO (camera, sun,
// point lights, fog, sky), its sun shadow map and its HDR target + post
// chain, and brings its own programs, a joint-matrix UBO and model buffers.
//
// Shading: metallic-roughness GGX (Trowbridge-Reitz D, Smith-Schlick G,
// Schlick F), tangent-space normal maps, ORM / glTF metallic-roughness
// (G = roughness, B = metallic) and separate occlusion (R), emissive,
// alpha mask, double-sided. Color maps are sampled from sRGB textures, data
// maps from linear ones; lighting runs in linear HDR and the composite pass
// tonemaps + applies gamma.
//
// Frame order:
//   gl3d_shadow_pass_begin(r, center)
//   gl3d_pbr_shadow_begin(r, &pbr); gl3d_pbr_draw_model_shadow(...) (xN)
//   gl3d_shadow_pass_end(r)
//   gl3d_begin_frame(r); gl3d_pbr_draw_model(...) (xN); gl3d_end_frame(r)
// ============================================================================

import "core:fmt"
import "core:strings"
import "core:math/linalg"
import gl "vendor:OpenGL"

GL3D_PBR_MAX_JOINTS :: 128

GL3D_PBR_FRAME_BLOCK :: `
layout(std140) uniform Frame {
    mat4 u_view_proj;
    mat4 u_light_view_proj;
    vec4 u_camera_pos;
    vec4 u_ambient;
    vec4 u_light_pos[16];
    vec4 u_light_color[16];
    vec4 u_meta;
    vec4 u_sun_dir;
    vec4 u_sun_color;
    vec4 u_shadow_params;
    vec4 u_fog_color;
    vec4 u_fog_params;
    vec4 u_sky_zenith;
    vec4 u_sky_horizon;
};
`

GL3D_PBR_SKIN_BLOCK :: `
layout(location=0) in vec3  a_pos;
layout(location=1) in vec3  a_normal;
layout(location=2) in vec2  a_uv;
layout(location=3) in vec4  a_tangent;
layout(location=4) in uvec4 a_joints;
layout(location=5) in vec4  a_weights;
layout(std140) uniform Joints { mat4 u_joint[128]; };
uniform mat4 u_model;
uniform int  u_skinned;
mat4 skin_matrix() {
    if (u_skinned == 0) return u_model;
    mat4 s = a_weights.x * u_joint[a_joints.x]
           + a_weights.y * u_joint[a_joints.y]
           + a_weights.z * u_joint[a_joints.z]
           + a_weights.w * u_joint[a_joints.w];
    return u_model * s;
}
`

GL3D_PBR_VERT :: "#version 330 core\n" + GL3D_PBR_FRAME_BLOCK + GL3D_PBR_SKIN_BLOCK + `
uniform vec2 u_uv_scale;
out vec3 v_world;
out vec3 v_normal;
out vec4 v_tangent;
out vec2 v_uv;
void main() {
    mat4 m = skin_matrix();
    vec4 world = m * vec4(a_pos, 1.0);
    mat3 m3 = mat3(m);
    v_world   = world.xyz;
    v_normal  = m3 * a_normal;
    v_tangent = vec4(m3 * a_tangent.xyz, a_tangent.w);
    v_uv      = a_uv * u_uv_scale;
    gl_Position = u_view_proj * world;
}
`

GL3D_PBR_FRAG :: "#version 330 core\n" + GL3D_PBR_FRAME_BLOCK + `
in vec3 v_world;
in vec3 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

uniform sampler2D u_base_tex;   // unit 0, sRGB
uniform sampler2D u_mr_tex;     // unit 1, linear (G rough, B metal; ORM R = AO)
uniform sampler2D u_shadow_map; // unit 2
uniform sampler2D u_normal_tex; // unit 3, linear
uniform sampler2D u_occ_tex;    // unit 4, linear (R)
uniform sampler2D u_emis_tex;   // unit 5, sRGB

uniform vec4  u_base_color;
uniform vec4  u_mat;       // x metallic, y roughness, z normal scale, w occlusion strength
uniform vec3  u_emissive;
uniform float u_alpha_cutoff; // < 0: opaque
uniform int   u_double_sided;

out vec4 frag_color;

const float PI = 3.14159265;

float shadow_factor(vec3 world, float ndl) {
    if (u_shadow_params.x < 0.5) return 1.0;
    vec4 lp = u_light_view_proj * vec4(world, 1.0);
    vec3 proj = lp.xyz / lp.w;
    vec2 suv = proj.xy * 0.5 + 0.5;
    if (suv.x <= 0.0 || suv.x >= 1.0 || suv.y <= 0.0 || suv.y >= 1.0) return 1.0;
    float current = u_shadow_params.w > 0.5 ? proj.z * 0.5 + 0.5 : proj.z;
    if (current >= 1.0) return 1.0;
    float texel = u_shadow_params.z;
    float bias  = u_shadow_params.y * (1.0 + 3.0 * (1.0 - clamp(ndl, 0.0, 1.0)));
    float sum = 0.0;
    for (int x = -2; x <= 2; ++x) {
        for (int y = -2; y <= 2; ++y) {
            float d = texture(u_shadow_map, suv + vec2(x, y) * texel * 0.75).r;
            sum += (current - bias > d) ? 0.0 : 1.0;
        }
    }
    return sum / 25.0;
}

float D_ggx(float ndh, float a) {
    float a2 = a * a;
    float d = ndh * ndh * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d + 1e-7);
}
float G_smith(float ndv, float ndl, float rough) {
    float k = (rough + 1.0) * (rough + 1.0) / 8.0;
    float gv = ndv / (ndv * (1.0 - k) + k);
    float gl_ = ndl / (ndl * (1.0 - k) + k);
    return gv * gl_;
}
vec3 F_schlick(float c, vec3 f0) {
    return f0 + (1.0 - f0) * pow(1.0 - c, 5.0);
}
vec3 F_schlick_rough(float c, vec3 f0, float rough) {
    return f0 + (max(vec3(1.0 - rough), f0) - f0) * pow(1.0 - c, 5.0);
}

// Light colors are treated as irradiance scale (x PI), matching the
// Blinn-Phong paths' brightness so scenes can mix both shaders.
vec3 shade(vec3 N, vec3 V, vec3 L, vec3 albedo, float metal, float rough, vec3 f0) {
    float ndl = max(dot(N, L), 0.0);
    if (ndl <= 0.0) return vec3(0.0);
    vec3  H   = normalize(L + V);
    float ndv = max(dot(N, V), 1e-4);
    float ndh = max(dot(N, H), 0.0);
    float vdh = max(dot(V, H), 0.0);
    float a   = rough * rough;
    vec3  F   = F_schlick(vdh, f0);
    vec3  spec = D_ggx(ndh, a) * G_smith(ndv, ndl, rough) * F / (4.0 * ndv * ndl + 1e-4);
    vec3  kd  = (1.0 - F) * (1.0 - metal);
    return (kd * albedo / PI + spec) * ndl * PI;
}

void main() {
    vec4 base = u_base_color * texture(u_base_tex, v_uv);
    if (u_alpha_cutoff >= 0.0 && base.a < u_alpha_cutoff) discard;

    vec4  mr    = texture(u_mr_tex, v_uv);
    float rough = clamp(u_mat.y * mr.g, 0.045, 1.0);
    float metal = clamp(u_mat.x * mr.b, 0.0, 1.0);
    float ao    = mix(1.0, texture(u_occ_tex, v_uv).r, u_mat.w);

    vec3 N = normalize(v_normal);
    vec3 T = v_tangent.xyz;
    if (u_double_sided == 1 && !gl_FrontFacing) { N = -N; T = -T; }
    T = normalize(T - N * dot(N, T));
    vec3 B = cross(N, T) * (v_tangent.w < 0.0 ? -1.0 : 1.0);
    vec3 tn = texture(u_normal_tex, v_uv).xyz * 2.0 - 1.0;
    tn.xy *= u_mat.z;
    N = normalize(mat3(T, B, N) * tn);

    vec3 V = normalize(u_camera_pos.xyz - v_world);
    vec3 albedo = base.rgb;
    vec3 f0 = mix(vec3(0.04), albedo, metal);

    vec3 lit = vec3(0.0);
    int num_lights = int(u_meta.x);
    for (int i = 0; i < num_lights; ++i) {
        vec3  lv     = u_light_pos[i].xyz - v_world;
        float dist   = length(lv);
        vec3  L      = lv / max(dist, 1e-4);
        float radius = max(u_light_pos[i].w, 1e-4);
        float att    = clamp(1.0 - dist / radius, 0.0, 1.0);
        att *= att;
        lit += shade(N, V, L, albedo, metal, rough, f0) * u_light_color[i].rgb * att;
    }

    vec3 sunL = normalize(-u_sun_dir.xyz);
    float sh = shadow_factor(v_world, dot(normalize(v_normal), sunL));
    lit += shade(N, V, sunL, albedo, metal, rough, f0) * u_sun_color.rgb * sh;

    // ambient: sky/ground hemisphere for diffuse, sky gradient for specular
    float hemi = N.y * 0.5 + 0.5;
    vec3 amb_d = u_ambient.rgb * mix(0.45, 1.0, hemi);
    vec3 R = reflect(-V, N);
    vec3 env = mix(u_sky_horizon.rgb, u_sky_zenith.rgb, clamp(R.y, 0.0, 1.0));
    env = mix(amb_d, env, 0.5);
    vec3 Fa = F_schlick_rough(max(dot(N, V), 0.0), f0, rough);
    vec3 kd = (1.0 - Fa) * (1.0 - metal);
    lit += (kd * albedo * amb_d + Fa * env * (1.0 - rough * 0.7)) * ao;

    lit += u_emissive * texture(u_emis_tex, v_uv).rgb;

    // squared-distance fog, tinted toward the sun (in-scatter)
    vec3  to_frag = v_world - u_camera_pos.xyz;
    float fog_dist = length(to_frag);
    float fog_f = 1.0 - exp(-u_fog_params.x * u_fog_params.x * fog_dist * fog_dist);
    float sun_amt = pow(max(dot(to_frag / max(fog_dist, 1e-4), sunL), 0.0), 8.0) * u_fog_params.y;
    vec3  fog_col = u_fog_color.rgb + u_sun_color.rgb * 0.08 * sun_amt;
    lit = mix(lit, fog_col, clamp(fog_f, 0.0, 1.0));

    frag_color = vec4(lit, base.a);
}
`

GL3D_PBR_SHADOW_VERT :: "#version 330 core\n" + GL3D_PBR_SKIN_BLOCK + `
uniform mat4 u_light_vp;
void main() {
    gl_Position = u_light_vp * skin_matrix() * vec4(a_pos, 1.0);
}
`

GL3D_PBR_SHADOW_FRAG :: `#version 330 core
void main() {}
`

GL3D_Pbr :: struct {
    prog:            u32,
    shadow_prog:     u32,
    joint_ubo:       u32,
    white_tex:       u32,
    flat_normal_tex: u32,
    // uniform locations (main program)
    loc_model, loc_skinned, loc_uv_scale, loc_base_color, loc_mat, loc_emissive: i32,
    loc_alpha_cutoff, loc_double_sided: i32,
    // shadow program
    sloc_model, sloc_skinned, sloc_light_vp: i32,
}

gl3d_pbr_init :: proc() -> (p: GL3D_Pbr, ok: bool) {
    p.prog, ok = gl3d_build_program(GL3D_PBR_VERT, GL3D_PBR_FRAG)
    if !ok { fmt.eprintln("gl3d_pbr: program failed"); return }
    p.shadow_prog, ok = gl3d_build_program(GL3D_PBR_SHADOW_VERT, GL3D_PBR_SHADOW_FRAG)
    if !ok { fmt.eprintln("gl3d_pbr: shadow program failed"); return }

    bind_blocks :: proc(prog: u32) {
        fi := gl.GetUniformBlockIndex(prog, "Frame")
        if fi != gl.INVALID_INDEX { gl.UniformBlockBinding(prog, fi, 0) }
        ji := gl.GetUniformBlockIndex(prog, "Joints")
        if ji != gl.INVALID_INDEX { gl.UniformBlockBinding(prog, ji, 1) }
    }
    bind_blocks(p.prog)
    bind_blocks(p.shadow_prog)

    gl.UseProgram(p.prog)
    units := [?]struct{name: cstring, unit: i32}{
        {"u_base_tex", 0}, {"u_mr_tex", 1}, {"u_shadow_map", 2},
        {"u_normal_tex", 3}, {"u_occ_tex", 4}, {"u_emis_tex", 5},
    }
    for u in units { gl.Uniform1i(gl.GetUniformLocation(p.prog, u.name), u.unit) }
    p.loc_model        = gl.GetUniformLocation(p.prog, "u_model")
    p.loc_skinned      = gl.GetUniformLocation(p.prog, "u_skinned")
    p.loc_uv_scale     = gl.GetUniformLocation(p.prog, "u_uv_scale")
    p.loc_base_color   = gl.GetUniformLocation(p.prog, "u_base_color")
    p.loc_mat          = gl.GetUniformLocation(p.prog, "u_mat")
    p.loc_emissive     = gl.GetUniformLocation(p.prog, "u_emissive")
    p.loc_alpha_cutoff = gl.GetUniformLocation(p.prog, "u_alpha_cutoff")
    p.loc_double_sided = gl.GetUniformLocation(p.prog, "u_double_sided")
    p.sloc_model    = gl.GetUniformLocation(p.shadow_prog, "u_model")
    p.sloc_skinned  = gl.GetUniformLocation(p.shadow_prog, "u_skinned")
    p.sloc_light_vp = gl.GetUniformLocation(p.shadow_prog, "u_light_vp")
    gl.UseProgram(0)

    gl.GenBuffers(1, &p.joint_ubo)
    gl.BindBuffer(gl.UNIFORM_BUFFER, p.joint_ubo)
    gl.BufferData(gl.UNIFORM_BUFFER, GL3D_PBR_MAX_JOINTS * size_of(linalg.Matrix4f32), nil, gl.DYNAMIC_DRAW)
    gl.BindBuffer(gl.UNIFORM_BUFFER, 0)

    white := [4]u8{255, 255, 255, 255}
    p.white_tex = gl3d_upload_texture(white[:], 1, 1, true)
    flat := [4]u8{128, 128, 255, 255}
    p.flat_normal_tex = gl3d_upload_texture(flat[:], 1, 1, true)
    ok = true
    return
}

gl3d_pbr_destroy :: proc(p: ^GL3D_Pbr) {
    gl.DeleteProgram(p.prog)
    gl.DeleteProgram(p.shadow_prog)
    gl.DeleteBuffers(1, &p.joint_ubo)
    gl3d_destroy_texture(p.white_tex)
    gl3d_destroy_texture(p.flat_normal_tex)
    p^ = {}
}

GL3D_Model_Prim :: struct {
    vao, vbo, ebo: u32,
    index_count:   i32,
}

GL3D_Model :: struct {
    src:        ^R3D_Model,
    prims:      [dynamic]GL3D_Model_Prim,
    hidden:     [dynamic]bool, // per primitive
    tex_srgb:   [dynamic]u32,  // per image (0 = not uploaded in this space)
    tex_linear: [dynamic]u32,
}

// Upload all primitives and the images their materials use (color slots as
// sRGB, data slots as linear). The R3D_Model must outlive the GL3D_Model.
gl3d_model_upload :: proc(m: ^R3D_Model) -> (gm: GL3D_Model) {
    gm.src = m
    resize(&gm.tex_srgb, len(m.images))
    resize(&gm.tex_linear, len(m.images))
    up :: proc(m: ^R3D_Model, list: ^[dynamic]u32, idx: int, srgb: bool) {
        if idx < 0 || idx >= len(m.images) || list[idx] != 0 { return }
        im := &m.images[idx]
        if im.pixels == nil { return }
        list[idx] = gl3d_upload_texture(im.pixels, i32(im.width), i32(im.height), true, false, srgb)
    }
    for &mt in m.materials {
        up(m, &gm.tex_srgb, mt.base_color_tex, true)
        up(m, &gm.tex_srgb, mt.emissive_tex, true)
        up(m, &gm.tex_linear, mt.metal_rough_tex, false)
        up(m, &gm.tex_linear, mt.normal_tex, false)
        up(m, &gm.tex_linear, mt.occlusion_tex, false)
    }
    for &p in m.prims {
        g: GL3D_Model_Prim
        g.index_count = i32(len(p.indices))
        gl.GenVertexArrays(1, &g.vao)
        gl.BindVertexArray(g.vao)
        gl.GenBuffers(1, &g.vbo)
        gl.BindBuffer(gl.ARRAY_BUFFER, g.vbo)
        gl.BufferData(gl.ARRAY_BUFFER, len(p.vertices) * R3D_SKIN_VERTEX_STRIDE, raw_data(p.vertices), gl.STATIC_DRAW)
        gl.GenBuffers(1, &g.ebo)
        gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, g.ebo)
        gl.BufferData(gl.ELEMENT_ARRAY_BUFFER, len(p.indices) * size_of(u32), raw_data(p.indices), gl.STATIC_DRAW)
        S :: R3D_SKIN_VERTEX_STRIDE
        gl.EnableVertexAttribArray(0)
        gl.VertexAttribPointer(0, 3, gl.FLOAT, false, S, offset_of(R3D_Skin_Vertex, pos))
        gl.EnableVertexAttribArray(1)
        gl.VertexAttribPointer(1, 3, gl.FLOAT, false, S, offset_of(R3D_Skin_Vertex, normal))
        gl.EnableVertexAttribArray(2)
        gl.VertexAttribPointer(2, 2, gl.FLOAT, false, S, offset_of(R3D_Skin_Vertex, uv))
        gl.EnableVertexAttribArray(3)
        gl.VertexAttribPointer(3, 4, gl.FLOAT, false, S, offset_of(R3D_Skin_Vertex, tangent))
        gl.EnableVertexAttribArray(4)
        gl.VertexAttribIPointer(4, 4, gl.UNSIGNED_SHORT, S, offset_of(R3D_Skin_Vertex, joints))
        gl.EnableVertexAttribArray(5)
        gl.VertexAttribPointer(5, 4, gl.FLOAT, false, S, offset_of(R3D_Skin_Vertex, weights))
        gl.BindVertexArray(0)
        append(&gm.prims, g)
        append(&gm.hidden, false)
    }
    return
}

gl3d_model_destroy :: proc(gm: ^GL3D_Model) {
    for &g in gm.prims {
        gl.DeleteBuffers(1, &g.vbo)
        gl.DeleteBuffers(1, &g.ebo)
        gl.DeleteVertexArrays(1, &g.vao)
    }
    for t in gm.tex_srgb { if t != 0 { gl3d_destroy_texture(t) } }
    for t in gm.tex_linear { if t != 0 { gl3d_destroy_texture(t) } }
    delete(gm.prims)
    delete(gm.hidden)
    delete(gm.tex_srgb)
    delete(gm.tex_linear)
    gm^ = {}
}

// Show or hide every primitive whose node name equals `name` (or starts
// with it when prefix = true). Returns how many matched.
gl3d_model_set_visible :: proc(gm: ^GL3D_Model, name: string, visible: bool, prefix := false) -> int {
    n := 0
    for &p, i in gm.src.prims {
        if p.name == name || (prefix && strings.has_prefix(p.name, name)) {
            gm.hidden[i] = !visible
            n += 1
        }
    }
    return n
}

@(private)
gl3d_pbr_prim_matrix :: proc(pbr: ^GL3D_Pbr, gm: ^GL3D_Model, pi: int, pose: ^R3D_Model_Pose, model: linalg.Matrix4f32, skin_loc: i32, model_loc: i32) {
    p := &gm.src.prims[pi]
    if p.skin >= 0 && pose != nil && p.skin < len(pose.joints) {
        mats := pose.joints[p.skin][:]
        n := min(len(mats), GL3D_PBR_MAX_JOINTS)
        gl.BindBufferBase(gl.UNIFORM_BUFFER, 1, pbr.joint_ubo)
        gl.BindBuffer(gl.UNIFORM_BUFFER, pbr.joint_ubo)
        gl.BufferSubData(gl.UNIFORM_BUFFER, 0, n * size_of(linalg.Matrix4f32), raw_data(mats))
        m := model
        gl.UniformMatrix4fv(model_loc, 1, false, &m[0][0])
        gl.Uniform1i(skin_loc, 1)
    } else {
        m := model
        if pose != nil && p.node >= 0 && p.node < len(pose.global) {
            m = model * pose.global[p.node]
        }
        gl.UniformMatrix4fv(model_loc, 1, false, &m[0][0])
        gl.Uniform1i(skin_loc, 0)
    }
}

// Draw a model (skinned and static primitives) at `model`. pose may be nil
// for static models (node transforms are then ignored).
gl3d_pbr_draw_model :: proc(r: ^GL3D_Renderer, pbr: ^GL3D_Pbr, gm: ^GL3D_Model, pose: ^R3D_Model_Pose, model: linalg.Matrix4f32) {
    gl3d_flush(r) // keep sprite batches in submission order
    gl.UseProgram(pbr.prog)
    aspect := f32(r.width) / f32(max(r.height, 1))
    uniforms := r3d_make_frame_uniforms(&r.camera, aspect, false, r.ambient, r.lights[:], &r.sun, &r.atmosphere)
    gl.BindBuffer(gl.UNIFORM_BUFFER, r.ubo)
    gl.BufferSubData(gl.UNIFORM_BUFFER, 0, size_of(R3D_Frame_Uniforms), &uniforms)
    gl.BindBufferBase(gl.UNIFORM_BUFFER, 0, r.ubo)

    gl.ActiveTexture(gl.TEXTURE2)
    gl.BindTexture(gl.TEXTURE_2D, r.shadow_tex)

    tex :: proc(list: []u32, idx: int, fallback: u32) -> u32 {
        if idx >= 0 && idx < len(list) && list[idx] != 0 { return list[idx] }
        return fallback
    }
    def := r3d_default_material()
    for pi in 0 ..< len(gm.prims) {
        if gm.hidden[pi] || gm.prims[pi].index_count == 0 { continue }
        p := &gm.src.prims[pi]
        mt := &def
        if p.material >= 0 && p.material < len(gm.src.materials) { mt = &gm.src.materials[p.material] }

        gl3d_pbr_prim_matrix(pbr, gm, pi, pose, model, pbr.loc_skinned, pbr.loc_model)
        gl.Uniform2f(pbr.loc_uv_scale, mt.uv_scale.x, mt.uv_scale.y)
        gl.Uniform4f(pbr.loc_base_color, mt.base_color.x, mt.base_color.y, mt.base_color.z, mt.base_color.w)
        gl.Uniform4f(pbr.loc_mat, mt.metallic, mt.roughness, mt.normal_scale, mt.occlusion_tex >= 0 ? mt.occlusion_strength : 0)
        gl.Uniform3f(pbr.loc_emissive, mt.emissive.x, mt.emissive.y, mt.emissive.z)
        gl.Uniform1f(pbr.loc_alpha_cutoff, mt.alpha_mode == .Mask ? mt.alpha_cutoff : -1)
        gl.Uniform1i(pbr.loc_double_sided, mt.double_sided ? 1 : 0)

        gl.ActiveTexture(gl.TEXTURE0)
        gl.BindTexture(gl.TEXTURE_2D, tex(gm.tex_srgb[:], mt.base_color_tex, pbr.white_tex))
        gl.ActiveTexture(gl.TEXTURE1)
        gl.BindTexture(gl.TEXTURE_2D, tex(gm.tex_linear[:], mt.metal_rough_tex, pbr.white_tex))
        gl.ActiveTexture(gl.TEXTURE3)
        gl.BindTexture(gl.TEXTURE_2D, tex(gm.tex_linear[:], mt.normal_tex, pbr.flat_normal_tex))
        gl.ActiveTexture(gl.TEXTURE4)
        gl.BindTexture(gl.TEXTURE_2D, tex(gm.tex_linear[:], mt.occlusion_tex, pbr.white_tex))
        gl.ActiveTexture(gl.TEXTURE5)
        gl.BindTexture(gl.TEXTURE_2D, tex(gm.tex_srgb[:], mt.emissive_tex, pbr.white_tex))
        gl.ActiveTexture(gl.TEXTURE0)

        if mt.double_sided { gl.Disable(gl.CULL_FACE) } else { gl.Enable(gl.CULL_FACE); gl.CullFace(gl.BACK) }
        gl.BindVertexArray(gm.prims[pi].vao)
        gl.DrawElements(gl.TRIANGLES, gm.prims[pi].index_count, gl.UNSIGNED_INT, nil)
    }
    gl.BindVertexArray(0)
    gl.Disable(gl.CULL_FACE) // gl3d's sprite paths expect culling off
}

// Switch the shadow pass to the skinning-aware caster program. Call after
// gl3d_shadow_pass_begin (which binds the shadow target and light matrix).
gl3d_pbr_shadow_begin :: proc(r: ^GL3D_Renderer, pbr: ^GL3D_Pbr) {
    if !r.sun.enabled || !r.sun.cast_shadows { return }
    gl.UseProgram(pbr.shadow_prog)
    lvp := r.sun.light_view_proj
    gl.UniformMatrix4fv(pbr.sloc_light_vp, 1, false, &lvp[0][0])
}

gl3d_pbr_draw_model_shadow :: proc(r: ^GL3D_Renderer, pbr: ^GL3D_Pbr, gm: ^GL3D_Model, pose: ^R3D_Model_Pose, model: linalg.Matrix4f32) {
    if !r.sun.enabled || !r.sun.cast_shadows { return }
    for pi in 0 ..< len(gm.prims) {
        if gm.hidden[pi] || gm.prims[pi].index_count == 0 { continue }
        gl3d_pbr_prim_matrix(pbr, gm, pi, pose, model, pbr.sloc_skinned, pbr.sloc_model)
        gl.BindVertexArray(gm.prims[pi].vao)
        gl.DrawElements(gl.TRIANGLES, gm.prims[pi].index_count, gl.UNSIGNED_INT, nil)
    }
    gl.BindVertexArray(0)
}
