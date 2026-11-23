package voidengine

// ============================================================================
// gl3d_terrain — draws an R3D_Terrain on the GL backend.
//
// Heights live in an R32F texture; every chunk shares one chunk-local vertex
// grid and the per-LOD index buffers from terrain3d.odin, so a chunk draw is
// one uniform (its grid offset) and one DrawElements. Normals are rebuilt per
// pixel from the height texture, so coarse LODs keep full-resolution shading.
//
// Ground material: up to 4 layers in texture arrays (albedo sRGB with height
// in alpha, OpenGL normal, ORM), mixed by an RGBA splat map with height-aware
// blending and a second, rotated sample scale per layer to hide tiling, plus
// a low-frequency tint. Lighting comes from gl3d_env (shadowed sun, sky
// ambient, height fog).
// ============================================================================

import "core:fmt"
import "core:math/linalg"
import gl "vendor:OpenGL"

GL3D_TERRAIN_VERT :: "#version 330 core\n" + GL3D_PBR_FRAME_BLOCK + `
layout(location=0) in vec3 a_grid; // chunk-local i, j, skirt flag
uniform sampler2D u_height;
uniform ivec2 u_chunk;
uniform vec4  u_hf;       // origin x, origin z, spacing, size
uniform float u_skirt;
uniform mat4  u_vp_override;
uniform int   u_use_override;
out vec3 v_world;
void main() {
    int size = int(u_hf.w);
    ivec2 tc = clamp(u_chunk + ivec2(a_grid.xy), ivec2(0), ivec2(size - 1));
    float h = texelFetch(u_height, tc, 0).r;
    vec3 w = vec3(u_hf.x + float(tc.x) * u_hf.z, h - a_grid.z * u_skirt, u_hf.y + float(tc.y) * u_hf.z);
    v_world = w;
    gl_Position = (u_use_override == 1 ? u_vp_override : u_view_proj) * vec4(w, 1.0);
}
`

GL3D_TERRAIN_FRAG :: "#version 330 core\n" + GL3D_PBR_FRAME_BLOCK + GL3D_ENV_BLOCK + GL3D_ENV_FUNCS + `
in vec3 v_world;
uniform sampler2D      u_height;  // unit 3
uniform sampler2D      u_splat;   // unit 4
uniform sampler2DArray u_albedo;  // unit 0
uniform sampler2DArray u_normal;  // unit 1
uniform sampler2DArray u_orm;     // unit 2
uniform vec4 u_hf;
uniform vec4 u_splat_rect;  // origin x, origin z, 1/width, 1/depth
uniform vec4 u_uv_scale;    // per layer, repeats per metre
uniform vec4 u_layer_tint[4];
uniform int  u_layers;
uniform float u_macro;      // low-frequency tint strength
uniform float u_normal_strength;
out vec4 frag_color;

float hash12(vec2 p) {
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}
float vnoise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(mix(hash12(i), hash12(i + vec2(1, 0)), u.x), mix(hash12(i + vec2(0, 1)), hash12(i + vec2(1, 1)), u.x), u.y);
}

void main() {
    float sp = u_hf.z;
    float size = u_hf.w;
    vec2 g = (v_world.xz - u_hf.xy) / sp;           // grid coordinates
    vec2 tc = (g + 0.5) / size;
    float e = 1.0 / size;
    float hl = texture(u_height, tc - vec2(e, 0)).r;
    float hr = texture(u_height, tc + vec2(e, 0)).r;
    float hd = texture(u_height, tc - vec2(0, e)).r;
    float hu = texture(u_height, tc + vec2(0, e)).r;
    vec3 Ng = normalize(vec3(hl - hr, 2.0 * sp, hd - hu));

    vec4 w = texture(u_splat, (v_world.xz - u_splat_rect.xy) * u_splat_rect.zw);
    w = max(w, vec4(0.0));
    float wsum = w.x + w.y + w.z + w.w;
    w = wsum > 1e-4 ? w / wsum : vec4(1, 0, 0, 0);

    float dist = length(v_world - u_camera_pos.xyz);
    // second sample: rotated and ~3.7x larger, blended in with distance
    const mat2 ROT = mat2(0.8, 0.6, -0.6, 0.8);
    float far_mix = clamp((dist - 25.0) / 150.0, 0.0, 0.5);

    vec4 alb[4];
    vec3 nrm[4];
    vec3 orm[4];
    float hgt[4];
    float best = -1.0;
    for (int i = 0; i < 4; ++i) {
        if (i >= u_layers) { alb[i] = vec4(0); nrm[i] = vec3(0,0,1); orm[i] = vec3(1,1,0); hgt[i] = -1.0; continue; }
        vec2 uv = v_world.xz * u_uv_scale[i];
        vec2 uv2 = ROT * uv * 0.27 + vec2(0.37, 0.71);
        vec4 a1 = texture(u_albedo, vec3(uv, float(i)));
        vec4 a2 = texture(u_albedo, vec3(uv2, float(i)));
        alb[i] = mix(a1, a2, far_mix);
        alb[i].rgb *= u_layer_tint[i].rgb;
        vec3 n1 = texture(u_normal, vec3(uv, float(i))).xyz * 2.0 - 1.0;
        vec3 n2 = texture(u_normal, vec3(uv2, float(i))).xyz * 2.0 - 1.0;
        n2.xy = n2.xy * ROT; // back from the rotated sample's tangent frame
        nrm[i] = normalize(mix(n1, n2, far_mix));
        orm[i] = mix(texture(u_orm, vec3(uv, float(i))).rgb, texture(u_orm, vec3(uv2, float(i))).rgb, far_mix);
        hgt[i] = alb[i].a;
    }
    // height-aware blend: the layer that sticks out wins at the seams
    float m = -1e9;
    float hw[4];
    for (int i = 0; i < 4; ++i) { hw[i] = w[i] > 0.0 ? w[i] + hgt[i] * 0.6 : -1e9; m = max(m, hw[i]); }
    float bw[4];
    float bsum = 0.0;
    for (int i = 0; i < 4; ++i) { bw[i] = max(hw[i] - m + 0.25, 0.0); bsum += bw[i]; }
    vec3 albedo = vec3(0); vec3 tn = vec3(0); vec3 o = vec3(0);
    for (int i = 0; i < 4; ++i) {
        float k = bw[i] / max(bsum, 1e-5);
        albedo += alb[i].rgb * k; tn += nrm[i] * k; o += orm[i] * k;
    }
    // macro variation breaks up large flat areas
    float mn = vnoise(v_world.xz * 0.013) * 0.6 + vnoise(v_world.xz * 0.061) * 0.4;
    albedo *= mix(1.0, 0.78 + 0.44 * mn, u_macro);

    tn = normalize(vec3(tn.xy * u_normal_strength, tn.z));
    vec3 T = normalize(vec3(1, 0, 0) - Ng * Ng.x);
    vec3 B = -normalize(cross(T, Ng)); // image-up (green) is -z: v grows with z, row 0 on top
    vec3 N = normalize(T * tn.x + B * tn.y + Ng * tn.z);
    vec3 V = normalize(u_camera_pos.xyz - v_world);
    vec3 lit = env_light(N, Ng, V, albedo, o.b, clamp(o.g, 0.04, 1.0), o.r, v_world);
    if (any(isnan(lit)) || any(isinf(lit))) lit = vec3(0.0);
    frag_color = vec4(max(lit, vec3(0.0)), 1.0);
}
`

GL3D_TERRAIN_SHADOW_FRAG :: `#version 330 core
void main() {}
`

GL3D_Terrain :: struct {
    prog, shadow_prog: u32,
    vao, vbo:     u32,
    ebo:          [8]u32,
    index_count:  [8]i32,
    lods:         int,
    height_tex:   u32,
    splat_tex:    u32,
    albedo_arr, normal_arr, orm_arr: u32,
    layers:       int,
    uv_scale:     [4]f32,
    layer_tint:   [4][4]f32,
    splat_rect:   [4]f32,
    macro:        f32,
    normal_strength: f32,
    hf_params:    [4]f32,
    // uniform locations: [0] main, [1] shadow
    loc_chunk, loc_hf, loc_skirt, loc_override, loc_use_override: [2]i32,
    loc_splat_rect, loc_uv_scale, loc_layer_tint, loc_layers, loc_macro, loc_nstr: i32,
}

gl3d_terrain_init :: proc(gt: ^GL3D_Terrain, ter: ^R3D_Terrain) -> bool {
    ok: bool
    gt.prog, ok = gl3d_build_program(GL3D_TERRAIN_VERT, GL3D_TERRAIN_FRAG)
    if !ok { fmt.eprintln("gl3d_terrain: program failed"); return false }
    gt.shadow_prog, ok = gl3d_build_program(GL3D_TERRAIN_VERT, GL3D_TERRAIN_SHADOW_FRAG)
    if !ok { fmt.eprintln("gl3d_terrain: shadow program failed"); return false }
    gl3d_env_bind_program(gt.prog)
    gl3d_env_bind_program(gt.shadow_prog)
    progs := [2]u32{gt.prog, gt.shadow_prog}
    for p, i in progs {
        gl.UseProgram(p)
        gl.Uniform1i(gl.GetUniformLocation(p, "u_height"), 3)
        gt.loc_chunk[i] = gl.GetUniformLocation(p, "u_chunk")
        gt.loc_hf[i] = gl.GetUniformLocation(p, "u_hf")
        gt.loc_skirt[i] = gl.GetUniformLocation(p, "u_skirt")
        gt.loc_override[i] = gl.GetUniformLocation(p, "u_vp_override")
        gt.loc_use_override[i] = gl.GetUniformLocation(p, "u_use_override")
    }
    gl.UseProgram(gt.prog)
    gl.Uniform1i(gl.GetUniformLocation(gt.prog, "u_albedo"), 0)
    gl.Uniform1i(gl.GetUniformLocation(gt.prog, "u_normal"), 1)
    gl.Uniform1i(gl.GetUniformLocation(gt.prog, "u_orm"), 2)
    gl.Uniform1i(gl.GetUniformLocation(gt.prog, "u_splat"), 4)
    gt.loc_splat_rect = gl.GetUniformLocation(gt.prog, "u_splat_rect")
    gt.loc_uv_scale = gl.GetUniformLocation(gt.prog, "u_uv_scale")
    gt.loc_layer_tint = gl.GetUniformLocation(gt.prog, "u_layer_tint")
    gt.loc_layers = gl.GetUniformLocation(gt.prog, "u_layers")
    gt.loc_macro = gl.GetUniformLocation(gt.prog, "u_macro")
    gt.loc_nstr = gl.GetUniformLocation(gt.prog, "u_normal_strength")
    gl.UseProgram(0)

    n := ter.chunk_quads
    verts := r3d_terrain_grid_vertices(n, context.temp_allocator)
    gl.GenVertexArrays(1, &gt.vao)
    gl.BindVertexArray(gt.vao)
    gl.GenBuffers(1, &gt.vbo)
    gl.BindBuffer(gl.ARRAY_BUFFER, gt.vbo)
    gl.BufferData(gl.ARRAY_BUFFER, len(verts) * size_of([3]f32), raw_data(verts), gl.STATIC_DRAW)
    gl.EnableVertexAttribArray(0)
    gl.VertexAttribPointer(0, 3, gl.FLOAT, false, size_of([3]f32), 0)
    gt.lods = ter.lod_count
    for lod in 0 ..< ter.lod_count {
        idx := r3d_terrain_lod_indices(n, lod, context.temp_allocator)
        gl.GenBuffers(1, &gt.ebo[lod])
        gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, gt.ebo[lod])
        gl.BufferData(gl.ELEMENT_ARRAY_BUFFER, len(idx) * size_of(u32), raw_data(idx), gl.STATIC_DRAW)
        gt.index_count[lod] = i32(len(idx))
    }
    gl.BindVertexArray(0)

    hf := ter.hf
    gl.GenTextures(1, &gt.height_tex)
    gl.BindTexture(gl.TEXTURE_2D, gt.height_tex)
    gl.PixelStorei(gl.UNPACK_ALIGNMENT, 4)
    gl.TexImage2D(gl.TEXTURE_2D, 0, gl.R32F, i32(hf.size), i32(hf.size), 0, gl.RED, gl.FLOAT, raw_data(hf.heights))
    gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR)
    gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR)
    gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE)
    gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE)
    gl.BindTexture(gl.TEXTURE_2D, 0)
    gt.hf_params = {hf.origin[0], hf.origin[1], hf.spacing, f32(hf.size)}
    gt.macro = 0.5
    gt.normal_strength = 1
    for i in 0 ..< 4 { gt.layer_tint[i] = {1, 1, 1, 1}; gt.uv_scale[i] = 0.2 }
    return true
}

gl3d_terrain_destroy :: proc(gt: ^GL3D_Terrain) {
    gl.DeleteProgram(gt.prog)
    gl.DeleteProgram(gt.shadow_prog)
    gl.DeleteVertexArrays(1, &gt.vao)
    gl.DeleteBuffers(1, &gt.vbo)
    for lod in 0 ..< gt.lods { gl.DeleteBuffers(1, &gt.ebo[lod]) }
    texs := [?]u32{gt.height_tex, gt.splat_tex, gt.albedo_arr, gt.normal_arr, gt.orm_arr}
    for t in texs { if t != 0 { tt := t; gl.DeleteTextures(1, &tt) } }
    gt^ = {}
}

// RGBA8 splat map (one weight per layer) covering the world rectangle
// starting at origin with the given size in metres.
gl3d_terrain_set_splat :: proc(gt: ^GL3D_Terrain, pixels: []u8, w, h: int, origin: [2]f32, size: [2]f32) {
    if gt.splat_tex == 0 { gl.GenTextures(1, &gt.splat_tex) }
    gl.BindTexture(gl.TEXTURE_2D, gt.splat_tex)
    gl.TexImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, i32(w), i32(h), 0, gl.RGBA, gl.UNSIGNED_BYTE, raw_data(pixels))
    gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR_MIPMAP_LINEAR)
    gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR)
    gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE)
    gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE)
    gl.GenerateMipmap(gl.TEXTURE_2D)
    gl.BindTexture(gl.TEXTURE_2D, 0)
    gt.splat_rect = {origin[0], origin[1], 1 / size[0], 1 / size[1]}
}

// Layer images: `layers` RGBA8 images of w x h each, concatenated.
gl3d_upload_texture_array :: proc(pixels: []u8, w, h, layers: int, srgb: bool) -> u32 {
    tex: u32
    gl.GenTextures(1, &tex)
    gl.BindTexture(gl.TEXTURE_2D_ARRAY, tex)
    internal: i32 = srgb ? gl.SRGB8_ALPHA8 : gl.RGBA8
    gl.TexImage3D(gl.TEXTURE_2D_ARRAY, 0, internal, i32(w), i32(h), i32(layers), 0, gl.RGBA, gl.UNSIGNED_BYTE, raw_data(pixels))
    gl.TexParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_MIN_FILTER, gl.LINEAR_MIPMAP_LINEAR)
    gl.TexParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_MAG_FILTER, gl.LINEAR)
    gl.TexParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_WRAP_S, gl.REPEAT)
    gl.TexParameteri(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_WRAP_T, gl.REPEAT)
    gl.TexParameterf(gl.TEXTURE_2D_ARRAY, gl.TEXTURE_MAX_ANISOTROPY, 16)
    gl.GenerateMipmap(gl.TEXTURE_2D_ARRAY)
    gl.BindTexture(gl.TEXTURE_2D_ARRAY, 0)
    return tex
}

// albedo (sRGB, height in A), normal (OpenGL), orm: each `layers` images.
gl3d_terrain_set_layers :: proc(gt: ^GL3D_Terrain, albedo, normal, orm: []u8, w, h, layers: int) {
    gt.layers = min(layers, 4)
    gt.albedo_arr = gl3d_upload_texture_array(albedo, w, h, layers, true)
    gt.normal_arr = gl3d_upload_texture_array(normal, w, h, layers, false)
    gt.orm_arr = gl3d_upload_texture_array(orm, w, h, layers, false)
}

@(private="file")
gl3d_terrain_draw_chunks :: proc(gt: ^GL3D_Terrain, ter: ^R3D_Terrain, which: int) {
    gl.Uniform4f(gt.loc_hf[which], gt.hf_params[0], gt.hf_params[1], gt.hf_params[2], gt.hf_params[3])
    gl.BindVertexArray(gt.vao)
    for &c in ter.chunks {
        if !c.visible { continue }
        gl.Uniform2i(gt.loc_chunk[which], i32(c.i0), i32(c.j0))
        gl.Uniform1f(gt.loc_skirt[which], ter.skirt * f32(int(1) << uint(c.lod)))
        gl.BindBuffer(gl.ELEMENT_ARRAY_BUFFER, gt.ebo[c.lod])
        gl.DrawElements(gl.TRIANGLES, gt.index_count[c.lod], gl.UNSIGNED_INT, nil)
    }
    gl.BindVertexArray(0)
}

// Draw the visible chunks (r3d_terrain_update first). Env must be updated.
gl3d_terrain_draw :: proc(r: ^GL3D_Renderer, gt: ^GL3D_Terrain, ter: ^R3D_Terrain, env: ^GL3D_Env) {
    gl3d_flush(r)
    gl3d_env_upload_frame(r)
    gl3d_env_bind(env)
    gl.UseProgram(gt.prog)
    gl.Uniform1i(gt.loc_use_override[0], 0)
    gl.Uniform4f(gt.loc_splat_rect, gt.splat_rect[0], gt.splat_rect[1], gt.splat_rect[2], gt.splat_rect[3])
    gl.Uniform4f(gt.loc_uv_scale, gt.uv_scale[0], gt.uv_scale[1], gt.uv_scale[2], gt.uv_scale[3])
    gl.Uniform4fv(gt.loc_layer_tint, 4, &gt.layer_tint[0][0])
    gl.Uniform1i(gt.loc_layers, i32(gt.layers))
    gl.Uniform1f(gt.loc_macro, gt.macro)
    gl.Uniform1f(gt.loc_nstr, gt.normal_strength)
    gl.ActiveTexture(gl.TEXTURE0); gl.BindTexture(gl.TEXTURE_2D_ARRAY, gt.albedo_arr)
    gl.ActiveTexture(gl.TEXTURE1); gl.BindTexture(gl.TEXTURE_2D_ARRAY, gt.normal_arr)
    gl.ActiveTexture(gl.TEXTURE2); gl.BindTexture(gl.TEXTURE_2D_ARRAY, gt.orm_arr)
    gl.ActiveTexture(gl.TEXTURE3); gl.BindTexture(gl.TEXTURE_2D, gt.height_tex)
    gl.ActiveTexture(gl.TEXTURE4); gl.BindTexture(gl.TEXTURE_2D, gt.splat_tex)
    gl.ActiveTexture(gl.TEXTURE0)
    gl.Enable(gl.CULL_FACE)
    gl.CullFace(gl.BACK)
    gl.Disable(gl.BLEND)
    gl3d_terrain_draw_chunks(gt, ter, 0)
    gl.Disable(gl.CULL_FACE)
    gl.Enable(gl.BLEND)
}

// Shadow casters for one cascade (inside gl3d_env_shadow_begin/_end). The
// terrain's chunk visibility/LOD is recomputed for the light's box, so call
// r3d_terrain_update again for the camera before gl3d_terrain_draw.
gl3d_terrain_draw_shadow :: proc(gt: ^GL3D_Terrain, ter: ^R3D_Terrain, light_vp: linalg.Matrix4f32, eye: linalg.Vector3f32) {
    f := r3d_frustum_from_matrix(light_vp, true)
    r3d_terrain_update(ter, eye, &f, 1)
    gl.UseProgram(gt.shadow_prog)
    vp := light_vp
    gl.UniformMatrix4fv(gt.loc_override[1], 1, false, &vp[0][0])
    gl.Uniform1i(gt.loc_use_override[1], 1)
    gl.ActiveTexture(gl.TEXTURE3); gl.BindTexture(gl.TEXTURE_2D, gt.height_tex)
    gl.ActiveTexture(gl.TEXTURE0)
    gl3d_terrain_draw_chunks(gt, ter, 1)
}
