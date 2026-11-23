package voidengine_test

// Tests for r3d_model.odin: a hand-built skinned .glb (2-joint skin, an
// animation with a STEP and a LINEAR track, a PBR material whose texture
// slots all point at an embedded PNG, a static prop parented to a joint),
// plus the real CC0 knight from the templar3d demo when it is present.

import "core:fmt"
import "core:math"
import "core:os"
import "core:encoding/base64"
import "core:testing"
import "core:math/linalg"
import engine "../src/core"

@(private)
SKIN_PNG := [?]u8{137, 80, 78, 71, 13, 10, 26, 10, 0, 0, 0, 13, 73, 72, 68, 82, 0, 0, 0, 2, 0, 0, 0, 2, 8, 6, 0, 0, 0, 114, 182, 13, 36, 0, 0, 0, 25, 73, 68, 65, 84, 120, 218, 5, 193, 1, 13, 0, 0, 8, 192, 32, 220, 44, 110, 242, 11, 34, 105, 71, 226, 30, 63, 128, 6, 129, 163, 85, 254, 240, 0, 0, 0, 0, 73, 69, 78, 68, 174, 66, 96, 130}

@(private)
build_skinned_glb :: proc() -> []u8 {
    bin: [dynamic]u8
    defer delete(bin)
    f :: proc(b: ^[dynamic]u8, vs: ..f32) { for v in vs { glb_f32(b, v) } }
    // 0: positions (quad in XY, 1 x 2 m)
    f(&bin, 0, 0, 0, 1, 0, 0, 1, 2, 0, 0, 2, 0)
    // 48: normals
    f(&bin, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1)
    // 96: uvs (v grows downward, as in glTF)
    f(&bin, 0, 1, 1, 1, 1, 0, 0, 0)
    // 128: joints (u8 x4)
    append(&bin, 0, 0, 0, 0,  0, 0, 0, 0,  1, 0, 0, 0,  1, 0, 0, 0)
    // 144: weights (v2 un-normalised: 0.6/0.2 -> 0.75/0.25)
    f(&bin, 1, 0, 0, 0, 1, 0, 0, 0, 0.6, 0.2, 0, 0, 1, 0, 0, 0)
    // 208: indices u16
    for i in ([]u16{0, 1, 2, 2, 3, 0}) { glb_u16(&bin, i) }
    pad4(&bin, 0) // -> 220
    // 220: inverse bind matrices (column-major): identity, translate(0,-1,0)
    f(&bin, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1)
    f(&bin, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, -1, 0, 1)
    // 348: key times
    f(&bin, 0, 1)
    // 356: rotation keys (identity -> 90 deg about Z)
    s := math.sqrt(f32(0.5))
    f(&bin, 0, 0, 0, 1, 0, 0, s, s)
    // 388: translation keys (STEP)
    f(&bin, 0, 0, 0, 5, 0, 0)
    // 412: png
    for b in SKIN_PNG { append(&bin, b) }
    pad4(&bin, 0)

    json := fmt.tprintf(`{{"asset":{{"version":"2.0"}},"scene":0,"scenes":[{{"nodes":[2,1]}}],` +
        `"nodes":[` +
        `{{"name":"Bone1","translation":[0,1,0],"children":[3]}},` +
        `{{"name":"SkinnedMesh","mesh":0,"skin":0}},` +
        `{{"name":"Bone0","children":[0]}},` +
        `{{"name":"Prop","mesh":1,"translation":[0,0.5,0]}}],` +
        `"skins":[{{"joints":[2,0],"inverseBindMatrices":6}}],` +
        `"meshes":[{{"name":"Body","primitives":[{{"attributes":{{"POSITION":0,"NORMAL":1,"TEXCOORD_0":2,"JOINTS_0":3,"WEIGHTS_0":4}},"indices":5,"material":0}}]}},` +
        `{{"name":"PropMesh","primitives":[{{"attributes":{{"POSITION":0,"NORMAL":1,"TEXCOORD_0":2}},"indices":5}}]}}],` +
        `"materials":[{{"name":"Mat","pbrMetallicRoughness":{{"baseColorFactor":[1,0.5,0.25,1],"baseColorTexture":{{"index":0}},` +
        `"metallicRoughnessTexture":{{"index":0}},"metallicFactor":0.3,"roughnessFactor":0.7}},` +
        `"normalTexture":{{"index":0,"scale":0.5}},"occlusionTexture":{{"index":0,"strength":0.8}},` +
        `"emissiveFactor":[1,1,0],"alphaMode":"MASK","alphaCutoff":0.3,"doubleSided":true}}],` +
        `"textures":[{{"source":0}}],"images":[{{"name":"tiny","bufferView":10,"mimeType":"image/png"}}],` +
        `"animations":[{{"name":"Bend","samplers":[{{"input":7,"output":8}},{{"input":7,"output":9,"interpolation":"STEP"}}],` +
        `"channels":[{{"sampler":0,"target":{{"node":0,"path":"rotation"}}}},{{"sampler":1,"target":{{"node":2,"path":"translation"}}}}]}}],` +
        `"accessors":[` +
        `{{"bufferView":0,"componentType":5126,"count":4,"type":"VEC3","min":[0,0,0],"max":[1,2,0]}},` +
        `{{"bufferView":1,"componentType":5126,"count":4,"type":"VEC3"}},` +
        `{{"bufferView":2,"componentType":5126,"count":4,"type":"VEC2"}},` +
        `{{"bufferView":3,"componentType":5121,"count":4,"type":"VEC4"}},` +
        `{{"bufferView":4,"componentType":5126,"count":4,"type":"VEC4"}},` +
        `{{"bufferView":5,"componentType":5123,"count":6,"type":"SCALAR"}},` +
        `{{"bufferView":6,"componentType":5126,"count":2,"type":"MAT4"}},` +
        `{{"bufferView":7,"componentType":5126,"count":2,"type":"SCALAR","min":[0],"max":[1]}},` +
        `{{"bufferView":8,"componentType":5126,"count":2,"type":"VEC4"}},` +
        `{{"bufferView":9,"componentType":5126,"count":2,"type":"VEC3"}}],` +
        `"bufferViews":[` +
        `{{"buffer":0,"byteOffset":0,"byteLength":48}},{{"buffer":0,"byteOffset":48,"byteLength":48}},` +
        `{{"buffer":0,"byteOffset":96,"byteLength":32}},{{"buffer":0,"byteOffset":128,"byteLength":16}},` +
        `{{"buffer":0,"byteOffset":144,"byteLength":64}},{{"buffer":0,"byteOffset":208,"byteLength":12}},` +
        `{{"buffer":0,"byteOffset":220,"byteLength":128}},{{"buffer":0,"byteOffset":348,"byteLength":8}},` +
        `{{"buffer":0,"byteOffset":356,"byteLength":32}},{{"buffer":0,"byteOffset":388,"byteLength":24}},` +
        `{{"buffer":0,"byteOffset":412,"byteLength":%d}}],` +
        `"buffers":[{{"byteLength":%d}}]}}`, len(SKIN_PNG), len(bin))

    out: [dynamic]u8
    jl := (len(json) + 3) &~ 3
    total := 12 + 8 + jl + 8 + len(bin)
    glb_u32(&out, 0x46546C67); glb_u32(&out, 2); glb_u32(&out, u32(total))
    glb_u32(&out, u32(jl)); glb_u32(&out, 0x4E4F534A)
    glb_bytes(&out, json)
    pad4(&out, ' ')
    glb_u32(&out, u32(len(bin))); glb_u32(&out, 0x004E4942)
    for b in bin { append(&out, b) }
    return out[:]
}

@(private)
load_skinned_fixture :: proc(t: ^testing.T) -> (engine.R3D_Model, bool) {
    glb := build_skinned_glb()
    defer delete(glb)
    path := "/tmp/voidengine_test_skinned.glb"
    if os.write_entire_file(path, glb) != nil { return {}, false }
    defer os.remove(path)
    return engine.r3d_model_load(path)
}

@(test)
test_model_load_structure :: proc(t: ^testing.T) {
    m, ok := load_skinned_fixture(t)
    defer engine.r3d_model_destroy(&m)
    testing.expect(t, ok)
    if !ok { return }
    sk := &m.skeleton
    // parents before children, whatever the file order
    for p, i in sk.parents { testing.expect(t, p < i) }
    b0 := engine.r3d_skeleton_find(sk, "Bone0")
    b1 := engine.r3d_skeleton_find(sk, "Bone1")
    prop := engine.r3d_skeleton_find(sk, "Prop")
    testing.expect(t, b0 >= 0 && b1 > b0 && prop > b1)
    testing.expect_value(t, sk.parents[b1], b0)
    testing.expect_value(t, sk.parents[prop], b1)

    testing.expect_value(t, len(m.skins), 1)
    testing.expect_value(t, m.skins[0].joints[0], b0)
    testing.expect_value(t, m.skins[0].joints[1], b1)
    testing.expect(t, near(m.skins[0].inverse_bind[1][1, 3], -1))

    testing.expect_value(t, len(m.prims), 2)
    body, propm: ^engine.R3D_Model_Prim
    for &p in m.prims {
        if p.name == "SkinnedMesh" { body = &p }
        if p.name == "Prop" { propm = &p }
    }
    testing.expect(t, body != nil && propm != nil)
    if body == nil || propm == nil { return }
    testing.expect_value(t, body.skin, 0)
    testing.expect_value(t, propm.skin, -1)
    testing.expect_value(t, propm.node, prop)
    testing.expect_value(t, len(body.indices), 6)
    testing.expect(t, body.vertices[2].joints == [4]u16{1, 0, 0, 0})
    testing.expect(t, near(body.vertices[2].weights[0], 0.75) && near(body.vertices[2].weights[1], 0.25))

    // generated tangents: unit, perpendicular to N, along +u (= +X); v runs
    // down while y runs up, so the bitangent sign is negative
    for v in body.vertices {
        tn := v.tangent.xyz
        testing.expect(t, near(linalg.length(tn), 1))
        testing.expect(t, near(linalg.dot(tn, v.normal), 0))
        testing.expect(t, near3(tn, {1, 0, 0}))
        testing.expect(t, v.tangent.w == -1)
    }
}

@(test)
test_model_load_material_and_image :: proc(t: ^testing.T) {
    m, ok := load_skinned_fixture(t)
    defer engine.r3d_model_destroy(&m)
    testing.expect(t, ok)
    if !ok { return }
    testing.expect_value(t, len(m.images), 1)
    im := &m.images[0]
    testing.expect(t, im.width == 2 && im.height == 2 && len(im.pixels) == 16)
    testing.expect(t, im.pixels[0] == 255 && im.pixels[1] == 0 && im.pixels[2] == 0)   // red top-left
    testing.expect(t, im.pixels[4] == 0 && im.pixels[5] == 255)                         // green
    testing.expect_value(t, im.pixels[15], 128)                                         // alpha kept
    testing.expect_value(t, len(m.materials), 1)
    mt := &m.materials[0]
    testing.expect(t, mt.base_color == [4]f32{1, 0.5, 0.25, 1})
    testing.expect(t, near(mt.metallic, 0.3) && near(mt.roughness, 0.7))
    testing.expect(t, mt.base_color_tex == 0 && mt.metal_rough_tex == 0 && mt.normal_tex == 0 &&
        mt.occlusion_tex == 0 && mt.emissive_tex == -1)
    testing.expect(t, near(mt.normal_scale, 0.5) && near(mt.occlusion_strength, 0.8))
    testing.expect(t, mt.emissive == [3]f32{1, 1, 0})
    testing.expect(t, mt.alpha_mode == .Mask && near(mt.alpha_cutoff, 0.3) && mt.double_sided)
}

@(test)
test_model_animation_skins_vertices :: proc(t: ^testing.T) {
    m, ok := load_skinned_fixture(t)
    defer engine.r3d_model_destroy(&m)
    testing.expect(t, ok)
    if !ok { return }
    testing.expect_value(t, len(m.clips), 1)
    clip := &m.clips[0]
    testing.expect(t, clip.name == "Bend" && near(clip.duration, 1))
    testing.expect_value(t, len(clip.tracks), 2)

    pose: engine.R3D_Model_Pose
    defer engine.r3d_model_pose_destroy(&pose)
    engine.r3d_model_pose_init(&pose, &m)
    body := &m.prims[0]
    if body.name != "SkinnedMesh" { body = &m.prims[1] }

    // rest pose = bind pose: skinning leaves vertices where they are
    for v in body.vertices {
        testing.expect(t, near3(engine.r3d_skin_point(pose.joints[0][:], v.pos, v.joints, v.weights), v.pos))
    }
    // the prop hangs off Bone1: (0,1,0) + (0,0.5,0)
    prop := engine.r3d_skeleton_find(&m.skeleton, "Prop")
    testing.expect(t, near3(engine.r3d_mat4_transform_point(pose.global[prop], {0, 0, 0}), {0, 1.5, 0}))

    // end of clip: Bone0 stepped to x=5, Bone1 bent 90 deg about Z
    engine.r3d_anim_clip_sample(&m.skeleton, clip, 1, false, pose.local[:])
    engine.r3d_model_pose_finish(&pose, &m)
    v3 := body.vertices[3] // (0,2,0), all on Bone1
    testing.expect(t, near3(engine.r3d_skin_point(pose.joints[0][:], v3.pos, v3.joints, v3.weights), {4, 1, 0}))
    v2 := body.vertices[2] // (1,2,0): 0.75 Bone1 -> (4,2,0), 0.25 Bone0 -> (6,2,0)
    testing.expect(t, near3(engine.r3d_skin_point(pose.joints[0][:], v2.pos, v2.joints, v2.weights), {4.5, 2, 0}))
    // STEP translation: still at 0 just before the last key
    engine.r3d_anim_clip_sample(&m.skeleton, clip, 0.99, false, pose.local[:])
    b0 := engine.r3d_skeleton_find(&m.skeleton, "Bone0")
    testing.expect(t, near3(pose.local[b0].t, {0, 0, 0}))
    // the prop follows the animated joint
    engine.r3d_anim_clip_sample(&m.skeleton, clip, 1, false, pose.local[:])
    engine.r3d_model_pose_finish(&pose, &m)
    testing.expect(t, near3(engine.r3d_mat4_transform_point(pose.global[prop], {0, 0, 0}), {4.5, 1, 0}))
}

@(test)
test_model_add_mesh_tangents :: proc(t: ^testing.T) {
    m: engine.R3D_Model
    defer engine.r3d_model_destroy(&m)
    plane := engine.r3d_mesh_plane(10, 2)
    defer engine.r3d_mesh_data_destroy(&plane)
    append(&m.materials, engine.r3d_default_material())
    pi := engine.r3d_model_add_mesh(&m, "ground", &plane, 0)
    testing.expect_value(t, pi, 0)
    testing.expect_value(t, len(m.skeleton.parents), 1)
    p := &m.prims[0]
    testing.expect(t, p.skin == -1 && p.node == 0)
    testing.expect(t, near3(p.bounds_min, {-5, 0, -5}) && near3(p.bounds_max, {5, 0, 5}))
    for v in p.vertices {
        testing.expect(t, near(linalg.dot(v.tangent.xyz, v.normal), 0))
        testing.expect(t, near3(v.tangent.xyz, {1, 0, 0})) // plane u runs along +X
    }
    // front faces (CCW) point up the normals, so back-face culling keeps them
    for i := 0; i + 2 < len(p.indices); i += 3 {
        a, b, c := p.vertices[p.indices[i]].pos, p.vertices[p.indices[i + 1]].pos, p.vertices[p.indices[i + 2]].pos
        testing.expect(t, linalg.cross(b - a, c - a).y > 0)
    }
}

// The CC0 KayKit knight used by examples/templar3d (skipped when absent).
@(test)
test_model_load_kaykit_knight :: proc(t: ^testing.T) {
    path := "examples/templar3d/assets/kaykit_knight.glb"
    if !os.exists(path) { return }
    m, ok := engine.r3d_model_load(path)
    defer engine.r3d_model_destroy(&m)
    testing.expect(t, ok)
    if !ok { return }
    testing.expect_value(t, len(m.skins), 1)
    testing.expect_value(t, len(m.skins[0].joints), 41)
    testing.expect(t, engine.r3d_anim_find_clip(m.clips[:], "Idle") >= 0)
    testing.expect(t, engine.r3d_anim_find_clip(m.clips[:], "Walking_A") >= 0)
    testing.expect(t, len(m.images) == 1 && m.images[0].width > 0)
    for p, i in m.skeleton.parents { testing.expect(t, p < i) }

    // a walk frame moves skinned vertices; every skinned vertex stays finite
    // and within a few metres of the model origin
    pose: engine.R3D_Model_Pose
    defer engine.r3d_model_pose_destroy(&pose)
    engine.r3d_model_pose_init(&pose, &m)
    walk := engine.r3d_anim_find_clip(m.clips[:], "Walking_A")
    engine.r3d_anim_clip_sample(&m.skeleton, &m.clips[walk], 0.3, true, pose.local[:])
    engine.r3d_model_pose_finish(&pose, &m)
    moved := 0
    for &p in m.prims {
        if p.skin < 0 { continue }
        for v in p.vertices {
            q := engine.r3d_skin_point(pose.joints[p.skin][:], v.pos, v.joints, v.weights)
            testing.expect(t, linalg.length(q) < 5)
            if linalg.length(q - v.pos) > 0.01 { moved += 1 }
        }
    }
    testing.expect(t, moved > 100)
}

// Palette-style uvs (tiny or zero uv area) must still give finite, unit,
// perpendicular tangents.
@(test)
test_tangents_degenerate_uvs :: proc(t: ^testing.T) {
    verts := []engine.R3D_Skin_Vertex{
        {pos = {0, 0, 0}, normal = {0, 0, 1}, uv = {0.5, 0.5}},
        {pos = {1, 0, 0}, normal = {0, 0, 1}, uv = {0.5 + 1e-7, 0.5}},
        {pos = {0, 1, 0}, normal = {0, 0, 1}, uv = {0.5, 0.5 + 3e-8}},
        {pos = {1, 1, 0}, normal = {0, 0, 1}, uv = {0.5, 0.5}},
    }
    idx := []u32{0, 1, 2, 1, 3, 2}
    engine.r3d_generate_tangents(verts, idx)
    for v in verts {
        tn := v.tangent.xyz
        testing.expect(t, !math.is_nan(tn.x) && !math.is_nan(tn.y) && !math.is_nan(tn.z))
        testing.expect(t, near(linalg.length(tn), 1))
        testing.expect(t, near(linalg.dot(tn, v.normal), 0))
        testing.expect(t, v.tangent.w == 1 || v.tangent.w == -1)
    }
}

@(test)
test_data_uri_helpers :: proc(t: ^testing.T) {
    b, ok := engine.r3d_decode_data_uri("data:application/octet-stream;base64,AAEC/w==")
    defer delete(b)
    testing.expect(t, ok && len(b) == 4 && b[0] == 0 && b[1] == 1 && b[2] == 2 && b[3] == 255)
    p, ok2 := engine.r3d_decode_data_uri("data:,a%20b")
    defer delete(p)
    testing.expect(t, ok2 && string(p) == "a b")
    _, bad := engine.r3d_decode_data_uri("data:image/png;base64")
    testing.expect(t, !bad)
    u := engine.r3d_uri_unescape("my%20tex%2Fx%zz.png")
    defer delete(u)
    testing.expect_value(t, u, "my tex/x%zz.png")
}

// A .gltf (text) whose buffer and first image are base64 data URIs and whose
// second image is an external file with a percent-escaped name.
@(test)
test_model_load_data_uri_images :: proc(t: ^testing.T) {
    bin: [dynamic]u8
    defer delete(bin)
    for v in ([?]f32{0, 0, 0, 1, 0, 0, 0, 1, 0}) { glb_f32(&bin, v) }   // positions
    for v in ([?]f32{0, 0, 1, 0, 0, 1}) { glb_f32(&bin, v) }            // uvs
    buf_b64, _ := base64.encode(bin[:])
    defer delete(buf_b64)
    png_b64, _ := base64.encode(SKIN_PNG[:])
    defer delete(png_b64)
    dir := "/tmp/voidengine_test_datauri"
    os.make_directory(dir)
    defer os.remove(dir)
    ext := "/tmp/voidengine_test_datauri/tiny tex.png"
    testing.expect(t, os.write_entire_file(ext, SKIN_PNG[:]) == nil)
    defer os.remove(ext)
    js := fmt.aprintf(
        `{{"asset":{{"version":"2.0"}},"scene":0,"scenes":[{{"nodes":[0]}}],"nodes":[{{"mesh":0}}],` +
        `"meshes":[{{"primitives":[{{"attributes":{{"POSITION":0,"TEXCOORD_0":1}},"material":0}}]}}],` +
        `"materials":[{{"pbrMetallicRoughness":{{"baseColorTexture":{{"index":0}}}},"emissiveTexture":{{"index":1}}}}],` +
        `"textures":[{{"source":0}},{{"source":1}}],` +
        `"images":[{{"uri":"data:image/png;base64,%s"}},{{"uri":"tiny%%20tex.png"}}],` +
        `"accessors":[{{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[0,0,0],"max":[1,1,0]}},` +
        `{{"bufferView":1,"componentType":5126,"count":3,"type":"VEC2"}}],` +
        `"bufferViews":[{{"buffer":0,"byteOffset":0,"byteLength":36}},{{"buffer":0,"byteOffset":36,"byteLength":24}}],` +
        `"buffers":[{{"byteLength":%d,"uri":"data:application/octet-stream;base64,%s"}}]}}`,
        png_b64, len(bin), buf_b64)
    defer delete(js)
    path := "/tmp/voidengine_test_datauri/m.gltf"
    testing.expect(t, os.write_entire_file(path, transmute([]u8)js) == nil)
    defer os.remove(path)
    m, ok := engine.r3d_model_load(path)
    defer engine.r3d_model_destroy(&m)
    testing.expect(t, ok)
    if !ok { return }
    testing.expect_value(t, len(m.images), 2)
    for &im in m.images {
        testing.expect(t, im.width == 2 && im.height == 2 && len(im.pixels) == 16)
        if len(im.pixels) == 16 { testing.expect(t, im.pixels[0] == 255 && im.pixels[5] == 255) }
    }
    testing.expect(t, len(m.prims) == 1 && len(m.prims[0].vertices) == 3)
}
