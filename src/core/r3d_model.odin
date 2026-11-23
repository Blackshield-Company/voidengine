package voidengine

// ============================================================================
// r3d_model — textured, skinned glTF 2.0 models (backend-agnostic, CPU side).
//
// Loads .gltf/.glb into an R3D_Model: the whole node hierarchy as a skeleton
// (parents before children), skins (joints + inverse bind matrices), every
// animation clip (T/R/S, step/linear/cubic), PBR metallic-roughness materials
// and their images decoded to RGBA8. Each triangle primitive becomes an
// R3D_Model_Prim with a 72-byte vertex (pos, normal, uv, tangent, 4 joints,
// 4 weights). Tangents are generated when the file ships none.
//
// Drawing rules (glTF spec): a skinned primitive is placed by its joint
// matrices only (its node transform is ignored); a static primitive uses its
// node's global matrix, so props parented to joints (swords, helmets) follow
// the animated skeleton.
//
// The older gltf.odin (static flatten + rigid node anims) is untouched.
// ============================================================================

import "core:math"
import "core:math/linalg"
import "core:os"
import "core:path/filepath"
import "core:strings"
import "core:encoding/base64"
import "core:image"
import _ "core:image/png"
import _ "core:image/jpeg"
import "vendor:cgltf"

R3D_Skin_Vertex :: struct {
    pos:     [3]f32,
    normal:  [3]f32,
    uv:      [2]f32,
    tangent: [4]f32, // xyz tangent, w = bitangent sign
    joints:  [4]u16,
    weights: [4]f32,
}

R3D_SKIN_VERTEX_STRIDE :: size_of(R3D_Skin_Vertex) // 72

R3D_Image :: struct {
    name:   string,
    width:  int,
    height: int,
    pixels: []u8, // RGBA8, top row first
}

R3D_Alpha_Mode :: enum u8 { Opaque, Mask, Blend }

// Texture slots refer to R3D_Model.images (-1 = none). Color slots
// (base_color, emissive) are sRGB; the others are linear data.
R3D_Material :: struct {
    name:               string,
    base_color:         [4]f32,
    metallic:           f32,
    roughness:          f32,
    emissive:           [3]f32,
    normal_scale:       f32,
    occlusion_strength: f32,
    alpha_mode:         R3D_Alpha_Mode,
    alpha_cutoff:       f32,
    double_sided:       bool,
    uv_scale:           [2]f32, // extra tiling (1,1 from files; ground planes tile)
    base_color_tex:     int,
    metal_rough_tex:    int, // glTF: G = roughness, B = metallic (ORM packs R = AO too)
    normal_tex:         int,
    occlusion_tex:      int, // R channel
    emissive_tex:       int,
}

r3d_default_material :: proc "contextless" () -> R3D_Material {
    return R3D_Material{
        base_color = {1, 1, 1, 1}, metallic = 0, roughness = 1,
        normal_scale = 1, occlusion_strength = 1, alpha_cutoff = 0.5,
        uv_scale = {1, 1},
        base_color_tex = -1, metal_rough_tex = -1, normal_tex = -1, occlusion_tex = -1, emissive_tex = -1,
    }
}

R3D_Model_Prim :: struct {
    name:     string, // node name
    node:     int,    // skeleton node the primitive hangs off
    skin:     int,    // -1 = static (uses the node's global matrix)
    material: int,    // -1 = default material
    vertices: [dynamic]R3D_Skin_Vertex,
    indices:  [dynamic]u32,
    bounds_min, bounds_max: [3]f32,
}

R3D_Model :: struct {
    skeleton:  R3D_Skeleton,
    skins:     [dynamic]R3D_Skin,
    prims:     [dynamic]R3D_Model_Prim,
    materials: [dynamic]R3D_Material,
    images:    [dynamic]R3D_Image,
    clips:     [dynamic]R3D_Anim_Clip,
}

r3d_model_destroy :: proc(m: ^R3D_Model) {
    for n in m.skeleton.names { delete(n) }
    delete(m.skeleton.names)
    delete(m.skeleton.parents)
    delete(m.skeleton.rest)
    for &s in m.skins {
        delete(s.name)
        delete(s.joints)
        delete(s.inverse_bind)
    }
    delete(m.skins)
    for &p in m.prims {
        delete(p.name)
        delete(p.vertices)
        delete(p.indices)
    }
    delete(m.prims)
    for &mt in m.materials { delete(mt.name) }
    delete(m.materials)
    for &im in m.images {
        delete(im.name)
        delete(im.pixels)
    }
    delete(m.images)
    for &c in m.clips { r3d_anim_clip_destroy(&c) }
    delete(m.clips)
    m^ = {}
}

// ----------------------------------------------------------------------------
// Loading
// ----------------------------------------------------------------------------

r3d_model_load :: proc(path: string, allocator := context.allocator) -> (model: R3D_Model, ok: bool) {
    context.allocator = allocator
    opts: cgltf.options
    cpath := strings.clone_to_cstring(path, context.temp_allocator)
    data, res := cgltf.parse_file(opts, cpath)
    if res != .success { return model, false }
    defer cgltf.free(data)
    if cgltf.load_buffers(opts, data, cpath) != .success { return model, false }
    if cgltf.validate(data) != .success { return model, false }

    base_dir := filepath.dir(path)

    // --- node order: parents before children (DFS from roots) ---
    nnodes := len(data.nodes)
    order := make([dynamic]int, 0, nnodes, context.temp_allocator)
    remap := make([]int, nnodes, context.temp_allocator)
    for i in 0 ..< nnodes { remap[i] = -1 }
    visit :: proc(data: ^cgltf.data, n: ^cgltf.node, order: ^[dynamic]int, remap: []int) {
        idx := int(cgltf.node_index(data, n))
        if remap[idx] >= 0 { return }
        remap[idx] = len(order)
        append(order, idx)
        for c in n.children { visit(data, c, order, remap) }
    }
    for i in 0 ..< nnodes {
        if data.nodes[i].parent == nil { visit(data, &data.nodes[i], &order, remap) }
    }

    sk := &model.skeleton
    for src in order {
        n := &data.nodes[src]
        name := n.name != nil ? strings.clone(string(n.name)) : strings.clone("")
        append(&sk.names, name)
        append(&sk.parents, n.parent != nil ? remap[int(cgltf.node_index(data, n.parent))] : -1)
        trs := R3D_TRS_IDENTITY
        if n.has_matrix {
            trs = r3d_trs_from_matrix(transmute(linalg.Matrix4f32)n.matrix_)
        } else {
            if n.has_translation { trs.t = n.translation }
            if n.has_rotation    { trs.r = r3d_quat_normalize(n.rotation) }
            if n.has_scale       { trs.s = n.scale }
        }
        append(&sk.rest, trs)
    }

    // --- images (decoded lazily by slot use is overkill; decode all) ---
    for i in 0 ..< len(data.images) {
        im := &data.images[i]
        img := R3D_Image{name = im.name != nil ? strings.clone(string(im.name)) : strings.clone("")}
        bytes: []u8
        owned := false
        if im.buffer_view != nil {
            p := cgltf.buffer_view_data(im.buffer_view)
            bytes = p[:im.buffer_view.size]
        } else if im.uri != nil {
            uri := string(im.uri)
            if strings.has_prefix(uri, "data:") {
                d, dok := r3d_decode_data_uri(uri)
                if dok { bytes = d; owned = true }
            } else {
                rel := r3d_uri_unescape(uri, context.temp_allocator)
                full, _ := filepath.join({base_dir, rel}, context.temp_allocator)
                d, err := os.read_entire_file_from_path(full, context.allocator)
                if err == nil { bytes = d; owned = true }
            }
        }
        if len(bytes) > 0 {
            img.pixels, img.width, img.height = r3d_decode_rgba8(bytes)
        }
        if owned { delete(bytes) }
        append(&model.images, img)
    }

    tex_image :: proc(data: ^cgltf.data, v: cgltf.texture_view) -> int {
        if v.texture == nil || v.texture.image_ == nil { return -1 }
        return int(cgltf.image_index(data, v.texture.image_))
    }

    // --- materials ---
    for i in 0 ..< len(data.materials) {
        src := &data.materials[i]
        mt := r3d_default_material()
        mt.name = src.name != nil ? strings.clone(string(src.name)) : strings.clone("")
        if src.has_pbr_metallic_roughness {
            pbr := &src.pbr_metallic_roughness
            mt.base_color = pbr.base_color_factor
            mt.metallic = pbr.metallic_factor
            mt.roughness = pbr.roughness_factor
            mt.base_color_tex = tex_image(data, pbr.base_color_texture)
            mt.metal_rough_tex = tex_image(data, pbr.metallic_roughness_texture)
        }
        mt.normal_tex = tex_image(data, src.normal_texture)
        if mt.normal_tex >= 0 && src.normal_texture.scale != 0 { mt.normal_scale = src.normal_texture.scale }
        mt.occlusion_tex = tex_image(data, src.occlusion_texture)
        if mt.occlusion_tex >= 0 { mt.occlusion_strength = src.occlusion_texture.scale }
        mt.emissive_tex = tex_image(data, src.emissive_texture)
        mt.emissive = src.emissive_factor
        if src.has_emissive_strength { mt.emissive *= src.emissive_strength.emissive_strength }
        switch src.alpha_mode {
        case .opaque: mt.alpha_mode = .Opaque
        case .mask:   mt.alpha_mode = .Mask
        case .blend:  mt.alpha_mode = .Blend
        }
        mt.alpha_cutoff = src.alpha_cutoff
        mt.double_sided = bool(src.double_sided)
        append(&model.materials, mt)
    }

    // --- skins ---
    for i in 0 ..< len(data.skins) {
        src := &data.skins[i]
        s := R3D_Skin{name = src.name != nil ? strings.clone(string(src.name)) : strings.clone("")}
        nj := len(src.joints)
        for j in 0 ..< nj {
            append(&s.joints, remap[int(cgltf.node_index(data, src.joints[j]))])
        }
        resize(&s.inverse_bind, nj)
        for j in 0 ..< nj { s.inverse_bind[j] = linalg.MATRIX4F32_IDENTITY }
        if src.inverse_bind_matrices != nil && int(src.inverse_bind_matrices.count) >= nj {
            raw := make([]f32, nj * 16, context.temp_allocator)
            _ = cgltf.accessor_unpack_floats(src.inverse_bind_matrices, raw_data(raw), uint(nj * 16))
            for j in 0 ..< nj {
                m: [16]f32
                copy(m[:], raw[j * 16:(j + 1) * 16])
                s.inverse_bind[j] = transmute(linalg.Matrix4f32)m
            }
        }
        append(&model.skins, s)
    }

    // --- primitives (per node, in skeleton order) ---
    for src, dst in order {
        n := &data.nodes[src]
        if n.mesh == nil { continue }
        skin := n.skin != nil ? int(cgltf.skin_index(data, n.skin)) : -1
        for &p in n.mesh.primitives {
            if p.type != .triangles { continue }
            prim, pok := r3d_model_load_primitive(data, &p)
            if !pok { continue }
            prim.name = strings.clone(n.name != nil ? string(n.name) : (n.mesh.name != nil ? string(n.mesh.name) : ""))
            prim.node = dst
            prim.skin = skin
            append(&model.prims, prim)
        }
    }

    // --- animations ---
    for i in 0 ..< len(data.animations) {
        a := &data.animations[i]
        clip := R3D_Anim_Clip{name = strings.clone(a.name != nil ? string(a.name) : "anim")}
        for &ch in a.channels {
            if ch.target_node == nil || ch.sampler == nil { continue }
            tr: R3D_Anim_Track
            #partial switch ch.target_path {
            case .translation: tr.path = .Translation
            case .rotation:    tr.path = .Rotation
            case .scale:       tr.path = .Scale
            case: continue
            }
            switch ch.sampler.interpolation {
            case .linear:       tr.interp = .Linear
            case .step:         tr.interp = .Step
            case .cubic_spline: tr.interp = .Cubic
            }
            tr.node = remap[int(cgltf.node_index(data, ch.target_node))]
            nkey := int(ch.sampler.input.count)
            comps := tr.path == .Rotation ? 4 : 3
            per := tr.interp == .Cubic ? comps * 3 : comps
            if nkey == 0 || int(ch.sampler.output.count) * comps < nkey * per { continue }
            tr.times = make([]f32, nkey)
            tr.values = make([]f32, nkey * per)
            _ = cgltf.accessor_unpack_floats(ch.sampler.input, raw_data(tr.times), uint(nkey))
            _ = cgltf.accessor_unpack_floats(ch.sampler.output, raw_data(tr.values), uint(nkey * per))
            if tr.times[nkey - 1] > clip.duration { clip.duration = tr.times[nkey - 1] }
            append(&clip.tracks, tr)
        }
        append(&model.clips, clip)
    }

    ok = len(model.prims) > 0
    return
}

// Decode PNG/JPEG (or anything core:image knows) to RGBA8.
r3d_decode_rgba8 :: proc(bytes: []u8, allocator := context.allocator) -> (pixels: []u8, w, h: int) {
    img, err := image.load_from_bytes(bytes, {}, context.temp_allocator)
    if err != nil || img == nil { return nil, 0, 0 }
    w, h = img.width, img.height
    src := img.pixels.buf[:]
    pixels = make([]u8, w * h * 4, allocator)
    ch := img.channels
    if img.depth != 8 {
        // 16-bit: take the high byte (big-endian samples in core:image)
        for i in 0 ..< w * h {
            for c in 0 ..< 4 {
                v: u8 = 255
                if c < ch { v = src[(i * ch + c) * 2 + 1] }
                else if c < 3 && ch < 3 { v = src[(i * ch) * 2 + 1] }
                pixels[i * 4 + c] = v
            }
        }
        return
    }
    for i in 0 ..< w * h {
        switch ch {
        case 4:
            copy(pixels[i * 4:i * 4 + 4], src[i * 4:i * 4 + 4])
        case 3:
            pixels[i * 4 + 0] = src[i * 3 + 0]
            pixels[i * 4 + 1] = src[i * 3 + 1]
            pixels[i * 4 + 2] = src[i * 3 + 2]
            pixels[i * 4 + 3] = 255
        case 2:
            pixels[i * 4 + 0] = src[i * 2]
            pixels[i * 4 + 1] = src[i * 2]
            pixels[i * 4 + 2] = src[i * 2]
            pixels[i * 4 + 3] = src[i * 2 + 1]
        case 1:
            pixels[i * 4 + 0] = src[i]
            pixels[i * 4 + 1] = src[i]
            pixels[i * 4 + 2] = src[i]
            pixels[i * 4 + 3] = 255
        }
    }
    return
}

// Load an image file into the model's image list; returns its index (-1 on error).
r3d_model_add_image_file :: proc(m: ^R3D_Model, path: string) -> int {
    data, err := os.read_entire_file_from_path(path, context.allocator)
    if err != nil { return -1 }
    defer delete(data)
    px, w, h := r3d_decode_rgba8(data)
    if px == nil { return -1 }
    append(&m.images, R3D_Image{name = strings.clone(filepath.base(path)), width = w, height = h, pixels = px})
    return len(m.images) - 1
}

// Wrap engine mesh data (r3d_mesh_plane, OBJ, …) as a static model primitive
// with generated tangents. Adds a root node when the skeleton is empty.
r3d_model_add_mesh :: proc(m: ^R3D_Model, name: string, mesh: ^R3D_Mesh_Data, material: int) -> int {
    if len(m.skeleton.parents) == 0 {
        append(&m.skeleton.names, strings.clone("root"))
        append(&m.skeleton.parents, -1)
        append(&m.skeleton.rest, R3D_TRS_IDENTITY)
    }
    p := R3D_Model_Prim{name = strings.clone(name), node = 0, skin = -1, material = material}
    for v in mesh.vertices {
        append(&p.vertices, R3D_Skin_Vertex{pos = v.pos, normal = v.normal, uv = v.uv})
    }
    for i in mesh.indices { append(&p.indices, i) }
    r3d_fix_winding(p.vertices[:], p.indices[:])
    r3d_generate_tangents(p.vertices[:], p.indices[:])
    r3d_prim_bounds(&p)
    append(&m.prims, p)
    return len(m.prims) - 1
}

@(private)
r3d_model_load_primitive :: proc(data: ^cgltf.data, p: ^cgltf.primitive) -> (prim: R3D_Model_Prim, ok: bool) {
    prim.material = p.material != nil ? int(cgltf.material_index(data, p.material)) : -1
    pos_acc, nrm_acc, uv_acc, tan_acc, jnt_acc, wgt_acc: ^cgltf.accessor
    for a in p.attributes {
        #partial switch a.type {
        case .position: pos_acc = a.data
        case .normal:   nrm_acc = a.data
        case .tangent:  tan_acc = a.data
        case .texcoord: if a.index == 0 { uv_acc = a.data }
        case .joints:   if a.index == 0 { jnt_acc = a.data }
        case .weights:  if a.index == 0 { wgt_acc = a.data }
        }
    }
    if pos_acc == nil { return }
    n := int(pos_acc.count)
    resize(&prim.vertices, n)
    tmp := make([]f32, n * 4, context.temp_allocator)

    _ = cgltf.accessor_unpack_floats(pos_acc, raw_data(tmp), uint(n * 3))
    for i in 0 ..< n { prim.vertices[i].pos = {tmp[i * 3], tmp[i * 3 + 1], tmp[i * 3 + 2]} }
    if nrm_acc != nil {
        _ = cgltf.accessor_unpack_floats(nrm_acc, raw_data(tmp), uint(n * 3))
        for i in 0 ..< n { prim.vertices[i].normal = {tmp[i * 3], tmp[i * 3 + 1], tmp[i * 3 + 2]} }
    }
    if uv_acc != nil {
        // accessor_unpack_floats de-normalises u8/u16 normalised uvs too
        _ = cgltf.accessor_unpack_floats(uv_acc, raw_data(tmp), uint(n * 2))
        for i in 0 ..< n { prim.vertices[i].uv = {tmp[i * 2], tmp[i * 2 + 1]} }
    }
    if tan_acc != nil {
        _ = cgltf.accessor_unpack_floats(tan_acc, raw_data(tmp), uint(n * 4))
        for i in 0 ..< n { prim.vertices[i].tangent = {tmp[i * 4], tmp[i * 4 + 1], tmp[i * 4 + 2], tmp[i * 4 + 3]} }
    }
    if jnt_acc != nil && wgt_acc != nil {
        for i in 0 ..< n {
            j: [4]u32
            _ = cgltf.accessor_read_uint(jnt_acc, uint(i), raw_data(j[:]), 4)
            prim.vertices[i].joints = {u16(j[0]), u16(j[1]), u16(j[2]), u16(j[3])}
        }
        _ = cgltf.accessor_unpack_floats(wgt_acc, raw_data(tmp), uint(n * 4))
        for i in 0 ..< n {
            w := [4]f32{tmp[i * 4], tmp[i * 4 + 1], tmp[i * 4 + 2], tmp[i * 4 + 3]}
            s := w[0] + w[1] + w[2] + w[3]
            if s > 1e-6 { w /= s } else { w = {1, 0, 0, 0} }
            prim.vertices[i].weights = w
        }
    }

    if p.indices != nil {
        count := int(p.indices.count)
        resize(&prim.indices, count)
        _ = cgltf.accessor_unpack_indices(p.indices, raw_data(prim.indices), 4, uint(count))
    } else {
        resize(&prim.indices, n)
        for i in 0 ..< n { prim.indices[i] = u32(i) }
    }

    if nrm_acc == nil { r3d_generate_normals(prim.vertices[:], prim.indices[:]) }
    if tan_acc == nil { r3d_generate_tangents(prim.vertices[:], prim.indices[:]) }
    r3d_prim_bounds(&prim)
    ok = true
    return
}

// Make every triangle counter-clockwise when seen from the side its vertex
// normals point to (glTF front faces), so back-face culling keeps it. Some
// engine mesh builders (r3d_mesh_plane) wind the other way.
r3d_fix_winding :: proc(verts: []R3D_Skin_Vertex, indices: []u32) {
    for i := 0; i + 2 < len(indices); i += 3 {
        a, b, c := verts[indices[i]], verts[indices[i + 1]], verts[indices[i + 2]]
        g := linalg.cross(b.pos - a.pos, c.pos - a.pos)
        if linalg.dot(g, a.normal + b.normal + c.normal) < 0 {
            indices[i + 1], indices[i + 2] = indices[i + 2], indices[i + 1]
        }
    }
}

r3d_prim_bounds :: proc(p: ^R3D_Model_Prim) {
    if len(p.vertices) == 0 { return }
    p.bounds_min = p.vertices[0].pos
    p.bounds_max = p.vertices[0].pos
    for v in p.vertices {
        p.bounds_min = linalg.min(p.bounds_min, v.pos)
        p.bounds_max = linalg.max(p.bounds_max, v.pos)
    }
}

r3d_generate_normals :: proc(verts: []R3D_Skin_Vertex, indices: []u32) {
    for &v in verts { v.normal = {} }
    for i := 0; i + 2 < len(indices); i += 3 {
        a, b, c := &verts[indices[i]], &verts[indices[i + 1]], &verts[indices[i + 2]]
        n := linalg.cross(b.pos - a.pos, c.pos - a.pos)
        a.normal += n; b.normal += n; c.normal += n
    }
    for &v in verts {
        l := linalg.length(v.normal)
        v.normal = l > 1e-12 ? v.normal / l : {0, 1, 0}
    }
}

// Per-vertex tangents from uv derivatives (accumulated per triangle, then
// Gram-Schmidt against the normal; w = handedness). Degenerate uvs fall back
// to any vector perpendicular to the normal.
r3d_generate_tangents :: proc(verts: []R3D_Skin_Vertex, indices: []u32) {
    n := len(verts)
    tan := make([][3]f32, n, context.temp_allocator)
    bit := make([][3]f32, n, context.temp_allocator)
    for i := 0; i + 2 < len(indices); i += 3 {
        i0, i1, i2 := indices[i], indices[i + 1], indices[i + 2]
        v0, v1, v2 := verts[i0], verts[i1], verts[i2]
        e1 := v1.pos - v0.pos
        e2 := v2.pos - v0.pos
        du1, dv1 := v1.uv.x - v0.uv.x, v1.uv.y - v0.uv.y
        du2, dv2 := v2.uv.x - v0.uv.x, v2.uv.y - v0.uv.y
        det := du1 * dv2 - du2 * dv1
        // palette-textured meshes (all three uvs on one texel) have ~0 uv
        // area: skip them rather than blow up to inf/NaN
        if !(math.abs(det) > 1e-10) { continue }
        r := 1 / det
        t := (e1 * dv2 - e2 * dv1) * r
        b := (e2 * du1 - e1 * du2) * r
        // weight by triangle area, not 1/uv-area (stable for tiny uv islands)
        tl, bl := linalg.length(t), linalg.length(b)
        area := linalg.length(linalg.cross(e1, e2))
        if !(tl > 1e-20) || !(bl > 1e-20) || !(tl < 1e30) || !(bl < 1e30) { continue }
        t = t / tl * area
        b = b / bl * area
        for k in ([3]u32{i0, i1, i2}) {
            tan[k] += t
            bit[k] += b
        }
    }
    for i in 0 ..< n {
        nn := verts[i].normal
        t := tan[i] - nn * linalg.dot(nn, tan[i])
        l := linalg.length(t)
        if !(l > 1e-8) || !(l < 1e30) { // NaN-safe
            // any perpendicular
            ref := math.abs(nn.y) < 0.99 ? [3]f32{0, 1, 0} : [3]f32{1, 0, 0}
            t = linalg.normalize(linalg.cross(ref, nn))
        } else {
            t /= l
        }
        w: f32 = linalg.dot(linalg.cross(nn, t), bit[i]) < 0 ? -1 : 1
        verts[i].tangent = {t.x, t.y, t.z, w}
    }
}

// Decompose an affine TRS matrix (no shear).
r3d_trs_from_matrix :: proc(m: linalg.Matrix4f32) -> (x: R3D_Trs) {
    x.t = {m[0, 3], m[1, 3], m[2, 3]}
    c0 := [3]f32{m[0, 0], m[1, 0], m[2, 0]}
    c1 := [3]f32{m[0, 1], m[1, 1], m[2, 1]}
    c2 := [3]f32{m[0, 2], m[1, 2], m[2, 2]}
    sx, sy, sz := linalg.length(c0), linalg.length(c1), linalg.length(c2)
    if linalg.dot(linalg.cross(c0, c1), c2) < 0 { sx = -sx }
    x.s = {sx, sy, sz}
    r00, r10, r20 := c0.x / sx, c0.y / sx, c0.z / sx
    r01, r11, r21 := c1.x / sy, c1.y / sy, c1.z / sy
    r02, r12, r22 := c2.x / sz, c2.y / sz, c2.z / sz
    tr := r00 + r11 + r22
    q: R3D_Quat
    if tr > 0 {
        s := math.sqrt(tr + 1) * 2
        q = {(r21 - r12) / s, (r02 - r20) / s, (r10 - r01) / s, 0.25 * s}
    } else if r00 > r11 && r00 > r22 {
        s := math.sqrt(1 + r00 - r11 - r22) * 2
        q = {0.25 * s, (r01 + r10) / s, (r02 + r20) / s, (r21 - r12) / s}
    } else if r11 > r22 {
        s := math.sqrt(1 + r11 - r00 - r22) * 2
        q = {(r01 + r10) / s, 0.25 * s, (r12 + r21) / s, (r02 - r20) / s}
    } else {
        s := math.sqrt(1 + r22 - r00 - r11) * 2
        q = {(r02 + r20) / s, (r12 + r21) / s, 0.25 * s, (r10 - r01) / s}
    }
    x.r = r3d_quat_normalize(q)
    return
}

// ----------------------------------------------------------------------------
// Runtime instance: pose buffers for one animated copy of a model.
// ----------------------------------------------------------------------------

R3D_Model_Pose :: struct {
    local:  [dynamic]R3D_Trs,
    global: [dynamic]linalg.Matrix4f32,
    joints: [dynamic][dynamic]linalg.Matrix4f32, // per skin
}

r3d_model_pose_init :: proc(p: ^R3D_Model_Pose, m: ^R3D_Model) {
    n := len(m.skeleton.parents)
    resize(&p.local, n)
    copy(p.local[:], m.skeleton.rest[:])
    resize(&p.global, n)
    resize(&p.joints, len(m.skins))
    for s, i in m.skins { resize(&p.joints[i], len(s.joints)) }
    r3d_model_pose_finish(p, m)
}

// After writing p.local: compute globals and skinning matrices.
r3d_model_pose_finish :: proc(p: ^R3D_Model_Pose, m: ^R3D_Model) {
    r3d_pose_global(&m.skeleton, p.local[:], p.global[:])
    for &s, i in m.skins { r3d_skin_matrices(&s, p.global[:], p.joints[i][:]) }
}

r3d_model_pose_destroy :: proc(p: ^R3D_Model_Pose) {
    delete(p.local)
    delete(p.global)
    for j in p.joints { delete(j) }
    delete(p.joints)
}

// Bytes of an RFC 2397 data URI ("data:[<mime>][;base64],<payload>"), as
// glTF embeds images; the payload may be base64 or percent-encoded.
r3d_decode_data_uri :: proc(uri: string, allocator := context.allocator) -> (bytes: []u8, ok: bool) {
    if !strings.has_prefix(uri, "data:") { return nil, false }
    comma := strings.index_byte(uri, ',')
    if comma < 0 { return nil, false }
    header := uri[5:comma]
    payload := uri[comma + 1:]
    if strings.has_suffix(header, ";base64") {
        out, err := base64.decode(payload, allocator = allocator)
        if err != nil { return nil, false }
        return out, true
    }
    raw := r3d_uri_unescape(payload, allocator)
    return transmute([]u8)raw, true
}

// Percent-decoding (%20 -> ' ') for glTF uris; other bytes are copied.
r3d_uri_unescape :: proc(s: string, allocator := context.allocator) -> string {
    hex :: proc(c: u8) -> int {
        switch c {
        case '0' ..= '9': return int(c - '0')
        case 'a' ..= 'f': return int(c - 'a') + 10
        case 'A' ..= 'F': return int(c - 'A') + 10
        }
        return -1
    }
    out := make([dynamic]u8, 0, len(s), allocator)
    for i := 0; i < len(s); i += 1 {
        if s[i] == '%' && i + 2 < len(s) {
            h, l := hex(s[i + 1]), hex(s[i + 2])
            if h >= 0 && l >= 0 {
                append(&out, u8(h * 16 + l))
                i += 2
                continue
            }
        }
        append(&out, s[i])
    }
    return string(out[:])
}
