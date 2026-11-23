package voidengine

// ============================================================================
// skin3d — backend-agnostic skeletal animation core.
//
// Pure CPU code (unit-tested): quaternion/TRS math, node skeletons, glTF-style
// animation tracks (step / linear / cubic spline, slerp for rotations), pose
// blending, a crossfading animation player, global poses and skinning (joint)
// matrices. The GL/VK backends only upload the joint matrices; vertices are
// skinned on the GPU. r3d_skin_point is the CPU reference used by the tests.
//
// Conventions follow glTF 2.0: quaternions are {x, y, z, w}; a local
// transform is T * R * S; a skeleton is the full node hierarchy (parents
// before children), and a skin is a list of node indices plus inverse bind
// matrices. joint_matrix[j] = global(node[j]) * inverse_bind[j].
// ============================================================================

import "core:math"
import "core:math/linalg"

R3D_Quat :: [4]f32 // x, y, z, w

R3D_QUAT_IDENTITY :: R3D_Quat{0, 0, 0, 1}

R3D_Trs :: struct {
    t: [3]f32,
    r: R3D_Quat,
    s: [3]f32,
}

R3D_TRS_IDENTITY :: R3D_Trs{t = {0, 0, 0}, r = {0, 0, 0, 1}, s = {1, 1, 1}}

// ----------------------------------------------------------------------------
// Quaternion helpers ({x,y,z,w}, Hamilton product)
// ----------------------------------------------------------------------------

r3d_quat_mul :: proc(a, b: R3D_Quat) -> R3D_Quat {
    return {
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
    }
}

r3d_quat_dot :: proc(a, b: R3D_Quat) -> f32 {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w
}

r3d_quat_normalize :: proc(q: R3D_Quat) -> R3D_Quat {
    l := math.sqrt(r3d_quat_dot(q, q))
    if l < 1e-12 { return R3D_QUAT_IDENTITY }
    return q / l
}

r3d_quat_axis_angle :: proc(axis: [3]f32, angle: f32) -> R3D_Quat {
    l := math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z)
    if l < 1e-12 { return R3D_QUAT_IDENTITY }
    s := math.sin(angle * 0.5) / l
    return {axis.x * s, axis.y * s, axis.z * s, math.cos(angle * 0.5)}
}

// Rotate a vector by a unit quaternion.
r3d_quat_rotate :: proc(q: R3D_Quat, v: [3]f32) -> [3]f32 {
    u := [3]f32{q.x, q.y, q.z}
    t := 2 * linalg.cross(u, v)
    return v + q.w * t + linalg.cross(u, t)
}

// Shortest-path spherical interpolation; falls back to nlerp when the
// quaternions are nearly parallel.
r3d_quat_slerp :: proc(a, b: R3D_Quat, t: f32) -> R3D_Quat {
    bb := b
    d := r3d_quat_dot(a, b)
    if d < 0 { bb = -b; d = -d }
    if d > 0.9995 {
        return r3d_quat_normalize(a + (bb - a) * t)
    }
    theta := math.acos(clamp(d, -1, 1))
    st := math.sin(theta)
    wa := math.sin((1 - t) * theta) / st
    wb := math.sin(t * theta) / st
    return r3d_quat_normalize(a * wa + bb * wb)
}

// ----------------------------------------------------------------------------
// TRS
// ----------------------------------------------------------------------------

// Column-major model matrix T * R * S. (Odin: m[row, col].)
r3d_trs_matrix :: proc(x: R3D_Trs) -> (m: linalg.Matrix4f32) {
    q := x.r
    xx, yy, zz := q.x * q.x, q.y * q.y, q.z * q.z
    xy, xz, yz := q.x * q.y, q.x * q.z, q.y * q.z
    wx, wy, wz := q.w * q.x, q.w * q.y, q.w * q.z
    m[0, 0] = (1 - 2 * (yy + zz)) * x.s.x
    m[1, 0] = (2 * (xy + wz)) * x.s.x
    m[2, 0] = (2 * (xz - wy)) * x.s.x
    m[0, 1] = (2 * (xy - wz)) * x.s.y
    m[1, 1] = (1 - 2 * (xx + zz)) * x.s.y
    m[2, 1] = (2 * (yz + wx)) * x.s.y
    m[0, 2] = (2 * (xz + wy)) * x.s.z
    m[1, 2] = (2 * (yz - wx)) * x.s.z
    m[2, 2] = (1 - 2 * (xx + yy)) * x.s.z
    m[0, 3] = x.t.x
    m[1, 3] = x.t.y
    m[2, 3] = x.t.z
    m[3, 3] = 1
    return
}

r3d_trs_lerp :: proc(a, b: R3D_Trs, w: f32) -> R3D_Trs {
    return R3D_Trs{
        t = a.t + (b.t - a.t) * w,
        r = r3d_quat_slerp(a.r, b.r, w),
        s = a.s + (b.s - a.s) * w,
    }
}

r3d_mat4_transform_point :: proc(m: linalg.Matrix4f32, p: [3]f32) -> [3]f32 {
    v := m * linalg.Vector4f32{p.x, p.y, p.z, 1}
    return v.xyz
}

// ----------------------------------------------------------------------------
// Skeleton (node hierarchy) and skins
// ----------------------------------------------------------------------------

R3D_Skeleton :: struct {
    names:   [dynamic]string,
    parents: [dynamic]int,     // -1 for roots; always < own index
    rest:    [dynamic]R3D_Trs, // local rest pose (the file's node TRS)
}

R3D_Skin :: struct {
    name:         string,
    joints:       [dynamic]int,                 // skeleton node indices
    inverse_bind: [dynamic]linalg.Matrix4f32,   // one per joint
}

r3d_skeleton_find :: proc(s: ^R3D_Skeleton, name: string) -> int {
    for n, i in s.names { if n == name { return i } }
    return -1
}

// Local pose -> global (model-space) node matrices. Requires parents to be
// ordered before children (r3d gltf loader guarantees it).
r3d_pose_global :: proc(s: ^R3D_Skeleton, local: []R3D_Trs, out: []linalg.Matrix4f32) {
    for i in 0 ..< len(s.parents) {
        m := r3d_trs_matrix(local[i])
        p := s.parents[i]
        out[i] = p >= 0 ? out[p] * m : m
    }
}

// Skinning matrices for one skin: global(joint) * inverse_bind.
r3d_skin_matrices :: proc(skin: ^R3D_Skin, global: []linalg.Matrix4f32, out: []linalg.Matrix4f32) {
    for j in 0 ..< len(skin.joints) {
        out[j] = global[skin.joints[j]] * skin.inverse_bind[j]
    }
}

// CPU reference skinning of one point (linear blend skinning). Weights are
// renormalised; zero total weight returns the point unchanged.
r3d_skin_point :: proc(mats: []linalg.Matrix4f32, p: [3]f32, joints: [4]u16, weights: [4]f32) -> [3]f32 {
    total := weights[0] + weights[1] + weights[2] + weights[3]
    if total <= 1e-8 { return p }
    acc: [3]f32
    for k in 0 ..< 4 {
        w := weights[k] / total
        if w == 0 { continue }
        acc += r3d_mat4_transform_point(mats[joints[k]], p) * w
    }
    return acc
}

// ----------------------------------------------------------------------------
// Animation tracks and clips
// ----------------------------------------------------------------------------

R3D_Anim_Path :: enum u8 { Translation, Rotation, Scale }
R3D_Anim_Interp :: enum u8 { Linear, Step, Cubic }

R3D_Anim_Track :: struct {
    node:   int,
    path:   R3D_Anim_Path,
    interp: R3D_Anim_Interp,
    times:  []f32,
    // comps (3 or 4) floats per key; cubic: 3 * comps per key in the glTF
    // order {in_tangent, value, out_tangent}.
    values: []f32,
}

R3D_Anim_Clip :: struct {
    name:     string,
    duration: f32,
    tracks:   [dynamic]R3D_Anim_Track,
}

r3d_anim_clip_destroy :: proc(c: ^R3D_Anim_Clip) {
    for &t in c.tracks {
        delete(t.times)
        delete(t.values)
    }
    delete(c.tracks)
    delete(c.name)
}

@(private)
r3d_track_comps :: proc(t: ^R3D_Anim_Track) -> int {
    return t.path == .Rotation ? 4 : 3
}

// Index k of the last key with times[k] <= tt (binary search), clamped so
// k+1 is valid when there are >= 2 keys.
@(private)
r3d_track_find_key :: proc(times: []f32, tt: f32) -> int {
    lo, hi := 0, len(times) - 1
    if hi <= 0 { return 0 }
    if tt >= times[hi] { return hi - 1 }
    for hi - lo > 1 {
        mid := (lo + hi) / 2
        if times[mid] <= tt { lo = mid } else { hi = mid }
    }
    return lo
}

// Sample a track at time t (clamped to its key range). Returns 3 or 4 floats
// in a [4]f32 (rotations normalised).
r3d_track_sample :: proc(tr: ^R3D_Anim_Track, t: f32) -> (v: [4]f32) {
    n := len(tr.times)
    comps := r3d_track_comps(tr)
    if n == 0 { return tr.path == .Rotation ? [4]f32{0, 0, 0, 1} : (tr.path == .Scale ? [4]f32{1, 1, 1, 0} : {}) }
    cubic := tr.interp == .Cubic
    stride := cubic ? comps * 3 : comps
    val_off := cubic ? comps : 0
    if n == 1 || t <= tr.times[0] {
        for c in 0 ..< comps { v[c] = tr.values[val_off + c] }
        if tr.path == .Rotation { v = r3d_quat_normalize(v) }
        return
    }
    if t >= tr.times[n - 1] {
        for c in 0 ..< comps { v[c] = tr.values[(n - 1) * stride + val_off + c] }
        if tr.path == .Rotation { v = r3d_quat_normalize(v) }
        return
    }
    k := r3d_track_find_key(tr.times, t)
    t0, t1 := tr.times[k], tr.times[k + 1]
    dt := max(t1 - t0, 1e-6)
    u := clamp((t - t0) / dt, 0, 1)

    switch tr.interp {
    case .Step:
        for c in 0 ..< comps { v[c] = tr.values[k * stride + c] }
    case .Linear:
        if tr.path == .Rotation {
            a := R3D_Quat{tr.values[k * 4], tr.values[k * 4 + 1], tr.values[k * 4 + 2], tr.values[k * 4 + 3]}
            b := R3D_Quat{tr.values[k * 4 + 4], tr.values[k * 4 + 5], tr.values[k * 4 + 6], tr.values[k * 4 + 7]}
            return r3d_quat_slerp(r3d_quat_normalize(a), r3d_quat_normalize(b), u)
        }
        for c in 0 ..< comps {
            a := tr.values[k * stride + c]
            b := tr.values[(k + 1) * stride + c]
            v[c] = a + (b - a) * u
        }
    case .Cubic:
        // glTF cubic Hermite: p(u) = h00 p0 + h10 dt m0 + h01 p1 + h11 dt m1,
        // m0 = out-tangent of key k, m1 = in-tangent of key k+1.
        u2 := u * u
        u3 := u2 * u
        h00 := 2 * u3 - 3 * u2 + 1
        h10 := u3 - 2 * u2 + u
        h01 := -2 * u3 + 3 * u2
        h11 := u3 - u2
        for c in 0 ..< comps {
            p0 := tr.values[k * stride + comps + c]
            m0 := tr.values[k * stride + 2 * comps + c]
            p1 := tr.values[(k + 1) * stride + comps + c]
            m1 := tr.values[(k + 1) * stride + c]
            v[c] = h00 * p0 + h10 * dt * m0 + h01 * p1 + h11 * dt * m1
        }
    }
    if tr.path == .Rotation { v = r3d_quat_normalize(v) }
    return
}

// Wrap or clamp a clip time.
r3d_anim_clip_time :: proc(clip: ^R3D_Anim_Clip, t: f32, loop: bool) -> f32 {
    d := clip.duration
    if d <= 0 { return 0 }
    if loop {
        r := math.mod(t, d)
        if r < 0 { r += d }
        return r
    }
    return clamp(t, 0, d)
}

// Sample a clip into a local pose. Nodes without tracks keep the rest pose.
r3d_anim_clip_sample :: proc(s: ^R3D_Skeleton, clip: ^R3D_Anim_Clip, t: f32, loop: bool, out: []R3D_Trs) {
    copy(out, s.rest[:])
    if clip == nil { return }
    tt := r3d_anim_clip_time(clip, t, loop)
    for &tr in clip.tracks {
        if tr.node < 0 || tr.node >= len(out) { continue }
        v := r3d_track_sample(&tr, tt)
        switch tr.path {
        case .Translation: out[tr.node].t = v.xyz
        case .Rotation:    out[tr.node].r = v
        case .Scale:       out[tr.node].s = v.xyz
        }
    }
}

// out = lerp(a, b, w) per node (slerp rotations). out may alias a.
r3d_pose_blend :: proc(a, b: []R3D_Trs, w: f32, out: []R3D_Trs) {
    for i in 0 ..< len(out) {
        out[i] = r3d_trs_lerp(a[i], b[i], w)
    }
}

// ----------------------------------------------------------------------------
// Animation player: one playing clip plus an optional fading-out clip.
// ----------------------------------------------------------------------------

R3D_Anim_State :: struct {
    clip:  int, // index into the clip list, -1 = rest pose
    time:  f32,
    speed: f32,
    loop:  bool,
}

R3D_Anim_Player :: struct {
    cur:       R3D_Anim_State,
    prev:      R3D_Anim_State,
    fading:    bool,
    fade_t:    f32,
    fade_dur:  f32,
    // When true, a crossfade starts the new clip at the old clip's normalised
    // phase (walk <-> run keeps the feet in step).
    sync_phase: bool,
    // scratch poses (sized on first evaluate)
    scratch_a, scratch_b: [dynamic]R3D_Trs,
}

r3d_anim_player_init :: proc(p: ^R3D_Anim_Player, clip: int, loop := true) {
    p.cur = R3D_Anim_State{clip = clip, time = 0, speed = 1, loop = loop}
    p.prev = {}
    p.prev.clip = -1
    p.fading = false
}

r3d_anim_player_destroy :: proc(p: ^R3D_Anim_Player) {
    delete(p.scratch_a)
    delete(p.scratch_b)
}

// Crossfade to `clip` over `fade` seconds. Playing the current clip again is
// a no-op (only speed/loop update). fade <= 0 cuts.
r3d_anim_play :: proc(p: ^R3D_Anim_Player, clips: []R3D_Anim_Clip, clip: int, fade: f32, loop := true, speed: f32 = 1) {
    if clip == p.cur.clip {
        p.cur.speed = speed
        p.cur.loop = loop
        return
    }
    start: f32 = 0
    if p.sync_phase && p.cur.clip >= 0 && clip >= 0 && clip < len(clips) && p.cur.clip < len(clips) {
        a := &clips[p.cur.clip]
        if a.duration > 0 {
            phase := r3d_anim_clip_time(a, p.cur.time, p.cur.loop) / a.duration
            start = phase * clips[clip].duration
        }
    }
    if fade > 0 {
        p.prev = p.cur
        p.fading = true
        p.fade_t = 0
        p.fade_dur = fade
    } else {
        p.fading = false
    }
    p.cur = R3D_Anim_State{clip = clip, time = start, speed = speed, loop = loop}
}

r3d_anim_update :: proc(p: ^R3D_Anim_Player, dt: f32) {
    p.cur.time += dt * p.cur.speed
    if p.fading {
        p.prev.time += dt * p.prev.speed
        p.fade_t += dt
        if p.fade_t >= p.fade_dur { p.fading = false }
    }
}

// Current crossfade weight of the incoming clip, smoothstepped (1 = done).
r3d_anim_fade_weight :: proc(p: ^R3D_Anim_Player) -> f32 {
    if !p.fading || p.fade_dur <= 0 { return 1 }
    x := clamp(p.fade_t / p.fade_dur, 0, 1)
    return x * x * (3 - 2 * x)
}

r3d_anim_evaluate :: proc(p: ^R3D_Anim_Player, s: ^R3D_Skeleton, clips: []R3D_Anim_Clip, out: []R3D_Trs) {
    get :: proc(clips: []R3D_Anim_Clip, i: int) -> ^R3D_Anim_Clip {
        return i >= 0 && i < len(clips) ? &clips[i] : nil
    }
    r3d_anim_clip_sample(s, get(clips, p.cur.clip), p.cur.time, p.cur.loop, out)
    if !p.fading { return }
    n := len(s.rest)
    if len(p.scratch_a) != n { resize(&p.scratch_a, n) }
    r3d_anim_clip_sample(s, get(clips, p.prev.clip), p.prev.time, p.prev.loop, p.scratch_a[:])
    r3d_pose_blend(p.scratch_a[:], out, r3d_anim_fade_weight(p), out)
}

// Locomotion blend: sample two looping clips at one shared normalised phase
// and mix them (w = 0 -> a, 1 -> b). Used for speed-driven walk/run.
r3d_anim_sample_blend2 :: proc(s: ^R3D_Skeleton, a, b: ^R3D_Anim_Clip, phase: f32, w: f32, scratch: []R3D_Trs, out: []R3D_Trs) {
    ph := phase - math.floor(phase)
    r3d_anim_clip_sample(s, a, ph * a.duration, true, out)
    r3d_anim_clip_sample(s, b, ph * b.duration, true, scratch)
    r3d_pose_blend(out, scratch, w, out)
}

r3d_anim_find_clip :: proc(clips: []R3D_Anim_Clip, name: string) -> int {
    for c, i in clips { if c.name == name { return i } }
    return -1
}
