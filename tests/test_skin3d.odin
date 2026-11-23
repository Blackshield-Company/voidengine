package voidengine_test

// Tests for skin3d.odin: quaternion/TRS math, global poses, skinning
// matrices, linear blend skinning, track sampling (step / linear / cubic),
// clip time wrapping, pose blending and the crossfading animation player.

import "core:math"
import "core:testing"
import "core:math/linalg"
import engine "../src/core"

@(private)
near :: proc(a, b: f32, eps: f32 = 1e-4) -> bool { return math.abs(a - b) <= eps }

@(private)
near3 :: proc(a, b: [3]f32, eps: f32 = 1e-4) -> bool {
    return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps)
}

@(private)
quat_same :: proc(a, b: engine.R3D_Quat, eps: f32 = 1e-4) -> bool {
    // q and -q are the same rotation
    return math.abs(math.abs(engine.r3d_quat_dot(a, b)) - 1) <= eps
}

@(test)
test_quat_rotate_and_mul :: proc(t: ^testing.T) {
    qz := engine.r3d_quat_axis_angle({0, 0, 1}, math.PI / 2)
    testing.expect(t, near3(engine.r3d_quat_rotate(qz, {1, 0, 0}), {0, 1, 0}))
    qy := engine.r3d_quat_axis_angle({0, 1, 0}, math.PI / 2)
    // (qz * qy) v = qz (qy v): x -> -z -> -z (z axis unchanged by qz)
    q := engine.r3d_quat_mul(qz, qy)
    testing.expect(t, near3(engine.r3d_quat_rotate(q, {1, 0, 0}), {0, 0, -1}))
    testing.expect(t, near3(engine.r3d_quat_rotate(q, {0, 1, 0}), engine.r3d_quat_rotate(qz, {0, 1, 0})))
}

@(test)
test_quat_slerp :: proc(t: ^testing.T) {
    a := engine.R3D_QUAT_IDENTITY
    b := engine.r3d_quat_axis_angle({0, 1, 0}, math.PI / 2)
    testing.expect(t, quat_same(engine.r3d_quat_slerp(a, b, 0), a))
    testing.expect(t, quat_same(engine.r3d_quat_slerp(a, b, 1), b))
    mid := engine.r3d_quat_slerp(a, b, 0.5)
    testing.expect(t, quat_same(mid, engine.r3d_quat_axis_angle({0, 1, 0}, math.PI / 4)))
    // constant angular speed: quarter point is 22.5 deg (nlerp would not be)
    q1 := engine.r3d_quat_slerp(a, b, 0.25)
    testing.expect(t, quat_same(q1, engine.r3d_quat_axis_angle({0, 1, 0}, math.PI / 8), 1e-5))
    // shortest path: -b is the same rotation, the midpoint must not swing the long way
    mid2 := engine.r3d_quat_slerp(a, -b, 0.5)
    testing.expect(t, quat_same(mid2, mid))
    l := engine.r3d_quat_dot(mid, mid)
    testing.expect(t, near(l, 1))
}

@(test)
test_trs_matrix_roundtrip :: proc(t: ^testing.T) {
    x := engine.R3D_Trs{
        t = {1, 2, 3},
        r = engine.r3d_quat_axis_angle(linalg.normalize([3]f32{1, 2, 0.5}), 0.9),
        s = {2, 0.5, 1.5},
    }
    m := engine.r3d_trs_matrix(x)
    // T * R * S applied to a point
    p := [3]f32{0.3, -1, 2}
    want := x.t + engine.r3d_quat_rotate(x.r, p * x.s)
    testing.expect(t, near3(engine.r3d_mat4_transform_point(m, p), want))
    y := engine.r3d_trs_from_matrix(m)
    testing.expect(t, near3(y.t, x.t))
    testing.expect(t, near3(y.s, x.s))
    testing.expect(t, quat_same(y.r, x.r))
}

// Two-bone chain: root at origin, child 1 m up. Returns skeleton + skin
// with inverse binds from the rest pose.
@(private)
make_chain :: proc() -> (s: engine.R3D_Skeleton, skin: engine.R3D_Skin) {
    append(&s.names, "root", "child")
    append(&s.parents, -1, 0)
    append(&s.rest, engine.R3D_TRS_IDENTITY, engine.R3D_Trs{t = {0, 1, 0}, r = engine.R3D_QUAT_IDENTITY, s = {1, 1, 1}})
    append(&skin.joints, 0, 1)
    g: [2]linalg.Matrix4f32
    engine.r3d_pose_global(&s, s.rest[:], g[:])
    append(&skin.inverse_bind, linalg.inverse(g[0]), linalg.inverse(g[1]))
    return
}

@(private)
free_chain :: proc(s: ^engine.R3D_Skeleton, skin: ^engine.R3D_Skin) {
    delete(s.names); delete(s.parents); delete(s.rest)
    delete(skin.joints); delete(skin.inverse_bind)
}

@(test)
test_pose_global_and_rest_skinning :: proc(t: ^testing.T) {
    s, skin := make_chain()
    defer free_chain(&s, &skin)
    pose := [2]engine.R3D_Trs{s.rest[0], s.rest[1]}
    pose[0].r = engine.r3d_quat_axis_angle({0, 0, 1}, math.PI / 2) // root turns 90 deg about Z
    g: [2]linalg.Matrix4f32
    engine.r3d_pose_global(&s, pose[:], g[:])
    // child origin (0,1,0) rotated about Z -> (-1,0,0)
    testing.expect(t, near3(engine.r3d_mat4_transform_point(g[1], {0, 0, 0}), {-1, 0, 0}))

    // rest pose: every skinning matrix is identity, vertices stay put
    rest_g: [2]linalg.Matrix4f32
    engine.r3d_pose_global(&s, s.rest[:], rest_g[:])
    mats: [2]linalg.Matrix4f32
    engine.r3d_skin_matrices(&skin, rest_g[:], mats[:])
    for m in mats {
        for r in 0 ..< 4 { for c in 0 ..< 4 {
            testing.expect(t, near(m[r, c], r == c ? 1 : 0))
        } }
    }
    p := [3]f32{0.2, 1.7, -0.1}
    testing.expect(t, near3(engine.r3d_skin_point(mats[:], p, {0, 1, 0, 0}, {0.3, 0.7, 0, 0}), p))
}

@(test)
test_linear_blend_skinning :: proc(t: ^testing.T) {
    s, skin := make_chain()
    defer free_chain(&s, &skin)
    pose := [2]engine.R3D_Trs{s.rest[0], s.rest[1]}
    pose[1].r = engine.r3d_quat_axis_angle({0, 0, 1}, math.PI / 2) // bend the child joint
    g, mats: [2]linalg.Matrix4f32
    engine.r3d_pose_global(&s, pose[:], g[:])
    engine.r3d_skin_matrices(&skin, g[:], mats[:])
    v := [3]f32{0, 1.5, 0}
    // fully on the child: (0,0.5) above the joint swings to (-0.5, 0) -> (-0.5, 1, 0)
    testing.expect(t, near3(engine.r3d_skin_point(mats[:], v, {1, 0, 0, 0}, {1, 0, 0, 0}), {-0.5, 1, 0}))
    // half/half: average of (0,1.5,0) and (-0.5,1,0)
    testing.expect(t, near3(engine.r3d_skin_point(mats[:], v, {0, 1, 0, 0}, {0.5, 0.5, 0, 0}), {-0.25, 1.25, 0}))
    // weights are renormalised (0.2 + 0.2 == 0.5 + 0.5)
    testing.expect(t, near3(engine.r3d_skin_point(mats[:], v, {0, 1, 0, 0}, {0.2, 0.2, 0, 0}), {-0.25, 1.25, 0}))
    // zero weights: unchanged
    testing.expect(t, near3(engine.r3d_skin_point(mats[:], v, {0, 0, 0, 0}, {0, 0, 0, 0}), v))
}

@(test)
test_track_sampling_modes :: proc(t: ^testing.T) {
    times := []f32{0, 1, 3}
    lin := engine.R3D_Anim_Track{node = 0, path = .Translation, interp = .Linear, times = times,
        values = []f32{0, 0, 0, 10, 0, 0, 10, 4, 0}}
    testing.expect(t, near3(engine.r3d_track_sample(&lin, 0.5).xyz, {5, 0, 0}))
    testing.expect(t, near3(engine.r3d_track_sample(&lin, 2).xyz, {10, 2, 0}))
    testing.expect(t, near3(engine.r3d_track_sample(&lin, -1).xyz, {0, 0, 0}))  // clamp before
    testing.expect(t, near3(engine.r3d_track_sample(&lin, 9).xyz, {10, 4, 0}))  // clamp after
    testing.expect(t, near3(engine.r3d_track_sample(&lin, 1).xyz, {10, 0, 0}))  // exactly on a key

    step := lin
    step.interp = .Step
    testing.expect(t, near3(engine.r3d_track_sample(&step, 0.99).xyz, {0, 0, 0}))
    testing.expect(t, near3(engine.r3d_track_sample(&step, 2.5).xyz, {10, 0, 0}))

    // cubic spline, zero tangents: Hermite reduces to smoothstep between keys
    ct := []f32{0, 2}
    cub := engine.R3D_Anim_Track{node = 0, path = .Translation, interp = .Cubic, times = ct,
        values = []f32{
            0, 0, 0,  0, 0, 0,  0, 0, 0, // key 0: in, value, out
            0, 0, 0,  8, 0, 0,  0, 0, 0, // key 1
        }}
    testing.expect(t, near3(engine.r3d_track_sample(&cub, 1).xyz, {4, 0, 0}))
    testing.expect(t, near(engine.r3d_track_sample(&cub, 0.5).x, 8 * (3 * 0.0625 - 2 * 0.015625)))
    // with tangents: out-tangent of key 0 = 3/s, dt = 2; at u = .5:
    // h00 p0 + h10 dt m0 + h01 p1 = 0 + 0.125*2*3 + 0.5*8
    cub.values[2 * 3 + 0] = 3
    testing.expect(t, near(engine.r3d_track_sample(&cub, 1).x, 0.75 + 4))

    // rotations slerp and stay unit length
    rt := []f32{0, 1}
    q90 := engine.r3d_quat_axis_angle({0, 1, 0}, math.PI / 2)
    rot := engine.R3D_Anim_Track{node = 0, path = .Rotation, interp = .Linear, times = rt,
        values = []f32{0, 0, 0, 1, q90.x, q90.y, q90.z, q90.w}}
    testing.expect(t, quat_same(engine.r3d_track_sample(&rot, 0.5), engine.r3d_quat_axis_angle({0, 1, 0}, math.PI / 4)))
    // constant angular speed across a wide key gap (nlerp would lag at 1/4)
    q160 := engine.r3d_quat_axis_angle({0, 1, 0}, math.to_radians(f32(160)))
    rot.values = []f32{0, 0, 0, 1, q160.x, q160.y, q160.z, q160.w}
    testing.expect(t, quat_same(engine.r3d_track_sample(&rot, 0.25), engine.r3d_quat_axis_angle({0, 1, 0}, math.to_radians(f32(40))), 1e-5))
}

@(test)
test_clip_time_wrap :: proc(t: ^testing.T) {
    c := engine.R3D_Anim_Clip{duration = 2}
    testing.expect(t, near(engine.r3d_anim_clip_time(&c, 5, true), 1))
    testing.expect(t, near(engine.r3d_anim_clip_time(&c, -0.5, true), 1.5))
    testing.expect(t, near(engine.r3d_anim_clip_time(&c, 5, false), 2))
}

// Two one-track clips moving node 1 along X: "a" to x=1, "b" to x=-1 (held).
@(private)
make_clips :: proc() -> [2]engine.R3D_Anim_Clip {
    clips: [2]engine.R3D_Anim_Clip
    clips[0].name = "a"; clips[0].duration = 1
    append(&clips[0].tracks, engine.R3D_Anim_Track{node = 1, path = .Translation, interp = .Linear,
        times = make([]f32, 2), values = make([]f32, 6)})
    clips[0].tracks[0].times[1] = 1
    clips[0].tracks[0].values[0] = 1; clips[0].tracks[0].values[3] = 1
    clips[1].name = "b"; clips[1].duration = 2
    append(&clips[1].tracks, engine.R3D_Anim_Track{node = 1, path = .Translation, interp = .Linear,
        times = make([]f32, 2), values = make([]f32, 6)})
    clips[1].tracks[0].times[1] = 2
    clips[1].tracks[0].values[0] = -1; clips[1].tracks[0].values[3] = -1
    return clips
}

@(private)
free_clips :: proc(c: ^[2]engine.R3D_Anim_Clip) {
    for &x in c {
        for &tr in x.tracks { delete(tr.times); delete(tr.values) }
        delete(x.tracks)
    }
}

@(test)
test_clip_sample_keeps_rest_for_untracked :: proc(t: ^testing.T) {
    s, skin := make_chain()
    defer free_chain(&s, &skin)
    clips := make_clips()
    defer free_clips(&clips)
    out: [2]engine.R3D_Trs
    engine.r3d_anim_clip_sample(&s, &clips[0], 0.3, true, out[:])
    testing.expect(t, near3(out[0].t, {0, 0, 0}))  // rest
    testing.expect(t, near3(out[1].t, {1, 0, 0}))  // tracked (replaces rest t)
    testing.expect(t, quat_same(out[1].r, engine.R3D_QUAT_IDENTITY))
}

@(test)
test_pose_blend :: proc(t: ^testing.T) {
    a := []engine.R3D_Trs{{t = {0, 0, 0}, r = engine.R3D_QUAT_IDENTITY, s = {1, 1, 1}}}
    b := []engine.R3D_Trs{{t = {4, 0, 0}, r = engine.r3d_quat_axis_angle({1, 0, 0}, math.PI / 2), s = {3, 3, 3}}}
    out := make([]engine.R3D_Trs, 1)
    defer delete(out)
    engine.r3d_pose_blend(a, b, 0.25, out)
    testing.expect(t, near3(out[0].t, {1, 0, 0}))
    testing.expect(t, near3(out[0].s, {1.5, 1.5, 1.5}))
    testing.expect(t, quat_same(out[0].r, engine.r3d_quat_axis_angle({1, 0, 0}, math.PI / 8)))
}

@(test)
test_crossfade_player :: proc(t: ^testing.T) {
    s, skin := make_chain()
    defer free_chain(&s, &skin)
    clips := make_clips()
    defer free_clips(&clips)
    p: engine.R3D_Anim_Player
    defer engine.r3d_anim_player_destroy(&p)
    engine.r3d_anim_player_init(&p, 0)
    out: [2]engine.R3D_Trs
    engine.r3d_anim_update(&p, 0.1)
    engine.r3d_anim_evaluate(&p, &s, clips[:], out[:])
    testing.expect(t, near(out[1].t.x, 1))

    engine.r3d_anim_play(&p, clips[:], 1, 0.2)
    testing.expect(t, p.fading)
    engine.r3d_anim_update(&p, 0.1) // halfway: smoothstep(0.5) = 0.5
    testing.expect(t, near(engine.r3d_anim_fade_weight(&p), 0.5))
    engine.r3d_anim_evaluate(&p, &s, clips[:], out[:])
    testing.expect(t, near(out[1].t.x, 0))       // halfway between +1 and -1
    testing.expect(t, near(out[0].t.x, 0))       // untracked node stays at rest
    engine.r3d_anim_update(&p, 0.05)              // 0.15 / 0.2 = 0.75 -> 0.84375
    testing.expect(t, near(engine.r3d_anim_fade_weight(&p), 0.84375))
    engine.r3d_anim_update(&p, 0.2)
    testing.expect(t, !p.fading)
    engine.r3d_anim_evaluate(&p, &s, clips[:], out[:])
    testing.expect(t, near(out[1].t.x, -1))

    // replaying the current clip is a no-op (no fade, time kept)
    before := p.cur.time
    engine.r3d_anim_play(&p, clips[:], 1, 0.3, true, 2)
    testing.expect(t, !p.fading)
    testing.expect(t, near(p.cur.time, before))
    testing.expect(t, near(p.cur.speed, 2))

    // cut (fade 0)
    engine.r3d_anim_play(&p, clips[:], 0, 0)
    testing.expect(t, !p.fading)
    engine.r3d_anim_evaluate(&p, &s, clips[:], out[:])
    testing.expect(t, near(out[1].t.x, 1))
}

@(test)
test_crossfade_sync_phase :: proc(t: ^testing.T) {
    clips := make_clips()
    defer free_clips(&clips)
    p: engine.R3D_Anim_Player
    defer engine.r3d_anim_player_destroy(&p)
    engine.r3d_anim_player_init(&p, 0)
    p.sync_phase = true
    engine.r3d_anim_update(&p, 0.25) // 25 % through the 1 s clip
    engine.r3d_anim_play(&p, clips[:], 1, 0.2)
    testing.expect(t, near(p.cur.time, 0.5)) // 25 % of the 2 s clip
}

@(test)
test_sample_blend2_shared_phase :: proc(t: ^testing.T) {
    s, skin := make_chain()
    defer free_chain(&s, &skin)
    clips := make_clips()
    defer free_clips(&clips)
    out, scratch: [2]engine.R3D_Trs
    engine.r3d_anim_sample_blend2(&s, &clips[0], &clips[1], 1.25, 0.75, scratch[:], out[:])
    testing.expect(t, near(out[1].t.x, 0.25 * 1 + 0.75 * -1))
}
