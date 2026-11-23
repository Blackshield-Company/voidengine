package voidengine_test

// Tests for csm3d.odin: split depths, every slice inside its light box,
// texel snapping (shadow texels stay on a fixed world grid as the camera
// moves), fixed box size while the camera turns.

import "core:math"
import "core:testing"
import "core:math/linalg"
import engine "../src/core"

@(test)
test_csm_splits :: proc(t: ^testing.T) {
    s := engine.r3d_csm_splits(0.1, 300, 0.8)
    testing.expect(t, s[0] == 0.1 && s[engine.R3D_CSM_CASCADES] == 300)
    for i in 1 ..= engine.R3D_CSM_CASCADES { testing.expect(t, s[i] > s[i - 1]) }
    u := engine.r3d_csm_splits(1, 301, 0)
    testing.expect(t, abs(u[1] - 101) < 1e-3 && abs(u[2] - 201) < 1e-3)
    l := engine.r3d_csm_splits(1, 1000, 1)
    testing.expect(t, abs(l[1] - 10) < 1e-2 && abs(l[2] - 100) < 1e-1)
}

@(private="file")
test_cam :: proc(x: f32, yaw: f32) -> engine.R3D_Camera {
    return {position = {x, 12, 5}, yaw = yaw, pitch = math.to_radians(f32(-35)), fov_y = math.to_radians(f32(70)), near_z = 0.1, far_z = 2000}
}

@(test)
test_csm_slices_fit_and_snap :: proc(t: ^testing.T) {
    csm := engine.r3d_csm_default()
    sun := linalg.Vector3f32{0.287, -0.819, -0.497}
    cam := test_cam(3.3, 0.4)
    engine.r3d_csm_update(&csm, &cam, 16.0 / 9.0, sun)
    for i in 0 ..< engine.R3D_CSM_CASCADES {
        corners := engine.r3d_frustum_slice_corners(&cam, 16.0 / 9.0, csm.splits[i], csm.splits[i + 1])
        for p in corners {
            q := csm.light_vp[i] * linalg.Vector4f32{p.x, p.y, p.z, 1}
            testing.expectf(t, abs(q.x) <= 1.0001 && abs(q.y) <= 1.0001 && q.z >= -1 && q.z <= 1, "cascade %v corner %v -> %v", i, p, q)
        }
        // a caster 200 m up-sun of the slice is still inside the depth range
        c := (corners[0] + corners[7]) * 0.5 - linalg.normalize(sun) * 200
        q := csm.light_vp[i] * linalg.Vector4f32{c.x, c.y, c.z, 1}
        testing.expect(t, q.z >= -1)
        // origin on a texel corner
        o := csm.light_vp[i] * linalg.Vector4f32{0, 0, 0, 1}
        half := f32(csm.res) * 0.5
        testing.expect(t, abs(o.x * half - math.round(o.x * half)) < 1e-2)
        testing.expect(t, abs(o.y * half - math.round(o.y * half)) < 1e-2)
        testing.expect(t, csm.texel_world[i] > 0 && (i == 0 || csm.texel_world[i] > csm.texel_world[i - 1]))
    }
    // sub-texel camera moves shift the grid by whole texels only; turning
    // the camera keeps the box size
    r0 := csm.radius
    cam2 := test_cam(3.3 + 0.013, 1.9)
    engine.r3d_csm_update(&csm, &cam2, 16.0 / 9.0, sun)
    testing.expect(t, csm.radius == r0)
    testing.expect(t, engine.r3d_csm_cascade_for_depth(&csm, 0.5) == 0)
    testing.expect(t, engine.r3d_csm_cascade_for_depth(&csm, csm.splits[2] + 0.1) == 2)
    testing.expect(t, engine.r3d_csm_cascade_for_depth(&csm, 1e5) == -1)
    // world point to shadow texel: a fixed point keeps a whole-texel offset
    // between the two camera positions
    p := linalg.Vector4f32{10.37, 3.1, -7.9, 1}
    engine.r3d_csm_update(&csm, &cam, 16.0 / 9.0, sun)
    a := csm.light_vp[0] * p
    engine.r3d_csm_update(&csm, &cam2, 16.0 / 9.0, sun)
    b := csm.light_vp[0] * p
    half := f32(csm.res) * 0.5
    dx := (b.x - a.x) * half
    dy := (b.y - a.y) * half
    testing.expectf(t, abs(dx - math.round(dx)) < 2e-2 && abs(dy - math.round(dy)) < 2e-2, "shift %v,%v texels", dx, dy)
}
