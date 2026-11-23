package voidengine

// ============================================================================
// csm3d — cascaded shadow maps for the sun, backend-agnostic math.
//
// The view frustum from near to shadow_far is cut into R3D_CSM_CASCADES
// slices (practical split: lambda blends logarithmic and uniform). Each slice
// is wrapped in a bounding sphere, so its ortho box does not change size as
// the camera turns, and the box origin is snapped to whole shadow-map texels,
// so the shadow edges do not crawl as the camera moves.
// ============================================================================

import "core:math"
import "core:math/linalg"

R3D_CSM_CASCADES :: 3

R3D_Csm :: struct {
    res:         int,  // shadow map texels per side, per cascade
    lambda:      f32,  // 0 = uniform splits, 1 = logarithmic
    shadow_far:  f32,  // shadows end here (view depth, metres)
    caster_back: f32,  // extra depth toward the sun for off-screen casters
    // results of r3d_csm_update
    splits:      [R3D_CSM_CASCADES + 1]f32, // view depths: near, ..., shadow_far
    light_vp:    [R3D_CSM_CASCADES]linalg.Matrix4f32,
    radius:      [R3D_CSM_CASCADES]f32,
    texel_world: [R3D_CSM_CASCADES]f32, // metres per shadow texel
}

r3d_csm_default :: proc "contextless" () -> R3D_Csm {
    return {res = 2048, lambda = 0.8, shadow_far = 260, caster_back = 250}
}

// Split depths for n cascades between near and far.
r3d_csm_splits :: proc "contextless" (near_z, far_z: f32, lambda: f32) -> (s: [R3D_CSM_CASCADES + 1]f32) {
    n :: R3D_CSM_CASCADES
    s[0] = near_z
    for i in 1 ..< n {
        f := f32(i) / f32(n)
        log_s := near_z * math.pow(far_z / near_z, f)
        uni_s := near_z + (far_z - near_z) * f
        s[i] = lambda * log_s + (1 - lambda) * uni_s
    }
    s[n] = far_z
    return
}

// World-space corners of the camera frustum slice between view depths d0, d1.
r3d_frustum_slice_corners :: proc "contextless" (cam: ^R3D_Camera, aspect, d0, d1: f32) -> (c: [8]linalg.Vector3f32) {
    right, up, fwd := r3d_camera_basis(cam)
    t := math.tan(cam.fov_y * 0.5)
    k := 0
    for d in ([2]f32{d0, d1}) {
        center := cam.position + fwd * d
        h := d * t
        w := h * aspect
        for sy in ([2]f32{-1, 1}) {
            for sx in ([2]f32{-1, 1}) {
                c[k] = center + right * (w * sx) + up * (h * sy)
                k += 1
            }
        }
    }
    return
}

// Recompute splits and the light matrices for this camera and sun direction
// (the direction light travels). GL clip depth.
r3d_csm_update :: proc "contextless" (csm: ^R3D_Csm, cam: ^R3D_Camera, aspect: f32, sun_dir: linalg.Vector3f32, vulkan := false) {
    far_z := min(csm.shadow_far, cam.far_z)
    csm.splits = r3d_csm_splits(cam.near_z, far_z, csm.lambda)
    L := linalg.normalize(sun_dir)
    up := abs(L.y) > 0.99 ? linalg.Vector3f32{0, 0, 1} : linalg.Vector3f32{0, 1, 0}
    res := f32(max(csm.res, 1))
    for i in 0 ..< R3D_CSM_CASCADES {
        corners := r3d_frustum_slice_corners(cam, aspect, csm.splits[i], csm.splits[i + 1])
        center: linalg.Vector3f32
        for p in corners { center += p }
        center /= 8
        radius: f32 = 0
        for p in corners { radius = max(radius, linalg.length(p - center)) }
        // quantise the radius so the box size (and texel size) stays put
        radius = math.ceil(radius * 4) / 4
        back := radius + csm.caster_back
        eye := center - L * back
        view := linalg.matrix4_look_at(eye, center, up)
        proj := vulkan ? r3d_ortho_vk(-radius, radius, -radius, radius, 0, back + radius) :
                         r3d_ortho_gl(-radius, radius, -radius, radius, 0, back + radius)
        vp := proj * view
        // snap: move the projection so the world origin lands on a texel corner
        o := vp * linalg.Vector4f32{0, 0, 0, 1}
        half := res * 0.5
        ox := o.x * half
        oy := o.y * half
        dx := (math.round(ox) - ox) / half
        dy := (math.round(oy) - oy) / half
        vp[0, 3] += dx
        vp[1, 3] += dy
        csm.light_vp[i] = vp
        csm.radius[i] = radius
        csm.texel_world[i] = 2 * radius / res
    }
}

// Which cascade covers a view depth (-1 past the last one).
r3d_csm_cascade_for_depth :: proc "contextless" (csm: ^R3D_Csm, depth: f32) -> int {
    for i in 0 ..< R3D_CSM_CASCADES {
        if depth <= csm.splits[i + 1] { return i }
    }
    return -1
}
