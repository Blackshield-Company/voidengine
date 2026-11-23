package voidengine

// ============================================================================
// noise_fnl — a bit-faithful port of the 2D OpenSimplex2S + FBm path of
// FastNoiseLite, the noise Godot's FastNoiseLite resource wraps
// (thirdparty/misc/FastNoiseLite.h). Godot's defaults are mirrored
// (TYPE_SIMPLEX_SMOOTH, FBm, 5 octaves, lacunarity 2, gain 0.5), so a world
// built from Godot noise (Templar Valley's height and ground rules) can be
// rebuilt here without baking data. All math is f32, like Godot's real_t.
//
// FastNoiseLite: MIT License, Copyright (c) 2023 Jordan Peck and contributors.
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions: The above
// copyright notice and this permission notice shall be included in all copies
// or substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS",
// WITHOUT WARRANTY OF ANY KIND.
// ============================================================================

FNL_Fractal :: enum { None, FBm }

FNL_Noise :: struct {
    seed:       i32,
    frequency:  f32,
    fractal:    FNL_Fractal,
    octaves:    i32,
    lacunarity: f32,
    gain:       f32,
    weighted:   f32,
    bounding:   f32,
}

// Godot FastNoiseLite.new() defaults, with the given seed/frequency/fractal.
fnl_godot :: proc(seed: i32, frequency: f32, fractal := FNL_Fractal.FBm, octaves: i32 = 5) -> FNL_Noise {
    n := FNL_Noise{seed = seed, frequency = frequency, fractal = fractal, octaves = octaves, lacunarity = 2, gain = 0.5}
    fnl_update_bounding(&n)
    return n
}

fnl_update_bounding :: proc(n: ^FNL_Noise) {
    gain := abs(n.gain)
    amp := gain
    amp_fractal: f32 = 1
    for _ in 1 ..< n.octaves {
        amp_fractal += amp
        amp *= gain
    }
    n.bounding = 1 / amp_fractal
}

// Godot get_noise_2d (offset 0, no domain warp). Range about -1..1.
fnl_noise_2d :: proc(n: ^FNL_Noise, px, py: f32) -> f32 {
    x := px * n.frequency
    y := py * n.frequency
    sqrt3 := f32(1.7320508075688772935274463415059)
    f2 := f32(0.5) * (sqrt3 - 1)
    t := (x + y) * f2
    x += t
    y += t
    if n.fractal == .None { return fnl_single_os2s(n.seed, x, y) }
    seed := n.seed
    sum: f32 = 0
    amp := n.bounding
    for _ in 0 ..< n.octaves {
        v := fnl_single_os2s(seed, x, y)
        seed += 1
        sum += v * amp
        amp *= fnl_lerp(1, min(v + 1, 2) * 0.5, n.weighted)
        x *= n.lacunarity
        y *= n.lacunarity
        amp *= n.gain
    }
    return sum
}

// Godot get_noise_1d(x) == get_noise_2d(x, 0).
fnl_noise_1d :: proc(n: ^FNL_Noise, x: f32) -> f32 { return fnl_noise_2d(n, x, 0) }

@(private="file") FNL_PRIME_X :: i32(501125321)
@(private="file") FNL_PRIME_Y :: i32(1136930381)
// PrimeX << 1 and PrimeY << 1 with C int wraparound
@(private="file") FNL_PRIME_X2 :: i32(1002250642)
@(private="file") FNL_PRIME_Y2 :: i32(-2021106534)

@(private="file")
fnl_lerp :: #force_inline proc(a, b, t: f32) -> f32 { return a + t * (b - a) }

@(private="file")
fnl_floor :: #force_inline proc(f: f32) -> i32 { return f >= 0 ? i32(f) : i32(f) - 1 }

@(private="file")
fnl_grad :: #force_inline proc(seed, xp, yp: i32, xd, yd: f32) -> f32 {
    h := (seed ~ xp ~ yp) * i32(0x27d4eb2d)
    h ~= h >> 15
    h &= 127 << 1
    return xd * FNL_GRADIENTS_2D[h] + yd * FNL_GRADIENTS_2D[h | 1]
}

@(private="file")
fnl_single_os2s :: proc(seed: i32, x, y: f32) -> f32 {
    sqrt3 := f32(1.7320508075688772935274463415059)
    G2 := (3 - sqrt3) / 6
    i := fnl_floor(x)
    j := fnl_floor(y)
    xi := x - f32(i)
    yi := y - f32(j)
    i *= FNL_PRIME_X
    j *= FNL_PRIME_Y
    i1 := i + FNL_PRIME_X
    j1 := j + FNL_PRIME_Y

    t := (xi + yi) * G2
    x0 := xi - t
    y0 := yi - t
    TWO3 :: f32(2.0) / f32(3.0)
    a0 := TWO3 - x0 * x0 - y0 * y0
    value := (a0 * a0) * (a0 * a0) * fnl_grad(seed, i, j, x0, y0)

    c1 := 2 * (1 - 2 * G2) * (1 / G2 - 2)
    c2 := -2 * (1 - 2 * G2) * (1 - 2 * G2)
    a1 := c1 * t + (c2 + a0)
    x1 := x0 - (1 - 2 * G2)
    y1 := y0 - (1 - 2 * G2)
    value += (a1 * a1) * (a1 * a1) * fnl_grad(seed, i1, j1, x1, y1)

    xmyi := xi - yi
    if t > G2 {
        if xi + xmyi > 1 {
            x2 := x0 + (3 * G2 - 2); y2 := y0 + (3 * G2 - 1)
            a2 := TWO3 - x2 * x2 - y2 * y2
            if a2 > 0 { value += (a2 * a2) * (a2 * a2) * fnl_grad(seed, i + FNL_PRIME_X2, j + FNL_PRIME_Y, x2, y2) }
        } else {
            x2 := x0 + G2; y2 := y0 + (G2 - 1)
            a2 := TWO3 - x2 * x2 - y2 * y2
            if a2 > 0 { value += (a2 * a2) * (a2 * a2) * fnl_grad(seed, i, j + FNL_PRIME_Y, x2, y2) }
        }
        if yi - xmyi > 1 {
            x3 := x0 + (3 * G2 - 1); y3 := y0 + (3 * G2 - 2)
            a3 := TWO3 - x3 * x3 - y3 * y3
            if a3 > 0 { value += (a3 * a3) * (a3 * a3) * fnl_grad(seed, i + FNL_PRIME_X, j + FNL_PRIME_Y2, x3, y3) }
        } else {
            x3 := x0 + (G2 - 1); y3 := y0 + G2
            a3 := TWO3 - x3 * x3 - y3 * y3
            if a3 > 0 { value += (a3 * a3) * (a3 * a3) * fnl_grad(seed, i + FNL_PRIME_X, j, x3, y3) }
        }
    } else {
        if xi + xmyi < 0 {
            x2 := x0 + (1 - G2); y2 := y0 - G2
            a2 := TWO3 - x2 * x2 - y2 * y2
            if a2 > 0 { value += (a2 * a2) * (a2 * a2) * fnl_grad(seed, i - FNL_PRIME_X, j, x2, y2) }
        } else {
            x2 := x0 + (G2 - 1); y2 := y0 + G2
            a2 := TWO3 - x2 * x2 - y2 * y2
            if a2 > 0 { value += (a2 * a2) * (a2 * a2) * fnl_grad(seed, i + FNL_PRIME_X, j, x2, y2) }
        }
        if yi < xmyi {
            x2 := x0 - G2; y2 := y0 - (G2 - 1)
            a2 := TWO3 - x2 * x2 - y2 * y2
            if a2 > 0 { value += (a2 * a2) * (a2 * a2) * fnl_grad(seed, i, j - FNL_PRIME_Y, x2, y2) }
        } else {
            x2 := x0 + G2; y2 := y0 + (G2 - 1)
            a2 := TWO3 - x2 * x2 - y2 * y2
            if a2 > 0 { value += (a2 * a2) * (a2 * a2) * fnl_grad(seed, i, j + FNL_PRIME_Y, x2, y2) }
        }
    }
    return value * 18.24196194486065
}

@(private="file", rodata)
FNL_GRADIENTS_2D := [256]f32{
    0.130526192220052, 0.99144486137381, 0.38268343236509, 0.923879532511287, 0.608761429008721, 0.793353340291235, 0.793353340291235, 0.608761429008721,
    0.923879532511287, 0.38268343236509, 0.99144486137381, 0.130526192220051, 0.99144486137381, -0.130526192220051, 0.923879532511287, -0.38268343236509,
    0.793353340291235, -0.60876142900872, 0.608761429008721, -0.793353340291235, 0.38268343236509, -0.923879532511287, 0.130526192220052, -0.99144486137381,
    -0.130526192220052, -0.99144486137381, -0.38268343236509, -0.923879532511287, -0.608761429008721, -0.793353340291235, -0.793353340291235, -0.608761429008721,
    -0.923879532511287, -0.38268343236509, -0.99144486137381, -0.130526192220052, -0.99144486137381, 0.130526192220051, -0.923879532511287, 0.38268343236509,
    -0.793353340291235, 0.608761429008721, -0.608761429008721, 0.793353340291235, -0.38268343236509, 0.923879532511287, -0.130526192220052, 0.99144486137381,
    0.130526192220052, 0.99144486137381, 0.38268343236509, 0.923879532511287, 0.608761429008721, 0.793353340291235, 0.793353340291235, 0.608761429008721,
    0.923879532511287, 0.38268343236509, 0.99144486137381, 0.130526192220051, 0.99144486137381, -0.130526192220051, 0.923879532511287, -0.38268343236509,
    0.793353340291235, -0.60876142900872, 0.608761429008721, -0.793353340291235, 0.38268343236509, -0.923879532511287, 0.130526192220052, -0.99144486137381,
    -0.130526192220052, -0.99144486137381, -0.38268343236509, -0.923879532511287, -0.608761429008721, -0.793353340291235, -0.793353340291235, -0.608761429008721,
    -0.923879532511287, -0.38268343236509, -0.99144486137381, -0.130526192220052, -0.99144486137381, 0.130526192220051, -0.923879532511287, 0.38268343236509,
    -0.793353340291235, 0.608761429008721, -0.608761429008721, 0.793353340291235, -0.38268343236509, 0.923879532511287, -0.130526192220052, 0.99144486137381,
    0.130526192220052, 0.99144486137381, 0.38268343236509, 0.923879532511287, 0.608761429008721, 0.793353340291235, 0.793353340291235, 0.608761429008721,
    0.923879532511287, 0.38268343236509, 0.99144486137381, 0.130526192220051, 0.99144486137381, -0.130526192220051, 0.923879532511287, -0.38268343236509,
    0.793353340291235, -0.60876142900872, 0.608761429008721, -0.793353340291235, 0.38268343236509, -0.923879532511287, 0.130526192220052, -0.99144486137381,
    -0.130526192220052, -0.99144486137381, -0.38268343236509, -0.923879532511287, -0.608761429008721, -0.793353340291235, -0.793353340291235, -0.608761429008721,
    -0.923879532511287, -0.38268343236509, -0.99144486137381, -0.130526192220052, -0.99144486137381, 0.130526192220051, -0.923879532511287, 0.38268343236509,
    -0.793353340291235, 0.608761429008721, -0.608761429008721, 0.793353340291235, -0.38268343236509, 0.923879532511287, -0.130526192220052, 0.99144486137381,
    0.130526192220052, 0.99144486137381, 0.38268343236509, 0.923879532511287, 0.608761429008721, 0.793353340291235, 0.793353340291235, 0.608761429008721,
    0.923879532511287, 0.38268343236509, 0.99144486137381, 0.130526192220051, 0.99144486137381, -0.130526192220051, 0.923879532511287, -0.38268343236509,
    0.793353340291235, -0.60876142900872, 0.608761429008721, -0.793353340291235, 0.38268343236509, -0.923879532511287, 0.130526192220052, -0.99144486137381,
    -0.130526192220052, -0.99144486137381, -0.38268343236509, -0.923879532511287, -0.608761429008721, -0.793353340291235, -0.793353340291235, -0.608761429008721,
    -0.923879532511287, -0.38268343236509, -0.99144486137381, -0.130526192220052, -0.99144486137381, 0.130526192220051, -0.923879532511287, 0.38268343236509,
    -0.793353340291235, 0.608761429008721, -0.608761429008721, 0.793353340291235, -0.38268343236509, 0.923879532511287, -0.130526192220052, 0.99144486137381,
    0.130526192220052, 0.99144486137381, 0.38268343236509, 0.923879532511287, 0.608761429008721, 0.793353340291235, 0.793353340291235, 0.608761429008721,
    0.923879532511287, 0.38268343236509, 0.99144486137381, 0.130526192220051, 0.99144486137381, -0.130526192220051, 0.923879532511287, -0.38268343236509,
    0.793353340291235, -0.60876142900872, 0.608761429008721, -0.793353340291235, 0.38268343236509, -0.923879532511287, 0.130526192220052, -0.99144486137381,
    -0.130526192220052, -0.99144486137381, -0.38268343236509, -0.923879532511287, -0.608761429008721, -0.793353340291235, -0.793353340291235, -0.608761429008721,
    -0.923879532511287, -0.38268343236509, -0.99144486137381, -0.130526192220052, -0.99144486137381, 0.130526192220051, -0.923879532511287, 0.38268343236509,
    -0.793353340291235, 0.608761429008721, -0.608761429008721, 0.793353340291235, -0.38268343236509, 0.923879532511287, -0.130526192220052, 0.99144486137381,
    0.38268343236509, 0.923879532511287, 0.923879532511287, 0.38268343236509, 0.923879532511287, -0.38268343236509, 0.38268343236509, -0.923879532511287,
    -0.38268343236509, -0.923879532511287, -0.923879532511287, -0.38268343236509, -0.923879532511287, 0.38268343236509, -0.38268343236509, 0.923879532511287,
}
