package voidengine

import "base:intrinsics"

// ============================================================================
// Seeded RNG streams — deterministic, independent random sequences.
//
// One master seed (a run seed, a daily seed, a randomizer seed code) fans out
// into independent streams: drawing from the loot stream never shifts the
// world stream, so a seed always rebuilds the same map no matter how much the
// player fights. xoshiro256** per stream, seeded through splitmix64.
//
//   Streams :: enum { World, Loot, Combat, Fx }
//   rngs: [Streams]Rng
//   rng_streams_fill(&rngs, rng_seed_from_string("FRAY-7KQ2-ASHE"))
//   x := rng_range_i(&rngs[.Loot], 1, 6)
// ============================================================================

Rng :: struct {
    s: [4]u64,
}

GOLDEN_GAMMA :: u64(0x9E3779B97F4A7C15)

splitmix64 :: proc "contextless" (state: ^u64) -> u64 {
    state^ += GOLDEN_GAMMA
    z := state^
    z = (z ~ (z >> 30)) * 0xBF58476D1CE4E5B9
    z = (z ~ (z >> 27)) * 0x94D049BB133111EB
    return z ~ (z >> 31)
}

rng_make :: proc "contextless" (seed: u64) -> Rng {
    r: Rng
    st := seed
    for i in 0 ..< 4 {
        r.s[i] = splitmix64(&st)
    }
    if r.s[0] | r.s[1] | r.s[2] | r.s[3] == 0 {
        r.s[0] = 1 // xoshiro must not be all-zero
    }
    return r
}

// An independent stream derived from a master seed and a stream id.
rng_stream :: proc "contextless" (master_seed: u64, stream_id: u64) -> Rng {
    st := master_seed ~ (stream_id * 0xD1B54A32D192ED03)
    mixed := splitmix64(&st)
    return rng_make(mixed ~ stream_id)
}

// One stream per value of an enum, e.g. [Streams]Rng.
rng_streams_fill :: proc "contextless" (out: ^[$E]Rng, master_seed: u64) where intrinsics.type_is_enum(E) {
    for &r, e in out {
        r = rng_stream(master_seed, u64(e))
    }
}

@(private)
rotl64 :: #force_inline proc "contextless" (x: u64, k: u64) -> u64 {
    return (x << k) | (x >> (64 - k))
}

// xoshiro256**
rng_next :: proc "contextless" (r: ^Rng) -> u64 {
    result := rotl64(r.s[1] * 5, 7) * 9
    t := r.s[1] << 17
    r.s[2] ~= r.s[0]
    r.s[3] ~= r.s[1]
    r.s[1] ~= r.s[2]
    r.s[0] ~= r.s[3]
    r.s[2] ~= t
    r.s[3] = rotl64(r.s[3], 45)
    return result
}

// Uniform float in [0, 1).
rng_f32 :: proc "contextless" (r: ^Rng) -> f32 {
    return f32(rng_next(r) >> 40) * (1.0 / f32(1 << 24))
}

rng_range_f :: proc "contextless" (r: ^Rng, lo, hi: f32) -> f32 {
    return lo + (hi - lo) * rng_f32(r)
}

// Uniform integer in [lo, hi] inclusive, without modulo bias.
rng_range_i :: proc "contextless" (r: ^Rng, lo, hi: int) -> int {
    if hi <= lo { return lo }
    span := u64(hi - lo) + 1
    limit := max(u64) - (max(u64) % span)
    for {
        x := rng_next(r)
        if x < limit {
            return lo + int(x % span)
        }
    }
}

rng_chance :: proc "contextless" (r: ^Rng, p: f32) -> bool {
    return rng_f32(r) < p
}

// Stable 64-bit seed from text (seed codes like "FRAY-7KQ2-ASHE", or a date
// string for daily challenges). FNV-1a, then one splitmix64 round.
rng_seed_from_string :: proc "contextless" (s: string) -> u64 {
    h := u64(0xCBF29CE484222325)
    for i in 0 ..< len(s) {
        h ~= u64(s[i])
        h *= 0x100000001B3
    }
    return splitmix64(&h)
}
