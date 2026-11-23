#!/usr/bin/env bash
# templar_valley GPU look-test + bench, run from an unpacked package
# (binary + lib/ + examples/templar_valley/assets). Selects the discrete GPU
# through SDL's EGL offscreen driver: on synthesis EGL device 1 is the
# RX 9070 XT (0 = iGPU, 2 = llvmpipe). Override with EGL_DEVICE=<n>.
set -euo pipefail
cd "$(dirname "$0")"
out="${1:-shots}"; mkdir -p "$out"
export LD_LIBRARY_PATH="$PWD/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export SDL_VIDEODRIVER=offscreen
export SDL_HINT_EGL_DEVICE="${EGL_DEVICE:-1}"
export SDL_AUDIODRIVER=dummy
size="${SIZE:-1920x1080}"
{
  nice -n 10 ./templar_valley --size "$size" --ssaa "${SSAA:-2}" --shots "$out"
  for p in 55 25; do
    nice -n 10 ./templar_valley --size "$size" --bench "${FRAMES:-600}" --bench-pitch "$p"
    nice -n 10 ./templar_valley --size 3840x2160 --bench "${FRAMES:-600}" --bench-pitch "$p"
  done
} 2>&1 | tee "$out/run.log"
grep '^BENCH' "$out/run.log" > "$out/bench.txt" || true
