# The `ros2` branch — VAMP as a ROS 2 (ament_cmake) package

This branch packages VAMP for the Rocketfarm Sentinel stack
(`sentinel-waypoint-generator` pulls it via `src/ros2.repos`, landing at
`src/RocketfarmAS/vamp`). Consumers `find_package(vamp REQUIRED)` and link
`vamp::vamp`.

## What this branch adds on top of `main`

- `package.xml` + an ament `CMakeLists.txt` **replacing** upstream's root
  CMakeLists (upstream's drives the CPM/nanobind python build and fetches
  dependencies from the network at configure time — a no-go for our offline,
  pinned builds).
- `external/` — the three transitive dependencies upstream fetches via CPM,
  vendored at the exact commits upstream's `cmake/Dependencies.cmake` pins:
  - `nigh` (kavrakilab/nigh @ 9713099, BSD-style) — nearest-neighbour structures
  - `pdqsort` (orlp/pdqsort @ b1ef26a, zlib) — single header
  - `SIMDxorshift` (lemire/SIMDxorshift @ 857c1a0, Apache-2.0) — compiled RNG
- `ROS2_PACKAGE.md` (this file).

Nothing under `src/impl/vamp` (the actual library) is modified on this branch.

## What the exported target provides

- Headers installed under one root so include forms match upstream usage
  verbatim: `<vamp/...>`, `<nigh/...>`, `<pdqsort.h>`, `<simdxorshift128plus.h>`.
- `vamp_simdxorshift` static lib (PIC).
- INTERFACE compile options `-O3 -mavx2 -mfma` — the fixed x86-64 SIMD baseline.
  VAMP's kernels compile in the consuming TU, so consumers inherit the baseline
  deliberately (identical to the previous in-tree vendoring in
  `vamp_sentinel_bridge`).
- Licenses installed under `share/vamp/licenses/`.

## Syncing with upstream (KavrakiLab/vamp)

1. Fast-forward this fork's `main` to the desired upstream commit (never vendor
   straight from KavrakiLab — this fork is the org-controlled source of record).
2. Merge `main` into `ros2`; ours-side for `CMakeLists.txt` (this branch's ament
   build replaces upstream's on purpose).
3. Re-check upstream's `cmake/Dependencies.cmake` for changed transitive-dep
   pins and refresh `external/` accordingly.
4. Downstream, bump the `version:` pin in sentinel's `src/ros2.repos`, rebuild,
   and run the fleet gate
   (`.github/test-fixtures/verify_plans_local.sh` with `BACKEND=vamp_direct`)
   plus `test_vamp_smoke` (SIMD-kernel compile/run pin).

## Fork-derivative planner code lives downstream

The Sentinel-specific forked planners/validators (upright RRT-Connect / FCIT /
simplify, weighted NN growth metric) are **not** on this branch — they are
derivative headers in `sentinel-waypoint-generator`'s `vamp_sentinel_bridge`,
documented in that package's `VENDORED.md`, and kept byte-diffable against the
headers this package installs.
