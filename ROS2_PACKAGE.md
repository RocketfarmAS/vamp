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

## Fork deltas to `src/impl/vamp`

This branch used to carry none — packaging only. It now carries one feature delta,
listed here so an upstream sync knows exactly what to re-apply. Everything else
under `src/impl/vamp` remains byte-identical to `main`.

| File | Delta | Upstream-intended |
| --- | --- | --- |
| `collision/cuboid_cuboid.hh` | New. Separating-axis test between two oriented boxes (`obb_separation`, `cuboid_cuboid`, `cuboid_z_aligned_cuboid`). | yes |
| `collision/cuboid_capsule.hh` | New. Separating-axis test between an oriented box and a capsule. | yes |
| `collision/attachments.hh` | `Attachment` gains `cuboids` / `posed_cuboids` next to the spheres, posed in `pose()`. An attachment with no cuboids behaves exactly as before. | yes |
| `collision/validity.hh` | New `cuboid_environment_in_collision`; `attachment_environment_collision` and `attachment_sphere_collision` also test the posed cuboids. Signatures unchanged, so generated robot kernels need no regeneration. | yes |

**Why**: a palletiser's carried box is a box. Filling it with inscribed spheres
under-covers its corners (measured up to 43 mm on a two-box grip, 54 mm on the
gripper body), and the consumer then has to inflate every static obstacle by that
much to stay safe — which rejects placements that are flush by design. An attached
oriented box removes the model error at its source.

**Conventions worth knowing before touching these files**: a returned separation is
negative when the lane is in collision (`sphere_cuboid`'s convention — exact
touching reads as free); edge-edge axes are deliberately left unnormalised because
only the sign of the maximum is read; and an axis that has collapsed to zero length
is dropped from the maximum rather than being propped up with a tolerance, so the
test never grows the geometry it is checking. Box-vs-capsule tests seven axes, which
can report contact near a rounded corner where a gap remains — conservative by
construction, never the reverse.

**Sync note**: expect conflicts in `attachments.hh` and `validity.hh` when merging
`main`; the two new headers are conflict-free. The upstream-facing form of this
delta is additive API, with sphere-only behaviour unchanged, so it should be
offerable to KavrakiLab as-is.

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
   build replaces upstream's on purpose). Re-apply the `src/impl/vamp` deltas
   listed above if upstream has touched `collision/attachments.hh` or
   `collision/validity.hh`.
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
