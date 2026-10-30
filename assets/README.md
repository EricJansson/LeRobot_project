# SO-101 URDF mesh assets

These 13 unmodified binary STL files provide every mesh referenced by [so101_new_calib.urdf](../so101_new_calib.urdf). They are placed here because the URDF resolves paths such as `assets/base_so101_v2.stl` relative to its location at the repository root. No URDF paths or runtime code were changed.

## Source and license

- Upstream: [TheRobotStudio/SO-ARM100, Simulation/SO101/assets](https://github.com/TheRobotStudio/SO-ARM100/tree/5f6d2b876a53a4872e405b991dd925556c9e38a4/Simulation/SO101/assets).
- Pinned revision: `5f6d2b876a53a4872e405b991dd925556c9e38a4`.
- Downloaded: 2026-10-30.
- License: Apache-2.0; the upstream license is preserved verbatim in [LICENSE](LICENSE).
- [SOURCES.json](SOURCES.json) records source paths, byte sizes, triangle counts, bounds, and SHA-256 checksums.

The local URDF and upstream URDF at that revision are structurally identical after XML parsing; their original byte hashes differ. Both hashes are recorded in the manifest. No upstream NOTICE files were found in that revision.

## Verification and limits

Every URDF mesh reference resolves to a downloaded file. All 13 files passed binary STL length/triangle-count checks and finite normal/vertex checks; total STL size is 16,129,292 bytes. These checks establish basic file integrity, not watertightness, physical accuracy, or suitability for a particular collision engine. STL has no intrinsic unit declaration; validate scale against the URDF and hardware before use.

The current teleop paths still do not load this model or check collisions. Acquisition does not enable self-collision avoidance. Model alignment, collision-pair selection, clearance, motion checking, and collision-engine integration remain future work; see [TODO](../TODO.md).
