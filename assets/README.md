# SO-101 URDF mesh assets

These 13 reduced binary STL files provide every visual and collision mesh referenced by [so101_new_calib.urdf](../so101_new_calib.urdf). They retain the original filenames and relative paths, such as `assets/base_so101_v2.stl`. The files and source manifest match `origin/IK-collision` at commit `e836a3ede4df3d8e7d4c022fc47cb25ac46e9032`, copied into this checkout on 2026-11-05. No runtime code or URDF geometry was changed.

## Source and license

- Reduced meshes: [isaka1022/so101-sim2real browser meshes](https://github.com/isaka1022/so101-sim2real/tree/addc4e13f22f6f8ac1e29d15b78380b02387a1e5/web/assets/meshes), pinned at `addc4e13f22f6f8ac1e29d15b78380b02387a1e5`; the branch records download on 2026-11-04.
- Original mesh source: [TheRobotStudio/SO-ARM100, Simulation/SO101/assets](https://github.com/TheRobotStudio/SO-ARM100/tree/5f6d2b876a53a4872e405b991dd925556c9e38a4/Simulation/SO101/assets), pinned at `5f6d2b876a53a4872e405b991dd925556c9e38a4`. The original full-resolution files were acquired here on 2026-10-30 and replaced by this reduced set on 2026-11-05.
- License: Apache-2.0; [LICENSE](LICENSE) is unchanged and byte-identical to the branch copy.
- [SOURCES.json](SOURCES.json) records both sources, the decimator's source-manifest URL, original mesh hashes, current byte sizes, triangle counts, bounds and SHA-256 checksums. All 13 original hashes match the previous full-resolution manifest.

The local URDF now matches the branch and pinned upstream bytes, with SHA-256 `3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c`. Only CRLF-to-LF line endings changed from this checkout's previous URDF; parsed model structure, joint geometry and mesh paths are identical. The copied manifest records the matching checksums. No upstream NOTICE files were found during the original acquisition review.

## Verification and limits

Every URDF mesh reference resolves to a local file. The reduced set contains **39,343 triangles** and totals **1,968,242 bytes**, down from 322,564 triangles and 16,129,292 bytes (87.8% fewer bytes). Offline checks verify exact branch bytes, manifest sizes/checksums/triangle counts/bounds, binary STL lengths and finite normals/vertices. These checks establish basic file integrity, not watertightness, physical accuracy or suitability for a particular collision engine. Reduced surfaces may change clearance estimates; model alignment and collision margins still need validation. STL has no intrinsic unit declaration; validate scale against the URDF and hardware before use.

The standalone [LeRobot Pi IK mode](../README.md#lerobot-ik-raspberry-pi) loads this URDF for FK/IK; direct-axis and custom IK/plan modes do not use it. None of the active paths checks collisions. Mesh acquisition and URDF loading do not enable self-collision avoidance.

The approved [PC-led architecture](../readme_files/PC_PI_ARCHITECTURAL_REFACTOR.md) will reuse the model in a provisional WSL2 motion worker, with separate self-collision and transition validation. Model alignment, collision-pair selection, clearance margins and collision-engine integration must be verified first; table/environment checking is deferred. Development resumes on Windows; see the [implementation plan](../readme_files/PC_PI_IMPLEMENTATION_PLAN.md).
