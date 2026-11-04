# SO-101 URDF mesh assets

These 13 reduced binary STL files provide the visual and collision meshes referenced by [so101_new_calib.urdf](../so101_new_calib.urdf). They retain the original filenames so the URDF can use the existing relative paths.

## Source and license

- Reduced STLs come from the pinned [so101-sim2real browser meshes](https://github.com/isaka1022/so101-sim2real/tree/addc4e13f22f6f8ac1e29d15b78380b02387a1e5/web/assets/meshes), downloaded on 2026-11-04.
- The reduced meshes were generated from the original [TheRobotStudio SO-101 meshes](https://github.com/TheRobotStudio/SO-ARM100/tree/5f6d2b876a53a4872e405b991dd925556c9e38a4/Simulation/SO101/assets). All 13 original hashes match the decimator's source manifest.
- Both sources use Apache-2.0 and publish the same [license text](LICENSE). [SOURCES.json](SOURCES.json) records the revisions, paths, byte sizes, triangle counts, bounds, and checksums.

The URDF matches the pinned TheRobotStudio model byte for byte in this checkout. The manifest records both checksums.

## Verification and limits

The 13 reduced files contain 39,343 triangles and total 1,968,242 bytes. All passed binary STL length, triangle-count, and finite normal and vertex checks. Physical model alignment and clearance still need validation.

The LeRobot IK launcher loads this model for kinematics. Live collision avoidance remains pending. See [TODO](../TODO.md).
