# TODO

Documentation baseline: 2026-10-30. Existing direct-axis, custom-IK, and plan modes are to remain intact. These tasks describe future implementation, not features supplied by the documentation update.

- [ ] Capture the current Windows/Conda runtime before migration: dependency versions, locally modified LeRobot files (including active per-joint PID tuning), robot ID/hardware calibration, and required device setup. Establish and validate a reproducible Windows venv with a maintained runtime manifest; do not assume reinstalling LeRobot 0.4.0 preserves local changes.
- [ ] Make an alternative entry point/implementation that uses LeRobot's built-in URDF-based FK/IK instead of the custom kinematics implementation. Preserve existing scripts and behavior. Check optional placo installation/platform support, explicit joint ordering, end-effector frame, units, and compatibility. Verify physical accuracy rather than assuming the URDF is more accurate.
- [ ] Obtain and track the URDF's missing assets/*.stl meshes and their provenance/license. Validate the model against physical hardware: frames, zero conventions, motor direction, joint limits, elbow offset, and gripper representation. Resolve the existing normalized-gripper/degree-limit mismatch at the new integration boundary.
- [ ] Implement and validate collision checking separately from IK: arm self-collision, base/table/environment, configurable clearance, and motion between targets. Define behavior for invalid/unreachable/colliding targets and verify offline before controlled hardware trials.
- [ ] Add phone control through the new path with one hardware owner, command arbitration, stale-input/disconnect handling, and explicit stop behavior. Choose phone transport, UI, and protocol during that implementation.
- [ ] Revalidate the previously reported second-pitch-joint offset / end-effector height discrepancy using measured reference poses; record whether it persists before changing calibration or geometry.

See [README](README.md), [architecture](readme_files/ARCHITECTURE.md), and [current environment/tuning](readme_files/CONNECT_SETUP.md).
