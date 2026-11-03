# TODO

Documentation baseline: 2026-10-30. Existing direct-axis, custom-IK, and plan modes are to remain intact. Unchecked tasks describe future implementation; completed asset acquisition is recorded below.

- [x] Set up a Python 3.11 `.venv` with dependencies in `requirements.txt`, preserving custom motor tuning and existing calibration/controls. Validate offline; see [setup](readme_files/CONNECT_SETUP.md).
- [x] Add a separate headless Pi mode using LeRobot URDF FK/IK and the working fine increments, preserving existing modes. See [setup and controls](README.md#lerobot-ik-raspberry-pi); physical model validation remains below.
- [ ] Add joint presets to the LeRobot IK mode with synchronized interpolation, joint limits, and FK target resynchronization.
- [ ] Add recording/playback to the LeRobot IK path while preserving joint MOVE/PAUSE actions, persistence, and cancellation controls.
- [x] Obtain the 13 URDF-referenced assets/*.stl meshes and record their pinned upstream revision, checksums, and Apache-2.0 license; see [asset provenance](assets/README.md).
- [ ] Validate the model against physical hardware: frames, zero conventions, motor direction, joint limits, elbow offset, and gripper representation. Resolve the existing normalized-gripper/degree-limit mismatch at the new integration boundary.
- [ ] Implement and validate collision checking separately from IK: arm self-collision, base/table/environment, configurable clearance, and motion between targets. Define behavior for invalid/unreachable/colliding targets and verify offline before controlled hardware trials.
- [ ] Add phone control through the Pi web dashboard with deliberate HTTP command submission, one hardware owner, command arbitration, stale-input/disconnect handling, and explicit stop behavior. Define the UI and command protocol during that implementation.
- [x] Add read-only LeRobot IK telemetry for a PC or phone on the Pi's local network. See [launch instructions](README.md#lerobot-ik-raspberry-pi).
- [ ] Revalidate the previously reported second-pitch-joint offset / end-effector height discrepancy using measured reference poses; record whether it persists before changing calibration or geometry.

See [README](README.md), [architecture](readme_files/ARCHITECTURE.md), and [current environment/tuning](readme_files/CONNECT_SETUP.md).
