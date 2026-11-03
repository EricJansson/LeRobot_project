# TODO

Documentation baseline: 2026-10-30. Existing direct-axis, custom-IK, and plan modes are to remain intact. Unchecked tasks describe future implementation; completed asset acquisition is recorded below.

- [x] Set up a Python 3.11 `.venv` with dependencies in `requirements.txt`, preserving custom motor tuning and existing calibration/controls. Validate offline; see [setup](readme_files/CONNECT_SETUP.md).
- [x] Add a separate headless Pi mode using LeRobot URDF FK/IK and the working fine increments, preserving existing modes. See [setup and controls](README.md#lerobot-ik-raspberry-pi); physical model validation remains below.
- [ ] Add joint presets to the LeRobot IK mode with synchronized interpolation, joint limits, and FK target resynchronization.
- [ ] Add desktop dashboard support to the LeRobot IK mode: telemetry, manual joint commands, and controller muting with one hardware owner and correct joint/gripper units.
- [ ] Add recording/playback to the LeRobot IK path while preserving joint MOVE/PAUSE actions, persistence, and cancellation controls.
- [x] Obtain the 13 URDF-referenced assets/*.stl meshes and record their pinned upstream revision, checksums, and Apache-2.0 license; see [asset provenance](assets/README.md).
- [ ] Validate the model against physical hardware: frames, zero conventions, motor direction, joint limits, elbow offset, and gripper representation. Resolve the existing normalized-gripper/degree-limit mismatch at the new integration boundary.
- [ ] Implement and validate collision checking separately from IK: arm self-collision, base/table/environment, configurable clearance, and motion between targets. Define behavior for invalid/unreachable/colliding targets and verify offline before controlled hardware trials.
- [ ] Add phone control through the new path with one hardware owner, command arbitration, stale-input/disconnect handling, and explicit stop behavior. Choose phone transport, UI, and protocol during that implementation.
- [ ] Add a Pi-hosted web dashboard accessible from a PC or phone on the same network. Preserve the desktop dashboard and one hardware owner; provide telemetry and deliberate command submission, with command arbitration and stale/disconnected-client handling. Design camera streaming separately. The current Pi migration uses local gamepad control with `--no-dashboard` and adds no network service.
- [ ] Revalidate the previously reported second-pitch-joint offset / end-effector height discrepancy using measured reference poses; record whether it persists before changing calibration or geometry.

See [README](README.md), [architecture](readme_files/ARCHITECTURE.md), and [current environment/tuning](readme_files/CONNECT_SETUP.md).
