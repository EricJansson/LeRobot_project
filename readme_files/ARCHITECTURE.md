# Architecture and control reference

[Home](../README.md) | [Run guide](CONNECT_SETUP.md) | [Calibration and limits](JOINT_LIMITS_IMPLEMENTATION.md)

Reviewed 2026-10-30. All three primary modes have been used on hardware by the owner. Behavior below is source-derived; this review did not operate hardware.

## Entry points and ownership

| Entry point (python -m ...) | Implementation and hardware owner |
| --- | --- |
| scripts.teleop_axis_control | Self-contained direct-joint loop; owns SO101Follower, bypassing RobotArmController and custom IK |
| scripts.main_teleop_ik | Wires robot/dashboard/gamepad/state; delegates to scripts/teleop_ik_control.py |
| scripts.main_teleop_plan | Delegates recording/playback to scripts/teleop_plan_control.py, reusing IK teleop primitives |
| scripts.controller_dashboard | Pygame display/panels; IPC client only, never the serial owner |

Both main_teleop entry points accept --port and --index; plan also accepts --plan-file. The reusable teleop_ik_control and teleop_plan_control modules are not standalone launchers.

controller/input/gamepad_mapper.py polls pygame and dispatches bindings. teleop_profile.py supplies an input-state abstraction; controller/scripts/run_teleop.py is an example without robot publishing. Older robot_program/input_control.py and keyboard_control.py are direct-hardware experiments, not the shared IK implementation. read_motors.py is a hardware diagnostic. Running robot_arm_controller as a module executes a smoke test that commands a move; it is not read-only.

## Controller mappings

LB/RB correspond to L1/R1 and LT/RT to L2/R2. Scripts assume axes 0–5 are LX, LY, RX, RY, LT, RT and buttons 0–9 are Cross, Circle, Square, Triangle, LB, RB, Back, Start, L3, R3. These are device/driver assumptions, not automatic mapping.

| Input | Direct axis mode | Cartesian IK / plan manual driving |
| --- | --- | --- |
| LB held | Enables manual driving | Enables manual driving |
| LX | Shoulder pan increment | Y increment |
| LY | Shoulder lift increment, inverted | X increment, inverted |
| RX | Wrist roll increment | Wrist roll increment |
| RY | Wrist flex increment, inverted | Z increment, inverted |
| RT minus LT | Elbow flex increment | Pitch phi increment |
| Cross / Circle | Close / open gripper while held | Start configured joint presets on press |
| Square / Triangle | No joint action | Start configured joint presets on press |
| D-pad | No joint action | Positive hat Y increases gripper target; negative hat Y decreases it |
| R3 held | Fine increments | Fine increments |
| Ctrl+C | Exit and disconnect | Exit, disconnect, terminate dashboard |

IK source comments disagree about D-pad open/close direction. The executable branch increases the gripper for positive hat Y and decreases it for negative hat Y; verify physical direction before relabeling. XYZ describes the custom mathematical frame, not camera-relative movement.

Axis mode runs at 25 Hz, with 1.2-degree joint increments at full input (0.4 fine) and low-pass smoothing alpha 0.35. Gripper increments are 2.0 (0.7 fine), bounded to 0–50 normalized units. IK runs at 25 Hz with 0.25 cm XYZ increments (0.08 fine) and requested pitch bounded to −90 through +90 degrees. Roll/pitch increments are 1.5 degrees (0.4 fine); gripper increments are 2.0 (0.7 fine). These are per-tick increments, not trajectory velocity commands.

### Presets and plan controls

QUICK_POSES in scripts/teleop_ik_control.py defines Cross (home), Circle (stand tall), Square (table grab), and Triangle (eye contact). Labels do not guarantee clearance or physical accuracy. Presets interpolate joint targets synchronously from captured hardware positions, default speed 0.2, with full-speed anchor 20 units per tick. Completion means the final target was commanded, not measured convergence.

In IK mode the mapper gates preset callbacks on LB. Once started, animation runs before the manual LB check, so releasing LB does not cancel it.

Plan mode disables mapper-level LB gating and enforces LB only for continuous manual driving. While idle, face-button presets can therefore start without LB.

| Input | Effect in plan mode |
| --- | --- |
| Hold RB, press Cross | Read actual hardware channels and append MOVE; overrides Cross preset |
| Hold LB, press Start | Play once |
| Hold RB, press Start | Loop until cancelled; LB branch wins if both modifiers are held |
| Back during playback | Cancel playback/interpolation/pause, resync, return to IDLE |
| LB plus active sticks/triggers/D-pad during playback | Cancel playback and return to manual driving |
| Dashboard Act during playback | Cancel playback, then apply pending manual target |

Hold modifiers before pressing action buttons; callbacks run on press edges. Recording and presets are suppressed during playback. Releasing LB alone does not stop playback. Back cancels plan playback; it is not a universal preset stop.

## Plan data and persistence

A plan is an ordered actions list. MOVE stores id, type "move", six-channel joints, and speed. PAUSE stores id, type "pause", and duration_s. Recording samples bus positions. Serialization rounds channel values to one decimal, speed to three, and pause duration to two.

Playback reuses the preset interpolator, with default recorded speed 0.3 and plan speed clamped to 0.01–1.0. PAUSE waits without changing arm targets; duration defaults to 1 second and is bounded below by zero. This is joint sequence playback, not Cartesian or collision-aware path planning.

The plan dashboard supports save/load, clear, action deletion, pause insertion, speed edits, and pause-duration edits. Structural edits cancel playback. Speed edits are accepted without cancellation, though an already-created interpolation retains its calculated steps. Actions are paginated and telemetry identifies the playback index.

Default storage is .runtime/teleop_plan.json; --plan-file selects another path whose parent must exist and be writable. A new session starts with an empty plan; load explicitly in the dashboard. There is no autosave or automatic resume. The default plan is ignored by Git. Cleanup removes IPC, not the saved plan. An edit acknowledgement confirms command processing, not disk success: the save helper returns failure on I/O errors but the handler does not propagate it.

## Custom kinematics and hardware adapter

robot_program/RobotArm.py defines ArmModel, RobotState, calibration loading, FK, and IK orchestration. utils/IK_calculations.py generates yaw/planar candidates and selects using joint limits and continuity; FK_calculations.py supplies planar helpers.

Default geometry: links (11.92, 13.5, 17.0) cm, shoulder height 11 cm, base offset 4 cm. The yaw axis is the origin in the table plane; Z points upward. Positive yaw rotates the radial arm plane from +X toward +Y, with the shoulder offset radially. Pitch phi is the sum of geometric planar angles relative to the horizontal radial direction.

| Hardware name | State | Role |
| --- | --- | --- |
| shoulder_pan | base_yaw_deg | Base yaw |
| shoulder_lift | theta1_deg | First planar joint |
| elbow_flex | theta2_deg | Second planar joint |
| wrist_flex | theta3_deg | Third planar joint |
| wrist_roll | Controller _wrist_roll | Independent of custom IK |
| gripper | Controller _gripper | Independent normalized hardware channel |

Custom XYZ uses **centimeters** and arm angles use **degrees**. For planar motors, geometric = motor × sign + offset, with signs (-1, -1, -1) and offsets (90, -90, 0) degrees. Limits are transformed into the solver frame and solutions back into motor angles. Do not blindly apply these corrections to a URDF model.

RobotArm.move_end_effector updates state only on success. RobotArmController.move_to_xyz solves and sends; failure returns false without sending. Exact pitch is tried first; defaults allow a nearby pitch search within ±45 degrees in 5-degree steps. Adaptation changes achievable orientation, not clearance. Manual IK failure snaps targets back to controller/model state to prevent target drift.

RobotArmController.from_port creates SO101Follower with id lerobot_arm and use_degrees=True. Its context connects, enables torque, and reads positions; exit disables torque and disconnects. Body joints use degrees, but the installed follower configures **gripper as RANGE_0_100 even with use_degrees=True**. Local comments calling every channel degrees are inaccurate. The controller applies exported gripper degree bounds to this normalized channel without conversion; documentation does not correct that mismatch.

sync_from_hardware explicitly replaces state from bus reads. end_effector_pose computes FK from internal state without reading hardware. Ordinary manual IK/telemetry can reflect commanded or estimated state, not continuous measured feedback. Preset ticks and recording explicitly read hardware. send_state clamps outgoing values without rewriting every internal field, so estimated pose can differ from the transmitted command.

## Dashboard communication

The dashboard reads gamepad state and exchanges JSON; the teleop process owns hardware. Manual fields validate against exported limits. Field focus publishes muted to suppress continuous manual gamepad driving. **Act** publishes pending joints; the owner calls set_joint_angles directly, resyncs, and replies done. Act has no interpolation or LB requirement.

| Repository-local path | Purpose |
| --- | --- |
| .runtime/lerobot_dashboard_cmd.json | Dashboard to teleop: active, muted, pending plus joints |
| .runtime/lerobot_dashboard_telemetry.json | Joint state / done acknowledgement; plan adds mode/actions/index |
| .runtime/lerobot_dashboard_plan_cmd.json | Plan edits with cmd_id and plan_cmd_ack replies |
| .runtime/teleop_plan.json | Saved plan, separate from IPC |
| scripts/dashboard_log.txt | IK dashboard stdout/stderr |
| scripts/dashboard_plan_log.txt | Plan dashboard stdout/stderr |

Writes add wall-clock ts, try temporary-file replacement with retries, then fall back to direct writes. Readers tolerate missing/malformed files. This is local best-effort IPC, not a network API or durable command queue. Pending targets are deduplicated by timestamp; plan commands use increasing IDs/acknowledgements. Fresh active/muted status expires after 0.8 seconds. Telemetry is normally pushed at 0.1-second intervals.

Launchers clear IPC on startup and attempt cleanup after normal exit. Dashboard closure/crash or controller disconnection exits the loop; controller context cleanup disconnects/disables torque and launcher finally blocks terminate the dashboard. Abrupt process termination is outside those guarantees.

Only run one robot control session at a time: independent owners compete for the serial port, and IK/plan sessions overwrite shared IPC even if given different ports. Runtime files/logs are ignored by Git.

## Motion protection boundaries

| Path | Checks present | Limits |
| --- | --- | --- |
| Custom IK | Reachability, planar joint limits, continuity preference, optional pitch search | No mesh/table/self/path collision check; yaw clamped later rather than excluded during selection |
| RobotArmController | Required calibration entries and outgoing six-channel clamping | Bounds do not establish clearance; gripper unit mismatch remains |
| Axis mode | LB input gating, smoothing, gripper bound 0–50 | Does not use repository six-channel limit controller or custom IK |
| Presets / plan MOVE | Joint interpolation and controller clamping | Can continue after LB release; interpolation is not collision checking |
| Dashboard Act | UI validation and controller clamping | Direct target, no interpolation or LB gating |

IK_guardrails.py and the preserved filename movement_contrainst.py contain standalone validation/workspace helpers; active teleop does not invoke them. Their existence provides no runtime protection. The browser simulator likewise provides no hardware protection.

## Extension boundary

Add a separate entry point/implementation for upstream IK, preserving existing behavior. Reuse helpers only when assumptions match. The current RobotArmController expects the custom model/state API; it is not already a generic solver adapter.

Installed LeRobot 0.4.0 provides lerobot.model.kinematics.RobotKinematics with URDF path, target frame (default gripper_frame_link), and joint names. FK returns a 4×4 pose transform; IK accepts current joint degrees and a desired transform, returning joint degrees. Internally optional placo uses radians. This differs from custom (x, y, z, phi) control; placo was absent during inspection.

The repository URDF uses meter/radian geometry and includes six revolute joints plus a fixed gripper frame. All 13 referenced assets/*.stl meshes are now included with their [license and provenance](../assets/README.md). The local URDF was structurally identical to the pinned upstream model during acquisition. Its limits differ from the exported hardware limits; an elbow comment mentions a five-degree calibration offset. Verify joint ordering, frames, zero conventions, gripper treatment, and measured FK before treating it as authoritative. More detailed geometry alone does not prove greater accuracy.

The upstream wrapper does not configure collision avoidance. Future checking must cover self-collision, base/table/environment, and motion between targets, with explicit margins and failure behavior. Phone input should feed the new hardware owner rather than open another connection. Transport/UI/protocol choices remain future work; command arbitration and disconnect behavior belong in that design. See [TODO](../TODO.md).
