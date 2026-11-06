# Phase 0 — PC-led offline feasibility, 2026-11-05

**Phase status: IN PROGRESS. Feasibility: feasible with changes (OFFLINE only).**

WSL2 LeRobot/PlaCo FK/IK works. Exact all-pair PlaCo distance queries are too slow
for the proposed 20 ms cycle. A separate Pinocchio/Coal margin-aware collision
predicate is the candidate for further work; it uses the same reduced meshes and
explicit experimental pair matrix. No distributed controller or motor service has
been implemented. Phase 1 has not started and the gate is not approved.

[Target architecture](PC_PI_ARCHITECTURAL_REFACTOR.md) ·
[Implementation plan](PC_PI_IMPLEMENTATION_PLAN.md) ·
[Legacy architecture](ARCHITECTURE.md) ·
[Hardware lessons](context/IK_TELEOP_LESSONS.md)

## Preserved baseline and source audit

Development remains in the Windows `New-architecture` checkout, starting from
`08b4db2`. All assets, scripts, fixtures and evidence required by this experiment
are present on this branch. No other branch/worktree is a runtime dependency.
The `origin/IK-collision` references in asset provenance are historical sources,
not instructions to switch branches. No branch was deleted.

The commands below identify **legacy hardware entry points; none was launched
against hardware during this work**. Use the established interpreter for each
legacy mode. Default port selection is not confirmation of the attached robot.

| Entry point (`python -m …`) | Ownership and motion behavior found in source |
|---|---|
| `scripts.teleop_axis_control` | Direct follower/serial owner, 25 Hz, LB-gated increments and smoothing; independent of custom IK. |
| `scripts.main_teleop_ik` | `RobotArmController` owns follower; custom cm/degree IK, presets and optional Pygame dashboard. `--port`, `--index`, `--no-dashboard` remain available. |
| `scripts.main_teleop_plan` | Same custom controller, joint recording/interpolation/playback and dashboard IPC; also `--plan-file`. Releasing LB alone does not stop playback. |
| `scripts.main_teleop_lerobot_ik` | Legacy standalone Pi owner, 50 Hz target, measured feedback for IK candidates, full accepted solutions, jaw-only bypass of IK, optional read-only HTTP dashboard. Old `--no-joint-step-limit` remains a compatibility flag. |
| `robot_program.keyboard_control` | Direct follower owner, continuous keyboard increments, torque enabled on entry and disabled on cleanup. |
| `robot_program.input_control` | Direct follower owner, terminal joint commands; parses arguments at import. Do not import as a passive inspection tool. |
| `robot_program.robot_arm_controller` | Its CLI smoke test connects and **commands a move** to `(30, 0, 20)` in custom-model cm units. |
| `robot_program.read_motors` | Connect/configuration is hardware access; its default `--torque off` changes torque. It is not a read-only offline diagnostic. |

`scripts.controller_dashboard` owns no serial connection, but its legacy Act
command reaches `set_joint_angles` through the teleop owner. Presets, plans and
dashboard Act are not collision checked. The legacy HTTP dashboard is read-only.
`controller/scripts/run_teleop.py` is an input example, not a motor publisher.
`scripts/attach-robot.bat` forwards USB to WSL for legacy use; it was not used.
`calibration/generate_with_degrees.py` writes the degree export at import and was
not run. Standalone workspace/guardrail utilities are not active teleop guards.

All active owners use the repository `SO101Follower` and robot ID `lerobot_arm`
with motor IDs 1–6. Default selection is COM3 on Windows and the first matching
Linux ACM/USB path (fallback `/dev/ttyACM0`). The current physical Pi port,
calibration match and live tuning were **not inspected or reconfirmed**; no SSH
connection, serial operation, motor read/write or torque operation was performed.
The Pi environment remains the hardware/legacy reference.

The repository follower still enforces pre-existing calibration and preserves
the configuration sequence: return delay 0, acceleration/maximum acceleration
254 where applicable, position mode and PID `(P,I,D)` of pan `(10,0,48)`, lift
`(12,0,44)`, elbow/wrist flex `(14,0,40)`, roll `(16,0,36)`, jaw `(18,0,32)`.
Jaw torque/current/overload settings remain 500/250/25. None was changed or sent.

Body channels are bus degrees; gripper is normalized `RANGE_0_100`, with the legacy
IK command range 0–50. The custom controller's exported-degree gripper clamping
mismatch is preserved as an unresolved legacy issue, not copied into a new motor
adapter. LeRobot transforms use meters, ordered joints are shoulder pan/lift,
elbow flex, wrist flex/roll, and target frame is `gripper_frame_link`.

The legacy IK loop resynchronizes on LB activation, reads actual joints for
candidates and emits full accepted solutions. Rejection returns its desired pose
to the measured pose without issuing a replacement hold command. It has no
persistent Pi watchdog/reconnect service; cleanup disables torque. This is not
the proposed new hold policy. The historical 5° rejection/partial-step limiter
was not restored.

## Environment and reproducibility

| Component | Verified configuration |
|---|---|
| Windows legacy | Python 3.11.14, LeRobot 0.4.0, NumPy 2.2.6, Torch 2.7.1, torchvision 0.22.1, Pygame 2.6.1, SDL 2.28.4, pytest 8.4.2 |
| WSL host | Ubuntu 22.04.3 LTS x86-64, kernel 6.18.54.1-microsoft-standard-WSL2, glibc 2.35, Intel Core i5-14400F, 16 exposed logical CPUs |
| Isolated solver | CPython 3.12.12 (uv-managed), LeRobot 0.6.1, PlaCo 0.9.15, Pinocchio 3.4.0, Coal 3.0.1, NumPy 2.2.6 |
| Isolated CPU Torch | torch 2.7.1+cpu, torchvision 0.22.1+cpu; `torch.version.cuda is None` |
| Native ABI | cmeel-urdfdom 4.0.1, cmeel-tinyxml2 10.0.0, cmeel-boost 1.87.0.1, eigenpy 3.10.3 |
| Bootstrap | uv 0.9.8; isolated root `~/.local/share/lerobot-phase0` |

The Windows `.venv` is unchanged. Default Windows `python` resolves to Conda
3.12.9, so commands explicitly select `.venv/Scripts/python.exe`. The Windows
`py` launcher is inaccessible in this session. WSL queries require execution
outside the desktop sandbox; initial sandbox `E_ACCESSDENIED` was resolved by
approved scoped WSL execution, not by changing WSL configuration.

WSL system Python remains 3.10.12. The existing `~/lerobot` checkout is LeRobot
0.4.1 at `d57d1aa1`, with a modified follower file; it was not changed or used as
the solver environment. Initial uv provisioning also created its normal user
Python 3.12 launcher link; the reproducible setup now uses `--no-bin` and does
not alter system Python or shell configuration.

From WSL in this Windows checkout:

```bash
bash scripts/setup_phase0_wsl.sh
PHASE0_PY="$HOME/.local/share/lerobot-phase0/venv/bin/python"
"$PHASE0_PY" -m scripts.phase0_benchmark --mode inspect --output .runtime/phase0_inspect.json
"$PHASE0_PY" -m scripts.phase0_benchmark --mode all --output .runtime/phase0_native.json
"$PHASE0_PY" -m scripts.phase0_benchmark --mode collision --backend placo-distances --case neutral --output .runtime/phase0_distances.json
"$PHASE0_PY" -m pytest -o addopts= -q -ra
```

The setup consumes `requirements-phase0-wsl.lock.txt` when present. It installs
only into the isolated Linux root, uses the CPU wheel index and runs dependency
and import checks. The direct requirements document intent; the lock records all
88 resolved packages. Do not install either file into Windows or the Pi.
[LeRobot 0.6.1 dependency metadata](https://github.com/huggingface/lerobot/blob/v0.6.1/pyproject.toml)
requires Python ≥3.12 and constrains the PlaCo/cmeel ABI. A matching
[PlaCo 0.9.15 Linux x86-64 Python 3.12 wheel](https://pypi.org/project/placo/0.9.15/)
was available and its installed imports succeeded.

From Windows PowerShell:

```powershell
.\.venv\Scripts\python.exe -X utf8 -m pytest -o addopts= -q -ra
.\.venv\Scripts\python.exe -X utf8 validate_calibration.py
.\.venv\Scripts\python.exe -m scripts.phase0_benchmark --mode ipc --output .runtime/phase0_ipc.json
.\.venv\Scripts\python.exe -m scripts.phase0_benchmark --mode gamepad --output .runtime/phase0_gamepad.json
```

The IPC default resolves the selected WSL user's isolated interpreter; override
`--wsl-distro` or `--wsl-python` as needed. It is an echo experiment, not a command
protocol or a Pi connection. Missing prerequisites produce a JSON/Markdown
`BLOCKED` report and nonzero exit. `--repetitions` and `--warmup` are configurable;
recorded full runs use 500/20. Inspection requires only the legacy NumPy stack.

## Model, collision and regression evidence

All 13 reduced binary STL assets resolve locally. Hashes match the manifest:
39,343 triangles, 1,968,242 bytes, 17 collision instances, eight links and seven
joints (six revolute plus fixed tool frame). File integrity is not physical
geometry validation. The following hashes remain unchanged:

| File | SHA-256 |
|---|---|
| `so101_new_calib.urdf` | `3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c` |
| `calibration/lerobot_arm.json` | `542352cd58b5ee6971d06e367caa45a200d80c5dfe9b602cb9d84190f74a67eb` |
| `calibration/lerobot_arm_with_degrees.json` | `4993c867e32b433aace9c81f3b442b22155d92b44c8ea8a811e07c632b2e35ba` |

Neutral model-only tool XYZ is approximately `(0.391361, -0.00000921, 0.226470)` m.
No corresponding physical measurement was available to establish alignment.
The historical elbow-limit 5° comment and tool-height question remain unresolved.
Calibration/export arithmetic passes with UTF-8 output; this does not establish
geometric alignment or resolve the normalized gripper mapping.

The benchmark reuses `LeRobotIKController.solve_pose` without modifying it. It
constructs an unconnected follower solely for installed LeRobot normalization,
then forbids serial opening, real bus operations, connection and motor writes.
The eight-step retraction replay substitutes explicit fixed feedback and an
in-memory command sink while physical entry points remain blocked. It records
eight full commands and a 13.10075° maximum command lead. This reproduces command
delivery logic only, not loaded physical retraction or holding behavior.

The collision model is separate from the IK model. Its temporary URDF uses
absolute mesh paths and temporary pair configuration; no root `collisions.json`
was added where legacy PlaCo loading could discover it. There are 15 included
link pairs, expanding to 85 geometry pairs. Base and moving jaw participate.
Same-link geometry is one rigid body. Six directly connected link pairs are
excluded experimentally: base–shoulder, shoulder–upper arm, upper–lower arm,
lower arm–wrist, wrist–gripper and gripper–moving jaw. Each exclusion and its
reason is saved in the results. Excluding an entire adjacent pair can hide
non-mating interference; this matrix still requires geometry/physical review.

Proofs and limits:

- Analytic boxes detect overlap and return the expected 200 mm clear gap with
  PlaCo's explicit collision/distance API. IK acceptance alone is never used as
  collision evidence.
- Coal security margins are tested at 0, 2 and 5 mm on both solid boxes and
  triangle-mesh BVHs, including gaps smaller/larger than the requested margin.
- Saved seeded robot fixtures demonstrate a clear pose, a colliding pose and a
  clear pose rejected with a 5 mm margin. The corpus search considered 1,146
  poses within intersected arm limits and the jaw's URDF range.
- A saved robot transition has clear endpoints and an interior collision. All
  sampled resolutions/margins reject it. Full subdivision checks include both
  endpoints; early exit occurs only once unsafe geometry is found.
- A separate Pinocchio model's FK and zero-margin collision classification agree
  with PlaCo on saved fixtures. They share native libraries: this tests mapping
  consistency, not independent physical-model correctness.
- Native predicates test the requested margin but do **not** compute a minimum
  clearance. Their `min_distance_m` is null, never an invented distance.
- Jaw fixtures use explicit URDF angles over its modeled range, not normalized
  motor-channel values. The conversion and physical jaw travel remain unverified.
- Subdivision is discrete, not continuous/swept collision proof. Reduced mesh
  accuracy, containment/contact behavior and unsampled clearance still require
  review. No table/environment collision is included.

## Measurements and tests

Raw samples, distributions, accept/reject counts/reasons, source/model hashes and
environment metadata are in [phase0_results](phase0_results/). Generated Markdown
tables accompany the JSON. Each timing group uses 20 warmups and 500 repetitions;
initial smoke runs are not the reported performance evidence.

| OFFLINE operation | Median ms | p95 ms | p99 ms | Worst ms | >20 ms | Accepted/500 |
|---|---:|---:|---:|---:|---:|---:|
| Neutral IK | 0.050 | 0.093 | 0.227 | 0.905 | 0 | 500 |
| Native zero-margin collision predicate | 0.046 | 0.106 | 0.184 | 0.402 | 0 | 500 |
| Neutral IK + path, 1 degree / 2 mm | 0.162 | 0.233 | 0.378 | 0.524 | 0 | 500 |
| 16 mm retraction IK + path, 0.5 degree / 2 mm | 1.133 | 1.601 | 1.918 | 2.394 | 0 | 500 |
| Interior-colliding path, 0.5 degree / 2 mm | 9.029 | 9.924 | 10.469 | 12.020 | 0 | 0 |
| PlaCo exact-distance comparison, neutral | 77.078 | 78.596 | 79.669 | 85.035 | 500 | 500 |
| Windows-WSL echo RTT at 50 Hz | 0.421 | 0.608 | 0.740 | 2.861 | 0 | 500 |

The native sweep contains 97 groups and 48,500 measurements. No native operation exceeded 20 ms; the maximum was 14.012 ms. All 500 unreachable requests per IK/combined group reject. The 5 mm near-collision fixture and interior-colliding paths reject in all corresponding groups.

The 239-byte IPC echo sustained 49.999 Hz with zero reply deadline misses or overrun reschedules. Host scheduling wake lateness p99 was 0.812 ms; the worst send interval was 20.773 ms. Intervals slightly above 20 ms are scheduling jitter, not automatically a missed whole period. Worker-local JSON parsing median/p99 was 0.023/0.045 ms. WSL home lookup/startup took 2632.2/161.8 ms, recorded separately from warmed RTT. None of these numbers includes IK, collision, the Pi or mechanical response.

The exact-distance comparison is limited to neutral: every one of its 500
measurements exceeded 20 ms. A full exact-distance path sweep is deliberately
skipped after this result; the native margin candidate receives the complete
pose/path sweep instead. Initial exact-distance fixture discovery was stopped
and later exhausted its small search without a transition; the broader native
search produced the saved path. These exploratory attempts are not timing runs.

The solver suite covers neutral, extended/retraction, an outward near-extension
proxy, near pan limit, wrist roll, unreachable and collision-near poses. The
near-extension proxy is not a Jacobian-conditioned singularity classification.
Combined rows include IK, candidate validation and sampled transition checks;
the independent collision rows query the supplied starting configuration.
Unreachable IK requests reject without issuing targets. The margin/resolution
sweep uses 0/2/5 mm and 0.5/1/2° as experimental settings, not approved policy.
No kernel real-time configuration or CPU affinity was applied. Timing is local
wall duration and includes desktop/VM scheduling variability, not servo response.

| Offline test run | Result |
|---|---|
| Initial Windows legacy baseline | 206 passed, 29 skipped, one dashboard failure; 55.56 s |
| Final Windows suite | 221 passed, 31 skipped, same one dashboard failure; 54.67 s |
| Final isolated WSL suite | **253 passed**, no skips; 24.05 s |
| Dependency checks | Windows `pip check` and isolated uv dependency check passed; native IK/collision and CPU Torch imports passed |
| Calibration utility | Passes with `-X utf8`; original Windows default-encoding run raises `UnicodeEncodeError` while printing a checkmark |

All tests are **OFFLINE**. Windows skips are the 29 LeRobot 0.6.1-only cases plus
two new real-PlaCo/Pinocchio checks. The Linux environment exercises those tests.
The Windows dashboard failure is reproducible: its second-bind assertion fails;
legacy dashboard code was deliberately unchanged. A first Linux run occurred
before fixture generation completed and failed the new missing-fixture test;
the complete final run above passes with the saved fixtures.

The initial Windows session detected no gamepad. On November 6 the connected
controller appeared to SDL as `Controller (XBOX 360 For Windows)`. A 500-sample
polling benchmark measured 0.0057 ms median, 0.0236 ms p99 and 0.0456 ms worst.
The last raw sample showed centered sticks and D-pad, released buttons and
triggers at their idle value. This measures the input read cost. [Polling data](phase0_results/windows_gamepad_connected.json)

**HARDWARE VERIFIED (gamepad input only, owner observation).** The owner ran
the standalone `python.exe -B -m scripts.controller_dashboard`
in the activated VS Code environment and saw the controls match their bindings.
The dashboard confirms visible input mapping. The hold-to-move command gate
is checked separately with a fake arm, and physical stopping remains unverified.

No Pi serial timing was collected. The historical ~2.78 ms six-motor
read figure remains reported background, not a reproduced result or a write/read
cycle measurement. There is no end-to-end 50 Hz or motion-latency claim.

## Follow-up on collision exclusions and sampling

**OFFLINE VERIFIED.** The audit swept each of the six excluded hinges over its
modeled range at steps of at most 1°, with zero included explicitly. It used the
intersected LeRobot limits for the arm and URDF angles for the jaw. Of 36 omitted
mesh pairs, six intersected at every sampled angle and four more intersected
away from neutral. The latter intersections would go unreported with blanket
adjacent-link exclusions.

| Joint | Additional intersecting meshes | Observed samples |
|---|---|---|
| Elbow flex | Upper arm shell against the lower arm shell, wrist motor holder and lower arm servo | Three distinct pairs, first hit at +90.84°, +92.84° or +96.83° depending on the pair |
| Wrist roll | Gripper wrist-roll follower against the wrist roll/pitch piece | Five samples between −146.25° and −115.34° |

The mesh-level experiment re-enabled 30 pairs and kept the six persistent
overlaps excluded. The original 85-pair link mask accepted all eight fixed-feedback
retraction paths at every tested margin. The narrower mask checked 115 pairs
and accepted all eight paths at zero margin. At 2 mm and 5 mm it rejected all
eight at the same extended starting pose. At 2 mm, the gripper follower was
within the threshold of both the moving jaw and the wrist roll/pitch piece.
The jaw was held at zero URDF degrees throughout the comparison. This points
to a need for pair-specific contact and margin review before choosing a mask
that preserves retraction. Physical interpretation of the mesh contacts remains
open.

The existing 85-pair checker classified 1,280 seeded paths with at most 4° of
change per joint. The corpus includes 512 uniformly seeded paths and 768 paths
near sampled collision boundaries of the saved transition. The 2°, 1°, 0.5° and
0.125° grids agreed on every path at each margin. The finer grid provides a
sampled comparison and can miss contact between samples too.

| Margin | Rejected paths out of 1,280 | 0.5° p99 / worst | 0.125° p99 / worst |
|---|---:|---:|---:|
| 0 mm | 496 | 1.859 / 3.390 ms | 5.976 / 9.841 ms |
| 2 mm | 584 | 2.600 / 4.908 ms | 8.059 / 14.846 ms |
| 5 mm | 704 | 3.070 / 15.070 ms | 9.747 / 51.437 ms |

One 0.125° / 5 mm path check exceeded 20 ms on this run. These are collision
path times, with IK and IPC outside the measurement. Any eventual validator
needs a latency budget that leaves room for those other stages. The 17 existing
feasibility regressions passed in the isolated WSL stack.

Rerun from WSL in the checkout using the existing isolated environment.

```bash
"$HOME/.local/share/lerobot-phase0/venv/bin/python" -m scripts.phase0_collision_audit \
  --output .runtime/phase0_collision_audit.json
```

The [audit data](phase0_results/wsl_collision_audit.json) records individual hit
angles, raw timings, seeded corpus settings, source hashes and replay outcomes.

## Pair-specific margin study

**OFFLINE VERIFIED.** The study uses the same 115 enabled mesh pairs and six persistent-overlap
exclusions. The focused profile applies 2 mm to the original 85 pairs and the
three re-enabled elbow pairs, and 0.25 mm to the other 27 re-enabled pairs. All
eight replay paths start from the same fake extended-arm feedback and use the
full IK commands. The jaw stays at zero URDF degrees for this replay.

| Experimental profile | Retractions accepted on each grid | Saved clear pose and transition endpoints |
|---|---:|---|
| 85 pairs, uniform 2 mm | 8/8 | Clear |
| 115 pairs, uniform 0 mm | 8/8 | Clear |
| 115 pairs, uniform 2 or 5 mm | 0/8 at either margin | Rejected |
| 115 pairs, 2 mm with only the two gripper overrides at 0.25 mm | 8/8 | Rejected by `base_link_1` / `shoulder_link_2` |
| 115 pairs, focused profile | 8/8 | Clear |

With the focused profile, the saved transition first rejects about 39.6% along
the 0.5° path when `base_link_1` approaches `gripper_link_1`. The 0.125° grid
first rejects at 39.3%. The saved colliding pose still rejects, and the saved
near pose triggers the wrist pair. In the two independent override probes,
0 and 0.25 mm accepted every retraction. Raising either gripper pair to 0.5 or
1 mm rejected all eight at the shared starting pose, with that pair triggering.

| Interference with the focused profile | Zero-margin hits retained | Total hit samples | First additional sample by increasing angle |
|---|---:|---:|---:|
| Elbow, upper arm shell / lower arm shell | 5/5 | 8 | −96.83° on the opposite end of travel |
| Elbow, upper arm shell / wrist motor holder | 7/7 | 8 | +89.84° |
| Elbow, upper arm shell / lower arm servo | 1/1 | 3 | +94.83° |
| Wrist roll, gripper follower / roll-pitch piece | 5/5 | 24 | −148.24° |

The elbow shell pair also gained a hit at +91.84° before its first zero-margin
hit at +92.84°. These are sampled threshold hits in the model, with each hinge
swept in isolation. The two path grids agreed on all 1,295 short paths, including
1,280 seeded paths and 15 paths around elbow and wrist boundaries. Each grid
accepted 647 and rejected 648. Both disagreement lists in the raw data are empty.
Physical mesh alignment and clearance between samples remain unverified.

### 20 ms budget

Measurements use the focused profile on the i5-14400F WSL2 stack above. The
repeated cases use 20 warmups and 500 samples per case at 0.5°. The retraction
rows pool samples across the eight fixed-feedback requests. Combined timing
starts with the Cartesian request and includes IK, candidate validation and the
sampled path check. The short corpus measures one path check per path per grid.

| Offline workload | Median ms | p95 ms | p99 ms | Worst ms | >20 ms | Accepted |
|---|---:|---:|---:|---:|---:|---:|
| Short corpus path checks, 0.5° | 11.637 | 21.628 | 23.944 | 27.272 | 154 | 647/1,295 |
| Short corpus path checks, 0.125° | 39.602 | 75.902 | 83.278 | 96.830 | 735 | 647/1,295 |
| Eight retraction requests, path checks | 27.666 | 42.576 | 46.287 | 67.221 | 2,540 | 4,000/4,000 |
| Eight retraction requests, IK + path | 27.979 | 43.371 | 48.979 | 66.681 | 2,529 | 4,000/4,000 |
| 16 mm retraction, IK + path | 40.875 | 46.198 | 51.518 | 76.452 | 500 | 500/500 |
| Interior-collision transition, path check | 160.015 | 167.271 | 174.711 | 199.222 | 500 | 0/500 |

Replay requests 1–2 met the budget in all combined runs. Request 3 exceeded
it in 29/500 runs, and requests 4–8 exceeded it every time. Both stages of the
16 mm case and the long transition path check exceeded the budget every time.
The other representative IK cases stayed below 20 ms in this run, with a
combined worst of 18.332 ms for wrist roll. The unreachable case rejected at
IK, so it has no path-only timing.

A follow-up timing probe used the same two paths at 0.5°, with five warmups and
20 samples per group. Re-enabling the adjacent pairs already put both paths
over budget at zero margin. Setting the focused margins once before sampling
saved a few milliseconds, while both paths still exceeded 20 ms in all runs.

| Timing probe configuration | Final replay path median | Interior-collision path median |
|---|---:|---:|
| 85 pairs, uniform 2 mm | 1.068 ms | 9.289 ms |
| 115 pairs, uniform 0 mm | 30.904 ms | 124.570 ms |
| 115 pairs, focused profile | 41.616 ms | 160.452 ms |
| 115 pairs, focused margins set once | 39.986 ms | 154.748 ms |

The separately measured Windows–WSL echo from the earlier benchmark had
0.740 ms p99 and a 2.861 ms maximum. The focused profile already exceeds
20 ms before IPC on the workloads above. It remains an offline experiment,
and the Phase 1 gate remains closed. [Echo results](phase0_results/windows_wsl_ipc_50hz.md)

All 18 Phase 0 regression tests passed in WSL2, including the new test for
independent overrides, invalid margins and excluded pairs.

Rerun in the isolated WSL2 environment from the checkout.

```bash
PHASE0_PY="$HOME/.local/share/lerobot-phase0/venv/bin/python"
"$PHASE0_PY" -B -m scripts.phase0_collision_audit --study pair-margins \
  --output .runtime/phase0_pair_margins.json
"$PHASE0_PY" -B -m pytest -o addopts= -q -ra tests/test_phase0_feasibility.py
```

The [raw study data](phase0_results/wsl_pair_margins.json) records the profile
thresholds, triggering mesh instances, hinge angles, sampled disagreement
witnesses, individual timing samples, environment and source hashes.

## Faster collision checking and late requests

**OFFLINE VERIFIED.** A follow-up kept the same focused 115 pairs, clearances
and sampled paths. A per-pair probe found that the two close-fitting gripper
pairs accounted for about 26 ms of the final retraction's 45 ms of native pair
queries. The new checker shortens the search for extra distance information on
adjacent pairs. It also reuses a clear result if a stricter query established
extra clearance and the calculated maximum mesh displacement fits within that
clearance. It queries the configured margin directly if the stricter query
cannot prove clearance. The cache starts empty for each timed path.

The new checker matched the full list of triggering pairs at 2,019 sampled
poses. It matched the reference's path outcome and first rejection sample for
all 1,295 paths at each of 0.5° and 0.125°. This includes the known elbow and
wrist interference and the saved collision between clear endpoints. All eight
full retraction paths still passed.

| Workload | Reference median / p99 | Faster checker median / p99 | Faster checker over 20 ms |
|---|---:|---:|---:|
| Short paths, 0.5° | 11.679 / 24.387 ms | 5.821 / 12.873 ms | 0/1,295 |
| Short paths, 0.125° | 39.165 / 82.779 ms | 18.517 / 41.475 ms | 629/1,295 |
| Eight retractions, IK + 0.5° path | 27.979 / 48.979 ms | 3.674 / 5.673 ms | 0/4,000 |
| Long interior-collision path, 0.5° | 160.015 / 174.711 ms | 82.199 / 90.881 ms | 500/500 |

The two short-path rows timed both checkers on the same run. The other rows use
the earlier reference timings above. The slowest combined retraction took
7.323 ms, and the slowest short 0.5° path took 18.071 ms. These are WSL2 worker
times. Windows input, IPC, Wi-Fi, Pi serial and physical response have separate
timing costs. The finer grid remains a comparison and the long transition
remains a stress case.

A 20 ms worker deadline was exercised with simulated feedback that followed
immediately, lagged or stayed fixed. Each scenario issued all eight valid fake
targets. Injected late, stale and superseded requests issued none. A separate
fresh request could pass after those faults. The experiment checks time before
sampling another pose and again before returning a clear target. It cannot
interrupt a native call already in progress. The Pi loop should run
independently and discard late targets. The worker's share of the 20 ms period
and the Pi's stop/hold policy still need end-to-end measurements and hardware
review. These experiments establish sampled model behavior only.

The focused WSL2 regression run passed all 59 feasibility, gamepad mapping
and LeRobot IK tests.

Rerun the [timing study](phase0_results/wsl_hinge_collision_timing.json) in WSL2.

```bash
"$HOME/.local/share/lerobot-phase0/venv/bin/python" -B -m scripts.phase0_collision_timing \
  --output .runtime/phase0_collision_timing.json
```

## Decisions, remaining gates and rollback

Retain WSL2/PlaCo provisionally for IK. Reject exact all-pair distances as the
per-tick approach on this setup; carry the independently queried native
Pinocchio/Coal margin predicate forward as an experimental candidate. Preserve
the solver/checker seams and replay corpus. The measurements support further
offline work, not a motion release or Phase 1 implementation in this pass.

Unresolved decisions and next steps:

1. Review the collision exclusions, mesh alignment/scale, physical FK and elbow
   offset before selecting a margin or path subdivision. The [seven historical
   poses](helper_files/robot_orientations.md) contain images and legacy readings.
   Four have body angles outside current URDF limits, including +100.20° elbow
   flex versus the modeled +96.83° limit. They contain no measured tool XYZ or
   paired jaw geometry and normalized readings. Current pose measurements are
   needed to resolve the discrepancies, subject to owner approval.
2. Establish normalized jaw ↔ URDF geometry conversion; do not reuse the custom
   degree-bound mismatch or infer that normalized values are angles.
3. Establish a conservative path/timeout policy for longer movements and native
   query outliers. Sampled clear paths are not continuous guarantees.
4. Choose Windows/WSL and PC/Pi transports after representative payload/load,
   fault and jitter tests. The echo protocol is not a selected control protocol.
5. Leave watchdog timeout, bounded loaded stop/hold, torque policy, Pi rate limits,
   remote supervision and actual end-to-end criteria unsettled. No 5° limiter,
   stale-command resume or repeated sagging-pose capture policy was introduced.
6. Measure Pi bus read/write and physical response only in separately authorized
   hardware work. Establish its current owner, port and calibration match then.

Start the manual follow-up with the arm stationary. With owner approval, identify
the Pi's current motor owner, adapter and calibration, then pair the reported
joint positions and jaw reading with a measured tool position and jaw opening.
Record the physical reference point and measurement uncertainty. Time position
reads separately. Benchmark writes and physical stopping in later approved
steps with the owner at the arm.

Rollback requires no legacy command change: continue using the existing modes
and interpreters. The new benchmark is opt-in, has no service or automatic startup,
and uses an isolated WSL environment. Reports and fixtures are development
artifacts. No Git staging, commits, robot hardware operations or branch deletion occurred.
