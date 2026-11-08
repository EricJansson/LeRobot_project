# Remote IK path verification

## Status and behavior

Implementation is OFFLINE tested; physical acceptance is pending owner-approved tests. Calibration, Feetech registers, torque settings and `remote_ik.hardware.conf` are unchanged. No service was deployed during development. No collision checks have been added.

The controller retains intended XYZ across neutral input, taps, LB release and ordinary rejection. Arming a fresh session adopts measured XYZ; disarm/fault/reconnect invalidates pending work. Reference advancement uses achieved progress along input, not solver acceptance alone. Unexpected Pi shortening is reported and does not advance the reference. The Pi continues to latch a measured stop once; resuming uses bounded recovery toward retained XYZ rather than jumping to it.

Remote-only solver convergence is 0.05 mm, with position-priority orientation weight 0.0001 for translation. Orientation-only requests use the previous 0.5 mm tolerance and 0.01 weight, with the retained-path FK check still applied. Legacy defaults remain 0.5 mm / 0.01. These are numerical command checks, not physical accuracy guarantees. A retained-path FK check prevents solver tolerance accumulating into sideways command drift. Infeasible requests are retried from fresh input each tick without a latched braking state.

## Offline reproduction

From the Windows checkout, run:

```powershell
.venv\Scripts\python.exe -m pytest tests/test_remote_ik.py tests/test_remote_ik_path.py tests/test_lerobot_web_dashboard.py -q
wsl.exe -d Ubuntu --exec sh -c 'cd /mnt/c/Users/rapid/Documents/GitHub/LeRobot_project && ~/.local/share/lerobot-phase0/venv/bin/python -m pytest tests/test_remote_ik_path.py tests/test_remote_ik.py::test_small_sustained_ik_input_survives_encoder_quantization tests/test_lerobot_ik.py -q'
```

Benchmark with the real WSL LeRobot/PlaCo solver; this command does not open serial or SSH. Redirect stdout to a new file outside the checkout. Pin the restored revision so results remain reproducible after committing:

```powershell
wsl.exe -d Ubuntu --exec sh -c 'cd /mnt/c/Users/rapid/Documents/GitHub/LeRobot_project && ~/.local/share/lerobot-phase0/venv/bin/python -m scripts.benchmark_remote_ik --baseline-revision aca7fb4f68f0e05ddf8c86ff4531f06ce4362fa9 --recorded-trace .runtime/remote-ik-test3-xyz.json'
```

Add `--check` to make failed progress/path gates return a nonzero exit status. Repeat with `--arm-rate 20` to expose Pi-rate shortening. The default is the unchanged configured 200 degrees/s. Benchmarks alternate direction every 20 ticks for ten cycles, including XYZ and orientation-only input. Compare rotation travel for orientation cases; XYZ travel there measures unwanted positional motion. These benchmarks use ideal tracking and exclude gamepad ramp, transport and physical loading. Separate regression tests exercise the actual control loop, worker and motor-session logic with quantized, unequal/stalled or delayed feedback. They do not validate motor dynamics.

Existing traces lack source revision metadata and are supporting evidence of unknown code version. Do not label them hardware verification of this implementation. In `remote-ik-test3-xyz.json`, X-only spans include roughly 19–21 mm measured height loss while issued height stays nearly constant; several pause resets adopted heights 14–43 mm below prior intent. Motor diagnostics are snapshots, not proof of a physical cause. Shoulder-lift/elbow lag remains distinct from encoder-invisible deflection.

New traces include revision, dirty status, controller identifier, selected speed, configuration, correction profile, input edges, candidate FK, accepted reference, session/feedback/ack IDs and timing. To enrich all issued/measured FK, use the existing worker `--analyze-trace` command. Then run:

```powershell
.venv\Scripts\python.exe -m scripts.remote_ik_diagnostics C:\path\to\enriched-trace.json
```

The Pi reads before issuing the current command: compare measured XYZ/joints with `issued_before`, not the just-issued target. Summary fields report candidate error, shortening, measured lag per joint, reference changes and round-trip p95. Error maxima across an entire mixed run do not isolate causality; inspect individual motion and stop segments and compare visible height.

## Development results (2026-11-07)

OFFLINE: Windows controller/path/dashboard regressions passed (76 tests, two platform skips). WSL path, encoder and legacy IK regressions passed (48 tests). The final ideal-tracking benchmark passes the 50% progress and 2 mm issued-reference gates in all 12 configured-rate cases and eight 20-degrees/s stress cases. Minimum progress ratios were 0.675 and 0.517 respectively; worst per-run p95 solver time was 2.36 ms. These figures exclude end-to-end transport and motor tracking.

Artifacts are outside the checkout at `C:\Users\rapid\.codex\visualizations\2026\11\07\01a1b7fd-98a6-7063-924e-e31760748ee1`: `benchmark.json`, `benchmark-rate-limited.json` and `baseline-trace-analysis.json`. Existing recordings were preserved. Retained instrumentation consists of the optional launcher trace, offline FK analysis/summary and benchmark command; it is deliberately available for follow-up hardware tests.

HARDWARE: not tested or deployed. Physical cross-axis error, stop latency, cumulative height, load-dependent sag and tap smoothness remain unverified. The final acceptance gates are still pending.

## Optional correction commissioning

Correction requires an explicit `--correction-profile` JSON with six positive numbers: `enter_m`, `exit_m`, `settle_s`, `speed_m_s`, `travel_m`, `timeout_s`. Require `exit_m < enter_m` and `exit_m <= 0.002`. There are deliberately no enabled hardware defaults. Measure stationary encoder/FK noise and settling first; choose hysteresis above noise, wait beyond normal settling, and separately approve speed/travel/timeout before enabling.

Correction targets the fixed retained XYZ, obeys the same joint-rate checks, uses an FK-verified step bound, and stops at the exit band, travel budget or no-progress timeout. A completed/failed episode does not replenish its budget on LB release. Operator input has priority and resets the episode; correction cannot latch operator motion into a fault. It does not command past the retained Cartesian target to compensate persistent load error. If issued XYZ is correct but measured XYZ stays low, that physical tracking issue remains unresolved. Do not add integral compensation or change servo registers implicitly.

## Supervised hardware run sheet — approval required

1. **Prerequisites:** owner approval for deployment and physical motion, Windows XInput controller, existing WSL environment, unchanged Pi calibration/configuration, clear bent-arm starting pose, arm support and accessible physical power disconnect. Stop other serial owners. Record a fixed scale/video reference for actual tool height. Agree stop-latency and post-stop-travel limits before testing. A displayed FK height alone cannot verify deflection.
2. **Matched baseline:** use a separate exported restored checkout, never reset this working tree. From Windows PowerShell in the project, create it with `git archive aca7fb4f68f0e05ddf8c86ff4531f06ce4362fa9 -o "$env:TEMP\remote-ik-baseline.zip"`, then `Expand-Archive "$env:TEMP\remote-ik-baseline.zip" "$env:TEMP\remote-ik-baseline"` (choose fresh paths). Copy the current `remote_ik.hardware.conf` there if absent. In that exported directory, use the original checkout's absolute `.venv\Scripts\python.exe` to run `-m scripts.install_remote_ik_pi`, then `-m scripts.main_teleop_remote_ik --real-arm --speed 1 --trace "$env:TEMP\remote-ik-baseline-run.json"`. All following inputs require clearance; begin at a separately agreed reduced scale before repeating at speed 1.
3. **Baseline inputs:** press neutral Start with LB released; hold LB. Move LY negative (positive X) for 0.4 s, center for 1 s, LY positive for 0.4 s, center for 1 s. Repeat ten times within the clear workspace. Repeat separately for LX and RY, then short combined input, triggers for pitch and RX for roll. Record physical XYZ and joint/FK lag. Try ten 0.1 s taps separated by 0.1 s neutral while continuously holding LB. Repeat pauses with LB release/repress. Record deflection and durations; repeatability depends on matching the starting pose, load and stick deflection.
4. **Candidate:** stop baseline with Back then Esc. From the main Windows checkout run `.venv\Scripts\python.exe -m scripts.install_remote_ik_pi`, then `.venv\Scripts\python.exe -m scripts.main_teleop_remote_ik --real-arm --speed 1 --trace "$env:TEMP\remote-ik-candidate-run.json"`. Repeat step 3 from the same pose/load. Leave correction disabled for this comparison. Near a clear reach boundary, briefly request outward movement, then immediately inward and sideways without dropping LB. Expect infeasible advance to stop without substituting height; fresh feasible input should recover without reset. Confirm gripper-only input leaves arm reference intact.
5. **Stops and reporting:** center input, then Back to disarm, Esc to exit. End the trial on unexpected movement, unacceptable slowdown, hunting, failed stop or loss of clearance; use physical power disconnect if software stopping fails, supporting the arm as needed. Report traces, video, pose/load, settings, cross-axis error, ten-cycle cumulative height change, travel speed, stop latency/settling, longest rejection, and whether LB release was needed. Analyze enriched recordings using step 2's tools. Test stationary correction only after approving a measured profile in a separate run.

Hardware acceptance requires <=2 mm unintended cross-axis error, <=2 mm cumulative height loss over ten cycles, >=50% matched restored travel speed on feasible moves, no ordinary stalls or LB-dependent recovery, and no correction hunting/catch-up jump. Stop rollout if physical tracking misses these targets; propose a separate measured tracking/hardware intervention rather than declaring success from offline results. Reinstall the separate restored checkout to roll back service code; do not alter calibration or torque policy.
