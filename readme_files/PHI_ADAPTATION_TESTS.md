# Phi adaptation and offline tests

[Home](../README.md) | [Architecture](ARCHITECTURE.md) | [Joint limits](JOINT_LIMITS_IMPLEMENTATION.md)

Reviewed 2026-10-30. This describes current custom-solver tests, replacing historical illustrative transcripts and unsupported blanket accuracy/coverage claims.

## Solver behavior

RobotArm tries the requested end-effector pitch phi first. If no solution exists and adaptation is enabled, it searches nearby pitch within a configurable range; defaults are ±45 degrees in 5-degree steps. Selected solutions must satisfy configured planar joint limits. Adaptation can make an otherwise infeasible orientation/position combination solvable, at the cost of changing pitch.

This is custom geometric IK, not LeRobot's URDF solver. Adaptation provides no collision checking or guarantee that every target can be reached.

## Test coverage and limits

[tests/test_robotarm.py](../tests/test_robotarm.py) covers model state/FK/IK, various targets, exact versus adapted pitch, different search ranges/steps, extreme orientations, zero-range behavior, and trajectory examples. TestRobotArmStress includes test_with_various_targets, test_with_and_without_phi_adaptation, test_phi_adaptation_increases_workspace, and test_trajectory_continuity.

Some tests accept None for unreachable targets and check FK only when a solution is found. Accuracy tolerances differ by scenario; some use per-axis tolerances and others total position error. Passing the suite does not mean every target succeeded or that all solutions meet a universal 0.5 cm / 2-degree bound. The workspace comparison asserts adapted successes are at least exact successes for its sample cases; it does not guarantee a particular percentage improvement.

Old example angles/statistics were illustrative, not captured results for the current calibration. Use the commands below to obtain current output. Mathematical FK consistency does not measure physical positioning error or validate mesh collisions, plan playback, dashboard IPC, or phone input.

## Run from the repository root

```powershell
# All offline tests
python -m pytest -q -o addopts= --tb=short

# Model tests, including adaptation
python -m pytest tests/test_robotarm.py

# Targeted comparison and stress cases
python -m pytest tests/test_robotarm.py::TestRobotArmStress::test_with_and_without_phi_adaptation
python -m pytest tests/test_robotarm.py::TestRobotArmStress::test_phi_adaptation_increases_workspace
python -m pytest tests/test_robotarm.py::TestRobotArmStress
```

pytest.ini defaults to verbose output with captured output disabled. The compact command overrides addopts. Baseline on 2026-10-30: **159 passed** for the complete suite in both default Python 3.12.9 and the existing robot Conda environment's Python 3.10.19. No hardware-control commands were run.
