"""Offline research harness regression tests; no physical devices are used."""
from importlib.util import find_spec
import json
from types import SimpleNamespace

import numpy as np
import pytest

from robot_program import phase0_feasibility as f
from scripts import phase0_benchmark as cli


class InteriorCollision:
    def query(self, q):
        collision = abs(q[0]) < 0.1
        return {"colliding": collision, "min_distance_m": -0.01 if collision else 0.01}


def test_path_checks_endpoints_and_interior():
    start, end = np.zeros(6), np.zeros(6)
    start[0], end[0] = -1, 1
    result = f.check_path(InteriorCollision(), start, end, 0.5, 0)
    assert not result["accepted"]
    assert result["rejected_fraction"] == 0.5
    samples = f.path_samples(start, end, 0.5)
    np.testing.assert_array_equal(samples[0], start)
    np.testing.assert_array_equal(samples[-1], end)
    assert np.max(np.abs(np.diff(samples, axis=0))) <= 0.5


@pytest.mark.parametrize("bad", [np.full(6, np.nan), np.full(6, np.inf), np.zeros(5)])
def test_path_rejects_malformed_and_nonfinite(bad):
    with pytest.raises(ValueError):
        f.path_samples(bad, np.zeros(6), 1)


@pytest.mark.parametrize("resolution", [0, -1, float("nan"), float("inf")])
def test_path_rejects_invalid_resolution(resolution):
    with pytest.raises(ValueError):
        f.path_samples(np.zeros(6), np.zeros(6), resolution)


def test_clearance_margin_and_missing_clearance_fail_closed():
    checker = SimpleNamespace(query=lambda q: {"colliding": False, "min_distance_m": 0.001})
    assert f.check_path(checker, np.zeros(6), np.zeros(6), 1, 0)["accepted"]
    assert not f.check_path(checker, np.zeros(6), np.zeros(6), 1, 0.002)["accepted"]
    for margin in (-1, float("nan")):
        with pytest.raises(ValueError):
            f.check_path(checker, np.zeros(6), np.zeros(6), 1, margin)
    checker.query = lambda q: {"colliding": False, "min_distance_m": float("nan")}
    with pytest.raises(ValueError):
        f.check_path(checker, np.zeros(6), np.zeros(6), 1, 0)


def test_missing_model_assets_are_explicit(tmp_path):
    model = tmp_path / "bad.urdf"
    model.write_text('<robot name="bad"><link name="a"><collision><geometry><mesh filename="missing.stl"/></geometry></collision></link></robot>')
    with pytest.raises(ValueError, match="Missing model assets"):
        f.model_inventory(model)


def test_failed_prerequisite_writes_blocked_report(tmp_path):
    output = tmp_path / "blocked.json"
    assert cli.main(["--model", str(tmp_path / "missing.urdf"), "--output", str(output)]) == 2
    report = json.loads(output.read_text())
    assert report["status"] == "BLOCKED"
    assert "FileNotFoundError" in report["error"]
    assert output.with_suffix(".md").is_file()


def test_measure_records_rejections_and_raw_samples():
    report = f.measure(lambda: {"accepted": False, "reason": "unreachable"}, warmup=1, repetitions=3)
    assert report["accepted"] == 0
    assert report["rejected"] == 3
    assert report["reasons"] == {"unreachable": 3}
    assert len(report["raw_ms"]) == 3


def test_hardware_entry_points_are_blocked_and_restored():
    pytest.importorskip("lerobot")
    import serial
    from lerobot.motors.feetech import FeetechMotorsBus
    from robot_program.lerobot_compat import UpstreamSO101Follower
    original = serial.Serial.open
    with f.hardware_forbidden():
        for operation in (serial.Serial.open, FeetechMotorsBus.connect,
                          FeetechMotorsBus.write, UpstreamSO101Follower.connect,
                          UpstreamSO101Follower.send_action):
            with pytest.raises(RuntimeError, match="forbids physical"):
                operation(None)
    assert serial.Serial.open is original


@pytest.mark.skipif(find_spec("placo") is None, reason="Requires isolated Linux PlaCo environment")
def test_real_analytic_and_saved_robot_collision_proofs():
    proof = f.synthetic_collision_proof()
    assert len(f.native_margin_proof()) == 6
    assert len(f.native_mesh_margin_proof()) == 6
    assert proof[0]["colliding"] and not proof[1]["colliding"]
    fixtures = json.loads((f.ROOT / "tests/fixtures/phase0_robot_collisions.json").read_text())
    checker = f.MarginCollisionChecker()
    try:
        assert not checker.query(fixtures["clear"]["joints_deg"])["colliding"]
        assert checker.query(fixtures["colliding"]["joints_deg"])["colliding"]
        path = fixtures["interior_collision"]
        assert not checker.query(path["start"])["colliding"]
        assert not checker.query(path["end"])["colliding"]
        result = f.check_path(checker, path["start"], path["end"], 0.5, 0)
        assert not result["accepted"] and 0 < result["rejected_fraction"] < 1
        assert {"base_link", "moving_jaw_so101_v1_link"}.issubset({n for pair in checker.pairs for n in pair})
    finally:
        checker.close()


@pytest.mark.skipif(find_spec("placo") is None, reason="Requires isolated Linux PlaCo environment")
def test_native_mapping_matches_placo_fk_and_collision_predicates():
    checker = f.MarginCollisionChecker()
    try:
        fixtures = json.loads((f.ROOT / "tests/fixtures/phase0_robot_collisions.json").read_text())
        for name in ("clear", "colliding", "near"):
            q = fixtures[name]["joints_deg"]
            native = checker.query(q)
            for joint, angle in zip((*f.ARM_JOINTS, "gripper"), np.deg2rad(q)):
                checker.robot.set_joint(joint, float(angle))
            checker.robot.update_kinematics()
            assert bool(checker.robot.self_collisions(False)) == native["colliding"]
            checker.pin.updateFramePlacements(checker.model, checker.data)
            native_fk = checker.data.oMf[checker.model.getFrameId("gripper_frame_link")].homogeneous
            np.testing.assert_allclose(native_fk, checker.robot.get_T_world_frame("gripper_frame_link"), atol=1e-10)
        assert not checker.query(fixtures["near"]["joints_deg"])["colliding"]
        assert checker.query_with_margin(fixtures["near"]["joints_deg"], 0.005)["colliding"]
    finally:
        checker.close()


def test_missing_collision_dependency_is_not_a_clear_result(monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules, "placo", None)
    with pytest.raises(ImportError):
        f.CollisionChecker()


def test_unavailable_collision_query_is_rejected(monkeypatch):
    import sys
    fake_robot = SimpleNamespace(load_collision_pairs=lambda path: None)
    monkeypatch.setitem(sys.modules, "placo", SimpleNamespace(RobotWrapper=lambda path: fake_robot))
    with pytest.raises(RuntimeError, match="queries unavailable"):
        f.CollisionChecker()
