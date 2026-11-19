"""Offline absolute-target, workspace and interruption tests. No motor hardware."""

from http.client import HTTPConnection
from contextlib import ExitStack
import json
import os
from pathlib import Path
import shutil
import subprocess
from threading import Event
from threading import Thread
from time import monotonic, sleep
from types import SimpleNamespace

import numpy as np
import pytest

from robot_program.aruco_control import ArucoControl, alignment_readout
from robot_program.remote_ik_control import ControlLoop
from robot_program.robot_workspace import RobotWorkspace, WorkspaceStore
from scripts.mobile_workspace_dashboard import MobileWorkspaceDashboard
from scripts.lerobot_web_dashboard import render_dashboard_page
from scripts.remote_ik_worker import solve_intent, workspace_path_clear
from scripts.mobile_teleop_gateway import MobileGateway, MobileInput, POSE_FRAME, POSE_PATH
from robot_program.aruco_control import CONTROL_HEADER, CONTROL_VERSION


BOUNDS = dict(version=1, x_min_cm=10, x_max_cm=30, y_min_cm=-10, y_max_cm=10,
              height_min_cm=5, height_max_cm=15, table_z_cm=0)


def test_normalized_mapping_and_table_reference():
    workspace = RobotWorkspace.parse({**BOUNDS, "table_z_cm": -3, "y_max_cm": 20})
    assert workspace.map_normalized([-1, -1, -1]) == pytest.approx([.1, -.1, .02])
    assert workspace.map_normalized([0, 0, 0]) == pytest.approx([.2, .05, .07])
    assert workspace.map_normalized([1, 1, 1]) == pytest.approx([.3, .2, .12])
    assert workspace.contains(workspace.map_normalized([-1, 0, 1]))
    for xyz in ([float("nan"), 0, 0], [1.01, 0, 0], [0, 0]):
        with pytest.raises(ValueError):
            workspace.map_normalized(xyz)


def test_alignment_reports_signed_marker_adjustments_and_actual_workspace_failure():
    workspace = RobotWorkspace.parse({**BOUNDS, "table_z_cm": 5, "height_min_cm": 10, "height_max_cm": 30})
    details = alignment_readout(workspace, [.2, -.02, .115], [.25, .10, .16])
    assert details["marker_adjustment_cm"] == pytest.approx([-5., -12., -4.5])
    assert details["alignment_cm"] == pytest.approx(np.linalg.norm([5., 12., 4.5]))
    assert details["tool_in_workspace"] is False
    assert "Z 11.5 cm is below minimum 15.0 cm" in details["workspace_warning"]
    assert "Marker cannot align here" in details["workspace_warning"]
    assert details["workspace_m"]["min"] == pytest.approx([.1, -.1, .15])


@pytest.mark.parametrize("axis,value,expected", [(0, .05, "X 5.0 cm is below minimum 10.0 cm"),
                                                (0, .35, "X 35.0 cm is above maximum 30.0 cm"),
                                                (1, -.12, "Y -12.0 cm is below minimum -10.0 cm"),
                                                (1, .12, "Y 12.0 cm is above maximum 10.0 cm"),
                                                (2, .04, "Z 4.0 cm is below minimum 5.0 cm"),
                                                (2, .16, "Z 16.0 cm is above maximum 15.0 cm")])
def test_workspace_warning_names_each_violating_axis_and_edge(axis, value, expected):
    measured = [.2, 0., .1]
    measured[axis] = value
    details = alignment_readout(RobotWorkspace.parse(BOUNDS), measured, None)
    assert expected in details["workspace_warning"]
    assert details["marker_adjustment_cm"] is None
    assert details["alignment_cm"] is None


def test_alignment_never_fabricates_guidance_without_measurements():
    for measured in (None, [float("nan"), 0., .1]):
        details = alignment_readout(RobotWorkspace.parse(BOUNDS), measured, [.2, 0., .1])
        assert details["marker_adjustment_cm"] is None
        assert details["tool_in_workspace"] is None
        assert not details["workspace_warning"]


@pytest.mark.parametrize("change", [{"x_min_cm": 0}, {"height_min_cm": -1}, {"x_max_cm": 10},
                                    {"y_min_cm": float("nan")}, {"table_z_cm": True}, {"version": True}])
def test_invalid_configuration_cannot_replace_saved_workspace(tmp_path, change):
    store = WorkspaceStore(tmp_path / "workspace.json")
    assert store.snapshot()[0] is None
    store.save(BOUNDS)
    before = store.path.read_bytes()
    with pytest.raises(ValueError):
        store.save({**BOUNDS, **change})
    assert store.path.read_bytes() == before
    assert store.snapshot()[1] == 1
    assert not list(tmp_path.glob("*.tmp"))
    assert WorkspaceStore(store.path).snapshot()[0].payload() == BOUNDS


def test_failed_save_and_corrupt_file_leave_control_unconfigured(tmp_path, monkeypatch):
    store = WorkspaceStore(tmp_path / "workspace.json")
    store.path.write_text("{broken", encoding="utf-8")
    assert WorkspaceStore(store.path).snapshot()[0] is None
    def fail(*_):
        raise OSError("read-only")
    monkeypatch.setattr(type(store.path), "replace", fail)
    with pytest.raises(OSError):
        store.save(BOUNDS)
    assert store.snapshot()[0] is None
    assert not list(tmp_path.glob("*.tmp"))


class CartesianSolver:
    def __init__(self):
        self.kinematics = self

    def forward_kinematics(self, q):
        pose = np.eye(4)
        pose[:3, 3] = np.asarray(q[:3]) / 100
        return pose

    def solve_pose(self, pose, seed):
        q = np.r_[pose[:3, 3] * 100, seed[3:5]]
        return SimpleNamespace(accepted=True, reason="", joints_deg=q, pose=self.forward_kinematics(q))


class Harness:
    def __init__(self, tmp_path, configured=True):
        self.now = 10.
        self.store = WorkspaceStore(tmp_path / "workspace.json")
        if configured:
            self.store.save(BOUNDS)
        self.source = ArucoControl(self.store, clock=lambda: self.now)
        self.source.begin("robot-axes", "normalized", True)
        self.feedback = dict(armed=False, moving=False, measured=[20., 0., 10., 0., 0., 35.],
                             sent=[20., 0., 10., 0., 0., 35.], hz=50, watchdog_s=.2,
                             rates=[60.] * 6, fault=False, reason="", arm_torque="holding")
        self.operations = []
        self.deltas = []
        self.on_solve = lambda: None
        self.solver = CartesianSolver()
        self.loop = ControlLoop(SimpleNamespace(solve=self.solve), None, None, Event(), 1., "Sim",
                                clock=lambda: self.now)

    def solve(self, measured, delta, **kwargs):
        self.deltas.append(delta)
        result = solve_intent(self.solver, measured, delta, [0, 100], **kwargs)
        if any(delta):
            self.on_solve()
        return result

    def command(self, action):
        self.source.command(json.dumps(dict(type="aruco-control", action=action)))

    def request(self, op, **values):
        self.operations.append(op)
        if op == "arm":
            self.feedback["armed"] = True
        elif op in ("disarm", "hold"):
            self.feedback["moving"] = False
            if op == "disarm":
                self.feedback["armed"] = False
        elif op == "move":
            self.feedback.update(sent=values["target"], measured=values["target"], moving=True)
        elif op == "capture":
            self.feedback["arm_torque"] = "holding"
        return self.feedback.copy()

    def tick(self):
        self.feedback = self.source.tick(self.loop, self, self.feedback, period=.02, elapsed=.02,
                                         mode="Sim", started=self.now, connected_at=10., feedback_count=1)
        self.now += .02

    def enable(self):
        self.source.publish_pose(True, (0, 0, 0))
        self.command("enable")
        self.tick()
        assert self.source.enabled


def test_alignment_requires_all_bounds_and_explicit_enable(tmp_path):
    h = Harness(tmp_path, configured=False)
    h.source.publish_pose(True, (0, 0, 0))
    h.command("enable")
    h.tick()
    assert not h.source.enabled and "arm" not in h.operations
    h.store.save(BOUNDS)
    h.source.snapshot()  # A workspace save invalidates prior enable requests.
    h.source.publish_pose(True, (1, 0, 0))
    h.command("enable")
    h.tick()
    assert not h.source.enabled and "Align" in h.source.telemetry()["reason"]
    h.source.publish_pose(True, (0, 0, 0))
    h.command("heartbeat")
    h.tick()
    assert not h.source.enabled  # Alignment alone cannot enable motion.
    h.command("enable")
    h.tick()
    assert h.source.enabled


def test_guidance_uses_one_current_marker_target_and_clears_after_loss(tmp_path):
    h = Harness(tmp_path)
    h.source.publish_pose(True, (.5, .4, .1))
    h.command("heartbeat")
    h.tick()
    assert h.source.telemetry()["marker_adjustment_cm"] == pytest.approx([-5., -4., -.5])
    h.source.publish_pose(True, (-.5, -.4, -.1))
    details = h.source.telemetry()
    assert details["marker_adjustment_cm"] == pytest.approx([5., 4., .5])
    assert details["alignment_cm"] == pytest.approx(np.linalg.norm(details["marker_adjustment_cm"]))
    assert details["tool_in_workspace"] is True
    h.source.publish_pose(False, (0., 0., 0.))
    assert h.source.telemetry()["marker_adjustment_cm"] is None
    assert h.source.telemetry()["alignment_cm"] is None
    assert h.source.telemetry()["tool_in_workspace"] is True


def test_axis_guidance_preserves_the_total_two_centimeter_enable_gate(tmp_path):
    h = Harness(tmp_path)
    h.source.publish_pose(True, (.15, .15, .3))  # 1.5 cm on every axis, 2.6 cm total.
    h.command("enable")
    h.tick()
    details = h.source.telemetry()
    assert details["marker_adjustment_cm"] == pytest.approx([-1.5, -1.5, -1.5])
    assert not h.source.enabled
    h.source.publish_pose(True, (.1, .1, .2))  # 1 cm on each axis, 1.7 cm total.
    h.command("enable")
    h.tick()
    assert h.source.enabled


def test_out_of_workspace_start_reports_bounds_even_without_a_tracked_marker(tmp_path):
    h = Harness(tmp_path)
    h.feedback["measured"][2] = 4.  # 4 cm, below the configured 5 cm floor.
    h.command("heartbeat")
    h.tick()
    details = h.source.telemetry()
    assert details["tool_in_workspace"] is False
    assert details["marker_adjustment_cm"] is None
    assert "Z 4.0 cm is below minimum 5.0 cm" in details["reason"]
    assert not details["can_enable"] and "move" not in h.operations


def test_stationary_marker_is_an_absolute_target_with_bounded_speed_and_no_rotation(tmp_path):
    h = Harness(tmp_path)
    h.enable()
    h.loop.speed = .25
    goal = [.21, .01, .11]
    for _ in range(80):
        h.source.publish_pose(True, (.1, .1, .2))
        h.command("heartbeat")
        before = np.asarray(h.feedback["sent"][:3]) / 100
        h.tick()
        after = np.asarray(h.feedback["sent"][:3]) / 100
        assert np.linalg.norm(after - before) <= .1 * .25 * .02 + 1e-10
        assert np.linalg.norm(after - goal) <= np.linalg.norm(before - goal) + 1e-10
        assert h.feedback["sent"][5] == 35.
    assert after == pytest.approx(goal, abs=.00005)
    assert all(delta[3:] == [0., 0., 0.] for delta in h.deltas)


def test_loss_and_pose_timeout_resume_but_heartbeat_expiry_requires_enable(tmp_path):
    h = Harness(tmp_path)
    h.enable()
    h.source.publish_pose(True, (.5, 0, 0))
    h.tick()
    h.source.publish_pose(False, (0, 0, 0))
    h.tick()
    assert h.operations[-1] == "hold" and h.source.enabled
    h.source.publish_pose(True, (.5, 0, 0))
    h.command("heartbeat")
    h.tick()
    assert h.operations[-1] == "move"
    for _ in range(7):
        h.command("heartbeat")
        h.tick()
    assert h.source.enabled and h.operations[-1] != "move"
    h.source.publish_pose(True, (.5, 0, 0))
    h.tick()
    assert h.operations[-1] == "move"
    h.now += .11
    h.source.publish_pose(True, (.5, 0, 0))
    h.command("heartbeat")
    h.tick()
    assert not h.source.enabled and h.operations[-1] == "disarm"


@pytest.mark.parametrize("interrupt", ["hold", "invalidate", "workspace", "loss", "heartbeat", "disconnect"])
def test_interruptions_during_solve_never_issue_the_candidate(tmp_path, interrupt):
    h = Harness(tmp_path)
    h.enable()
    h.source.publish_pose(True, (.5, 0, 0))
    def interruption():
        if interrupt in ("hold", "invalidate"):
            h.command(interrupt)
        elif interrupt == "workspace":
            h.store.save({**BOUNDS, "x_max_cm": 31})
        elif interrupt == "loss":
            h.source.publish_pose(False, (0, 0, 0))
            h.source.publish_pose(True, (.5, 0, 0))
        elif interrupt == "disconnect":
            h.source.end()
        else:
            h.now += .11
    h.on_solve = interruption
    h.tick()
    assert "move" not in h.operations
    assert h.source.enabled is (interrupt == "loss")


def test_outside_measured_tool_cancels_without_automatic_return(tmp_path):
    h = Harness(tmp_path)
    h.enable()
    h.feedback["measured"] = [5., 0., 10., 0., 0., 35.]
    h.tick()
    assert not h.source.enabled and h.operations[-1] == "disarm"
    assert "outside" in h.source.telemetry()["reason"]


def test_released_arm_requires_holding_before_enable(tmp_path):
    h = Harness(tmp_path)
    h.feedback["arm_torque"] = "released"
    h.source.publish_pose(True, (0, 0, 0))
    h.command("enable")
    h.tick()
    assert not h.source.enabled
    h.command("hold")
    h.tick()
    assert h.operations[-1] == "capture" and not h.source.enabled
    h.enable()


def test_workspace_checks_intermediate_fk_and_both_measured_and_sent_paths():
    class DippingTool(CartesianSolver):
        def forward_kinematics(self, q):
            pose = super().forward_kinematics(q)
            pose[2, 3] = .04 + .02 * (q[0] - 20) ** 2
            return pose
    solver = DippingTool()
    limits = RobotWorkspace.parse(BOUNDS).limits()
    left, right = [19, 0, 10, 0, 0], [21, 0, 10, 0, 0]
    assert workspace_path_clear(solver, left, (left,), limits)
    assert workspace_path_clear(solver, right, (right,), limits)
    assert not workspace_path_clear(solver, right, (left,), limits)
    assert not workspace_path_clear(solver, right, (right, left), limits)
    with pytest.raises(ValueError):
        workspace_path_clear(solver, right, (left,), {"min": [0, 0, 0], "max": [0, 0, 0]})


def test_boundary_rejection_does_not_block_fresh_inward_motion():
    solver = CartesianSolver()
    q = [30., 0., 10., 0., 0., 35.]
    kwargs = dict(sent=q, desired_xyz_m=[.3, 0, .1], joint_step_limits=[2.] * 6,
                  workspace_m=RobotWorkspace.parse(BOUNDS).limits())
    outward = solve_intent(solver, q, [.002, 0, 0, 0, 0, 0], [0, 100], **kwargs)
    assert outward["target"] is None
    inward = solve_intent(solver, q, [-.002, 0, 0, 0, 0, 0], [0, 100], **kwargs)
    assert inward["target"] is not None
    assert inward["candidate_xyz_m"][0] < .3


def test_solver_tolerance_cannot_increase_absolute_target_speed_or_overshoot():
    class ImpreciseSolver(CartesianSolver):
        def solve_pose(self, pose, seed):
            result = super().solve_pose(pose, seed)
            result.joints_deg[0] += .01  # 0.1 mm beyond the requested point, within legacy tolerance.
            result.pose = self.forward_kinematics(result.joints_deg)
            return result
    q = [20., 0., 10., 0., 0., 35.]
    kwargs = dict(sent=q, desired_xyz_m=[.2, 0, .1], joint_step_limits=[2.] * 6)
    legacy = solve_intent(ImpreciseSolver(), q, [.002, 0, 0, 0, 0, 0], [0, 100], **kwargs)
    assert legacy["candidate_xyz_m"][0] > .202
    absolute = solve_intent(ImpreciseSolver(), q, [.002, 0, 0, 0, 0, 0], [0, 100],
                            workspace_m=RobotWorkspace.parse(BOUNDS).limits(), **kwargs)
    assert absolute["target"] is not None
    assert .2 < absolute["candidate_xyz_m"][0] <= .202 + 1e-8


def test_workspace_rejects_adapted_paths_that_escape_sideways():
    class SlidingSolver(CartesianSolver):
        limits_deg = np.array([[-90., 90.]] * 5)

        def solve_pose(self, pose, seed, *, position_only=False, **_):
            movement = pose[:3, 3] - self.forward_kinematics(seed)[:3, 3]
            q = seed.copy()
            q[:3] += movement * 100
            q[0] += movement[2] * 50
            return SimpleNamespace(accepted=position_only, reason="orientation", joints_deg=q,
                                   pose=self.forward_kinematics(q))
    q = [0.] * 6
    limits = dict(min=[-.1, -.1, -.1], max=[0., .1, .1])
    kwargs = dict(sent=q, joint_step_limits=[4.] * 6)
    free = solve_intent(SlidingSolver(), q, [0, 0, .002, 0, 0, 0], [0, 100], **kwargs)
    assert free["target"] is not None and free["candidate_xyz_m"][0] > 0
    guarded = solve_intent(SlidingSolver(), q, [0, 0, .002, 0, 0, 0], [0, 100], workspace_m=limits, **kwargs)
    assert guarded["target"] is None or guarded["candidate_xyz_m"][0] <= 0


def test_dashboard_save_requires_local_origin_and_current_session(tmp_path):
    store = WorkspaceStore(tmp_path / "workspace.json")
    with MobileWorkspaceDashboard("127.0.0.1", 0, workspace=store) as server:
        authority = f"127.0.0.1:{server.port}"
        client = HTTPConnection("127.0.0.1", server.port, timeout=2)
        headers = {"Content-Type": "application/json", "Origin": f"http://{authority}",
                   "X-Dashboard-Session": server.session_id}
        for changes in ({"Origin": "http://other"}, {"X-Dashboard-Session": "old"}):
            client.request("POST", "/api/robot-workspace", json.dumps(BOUNDS), {**headers, **changes})
            response = client.getresponse()
            assert response.status == 403
            response.read()
        assert store.snapshot()[0] is None
        client.request("POST", "/api/robot-workspace", json.dumps(BOUNDS), headers)
        response = client.getresponse()
        assert response.status == 200
        assert json.loads(response.read())["workspace"] == BOUNDS
        client.close()


def test_mobile_workspace_editor_preserves_edits_and_recovers_after_restart():
    node = shutil.which("node")
    if not node:
        pytest.skip("Node.js is needed for the workspace frontend check")
    page = Path(__file__).resolve().parents[1] / "scripts" / "mobile_teleop_dashboard.html"
    result = subprocess.run([node, str(Path(__file__).with_name("mobile_dashboard_workspace.cjs"))],
                            input=render_dashboard_page(page).decode("utf-8"), capture_output=True,
                            encoding="utf-8", timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr


def test_gateway_control_negotiation_and_duplicate_poses_do_not_refresh_motion(tmp_path):
    from websockets.sync.client import connect
    store = WorkspaceStore(tmp_path / "workspace.json")
    store.save(BOUNDS)
    with MobileGateway("127.0.0.1", 0, MobileInput(), "offline-token", workspace=store) as gateway:
        headers = {"Authorization": "Bearer offline-token", "X-Marker-Pose-Frame": "robot-axes",
                   "X-Marker-Pose-XYZ-Units": "normalized", CONTROL_HEADER: CONTROL_VERSION}
        with connect(f"ws://127.0.0.1:{gateway.port}{POSE_PATH}", additional_headers=headers, proxy=None) as client:
            assert client.response.headers[CONTROL_HEADER] == CONTROL_VERSION
            client.send(json.dumps(dict(type="aruco-control", action="heartbeat")))
            client.send(POSE_FRAME.pack(0, 1, 0, 0, 0, 90, -60, 170))
            deadline = monotonic() + 1
            while gateway.aruco.snapshot()["pose_at"] is None and monotonic() < deadline:
                sleep(.001)
            first = gateway.aruco.snapshot()["pose_at"]
            assert first is not None
            client.send(POSE_FRAME.pack(0, 1, .5, .5, .5, 0, 0, 0))
            client.send(json.dumps(dict(type="aruco-control", action="heartbeat")))
            sleep(.03)
            assert gateway.aruco.snapshot()["pose_at"] == first
            assert gateway.aruco.snapshot()["target_xyz_m"] == pytest.approx([.2, 0, .1])
        deadline = monotonic() + 1
        while gateway.aruco.active and monotonic() < deadline:
            sleep(.001)
        assert not gateway.aruco.active and not gateway.aruco.enabled


@pytest.mark.integration
@pytest.mark.skipif(os.environ.get("LEROBOT_WSL_TESTS") != "1", reason="Opt in to WSL2 IK + local motor simulation")
def test_wsl_solver_gateway_and_simulated_motor_follow_pause_resume_and_hold(tmp_path):
    """Explicit opt-in software integration; never SSH, deploy, or open serial."""
    from websockets.sync.client import connect
    from scripts.main_teleop_remote_ik import Worker, DEFAULT_WSL_PYTHON, local_simulated_link
    stop = Event()
    with ExitStack() as stack:
        worker = stack.enter_context(Worker("Ubuntu", DEFAULT_WSL_PYTHON))
        initial = [min(max(0., low), high) for low, high in worker.metadata["limits"]]
        zero = worker.solve(initial, [0.] * 6, sent=initial, desired_xyz_m=None,
                            joint_step_limits=[2.] * 6, correction=False, recovery_step_m=.001)
        x, y, z = zero["xyz_m"]
        store = WorkspaceStore(tmp_path / "workspace.json")
        store.save(dict(version=1, x_min_cm=(x - .03) * 100, x_max_cm=(x + .03) * 100,
                        y_min_cm=(y - .03) * 100, y_max_cm=(y + .03) * 100,
                        height_min_cm=(z - .03) * 100, height_max_cm=(z + .03) * 100, table_z_cm=0))
        phone = MobileInput()
        gateway = stack.enter_context(MobileGateway("127.0.0.1", 0, phone, "offline-token", workspace=store))
        loop = ControlLoop(worker, local_simulated_link(stack, worker, stop), phone, stop, .1, "Local simulation",
                           absolute_input=gateway.aruco)
        thread = Thread(target=loop.run, daemon=True)
        thread.start()
        stack.callback(thread.join, 2)
        stack.callback(stop.set)
        headers = {"Authorization": "Bearer offline-token", "X-Marker-Pose-Frame": "robot-axes",
                   "X-Marker-Pose-XYZ-Units": "normalized", CONTROL_HEADER: CONTROL_VERSION}
        with connect(f"ws://127.0.0.1:{gateway.port}{POSE_PATH}", additional_headers=headers, proxy=None) as client:
            seq = 0
            def send(action="heartbeat", nx=0., tracked=True):
                nonlocal seq
                client.send(json.dumps(dict(type="aruco-control", action=action)))
                client.send(POSE_FRAME.pack(seq, int(tracked), nx, 0, 0, 120, -30, 80))
                seq += 1
                sleep(.02)
                assert loop.error is None
            for _ in range(20):
                send()
            assert gateway.aruco.telemetry()["can_enable"], loop.status
            send("enable")
            for _ in range(30):
                send(nx=-.1)
            assert gateway.aruco.enabled, loop.status
            assert loop.status["xyz_m"][0] < x - .0002, loop.status
            for _ in range(10):
                send(tracked=False)
            paused = loop.status["sent"].copy()
            for _ in range(5):
                send(tracked=False)
            assert loop.status["sent"] == paused
            assert gateway.aruco.enabled
            for _ in range(20):
                send(nx=-.15)
            assert loop.status["sent"] != paused
            send("hold", nx=-.15)
            for _ in range(8):
                send(nx=-.15)
            assert not gateway.aruco.enabled
            assert loop.status["control_state"] == "Holding"
