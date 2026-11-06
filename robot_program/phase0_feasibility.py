"""CPU-only research harness. This module is never a motor controller.

Collision margins, exclusions and subdivision settings are experimental. A
sampled clear path is not a proof of continuous or physical clearance.
"""
from __future__ import annotations

from contextlib import contextmanager, ExitStack
from importlib import metadata
from itertools import combinations
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import struct
import sys
from tempfile import TemporaryDirectory
from time import perf_counter_ns
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

from robot_program.lerobot_ik_controller import ARM_JOINTS, LeRobotIKController

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "so101_new_calib.urdf"
FIXTURES = ROOT / "tests/fixtures/phase0_poses.json"


def forbidden_hardware(*args, **kwargs):
    raise RuntimeError("Phase 0 forbids physical serial, motor and torque operations")


@contextmanager
def hardware_forbidden():
    """Block the actual serial/bus entry points, including accidental connect."""
    import serial
    from lerobot.motors.feetech import FeetechMotorsBus
    from robot_program.lerobot_compat import UpstreamSO101Follower

    with ExitStack() as stack:
        stack.enter_context(patch.object(serial.Serial, "open", forbidden_hardware))
        for cls, names in (
            (FeetechMotorsBus, ("connect", "sync_read", "sync_write", "write",
                               "enable_torque", "disable_torque")),
            (UpstreamSO101Follower, ("connect", "send_action")),
        ):
            for name in names:
                if hasattr(cls, name):
                    stack.enter_context(patch.object(cls, name, forbidden_hardware))
        yield


def environment():
    packages = sorted((d.metadata["Name"], d.version) for d in metadata.distributions())
    cpu = platform.processor()
    if Path("/proc/cpuinfo").is_file():
        cpu = next((line.split(":", 1)[1].strip() for line in Path("/proc/cpuinfo").read_text().splitlines()
                    if line.startswith("model name")), cpu)
    return {
        "status": "OFFLINE VERIFIED", "python": sys.version,
        "executable": sys.executable, "platform": platform.platform(),
        "machine": platform.machine(), "cpu": cpu, "logical_cpus": os.cpu_count(),
        "packages": dict(packages),
    }


def model_inventory(model=MODEL):
    model = Path(model).resolve()
    root = ET.parse(model).getroot()
    meshes = sorted({n.attrib["filename"] for n in root.findall(".//mesh")})
    missing = [name for name in meshes if not (model.parent / name).is_file()]
    if missing:
        raise ValueError(f"Missing model assets: {missing}")
    manifest = json.loads((ROOT / "assets/SOURCES.json").read_text())
    for entry in manifest["files"]:
        data = (ROOT / entry["path"]).read_bytes()
        if hashlib.sha256(data).hexdigest() != entry["sha256"]:
            raise ValueError(f"Asset hash mismatch: {entry['path']}")
    return {
        "status": "OFFLINE VERIFIED", "urdf_sha256": sha256(model),
        "calibration_sha256": {p.name: sha256(p) for p in sorted((ROOT / "calibration").glob("*.json"))},
        "mesh_count": len(meshes), "collision_instances": len(root.findall(".//collision")),
        "mesh_triangles": sum(e["triangles"] for e in manifest["files"]),
        "mesh_bytes": sum(e["bytes"] for e in manifest["files"]),
        "links": [n.attrib["name"] for n in root.findall("link")],
        "joints": [{**n.attrib, "parent": n.find("parent").attrib["link"],
                    "child": n.find("child").attrib["link"],
                    "origin": n.find("origin").attrib if n.find("origin") is not None else {},
                    "axis": n.find("axis").attrib if n.find("axis") is not None else {},
                    "limit": n.find("limit").attrib if n.find("limit") is not None else {}}
                   for n in root.findall("joint")],
        "physical_alignment": "UNVERIFIED: no new hardware measurements",
    }


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def offline_controller(model=MODEL):
    if metadata.version("lerobot") != "0.6.1":
        raise RuntimeError("Phase 0 solver benchmark requires isolated LeRobot 0.6.1")
    from robot_program.lerobot_compat import SO101FollowerConfig
    from robot_program.so101_follower import SO101Follower

    # Construction loads calibration and bus conversion only. Never enter this
    # follower's context, invoke from_port(), or connect to it.
    follower = SO101Follower(SO101FollowerConfig(
        port="/dev/PHASE0_FORBIDDEN", id="lerobot_arm", use_degrees=True,
    ))
    follower.connect = forbidden_hardware
    follower.send_action = forbidden_hardware
    controller = LeRobotIKController(follower, urdf_path=Path(model))
    return controller


def finite_joints(value, count=6):
    q = np.asarray(value, dtype=float)
    if q.shape != (count,) or not np.isfinite(q).all():
        raise ValueError(f"Expected {count} finite joint values")
    return q


def path_samples(start, end, resolution_deg):
    start, end = finite_joints(start), finite_joints(end)
    if not math.isfinite(resolution_deg) or resolution_deg <= 0:
        raise ValueError("Path resolution must be finite and positive")
    steps = max(1, math.ceil(float(np.max(np.abs(end - start))) / resolution_deg))
    if steps > 10000:
        raise ValueError("Path exceeds offline subdivision budget")
    return np.linspace(start, end, steps + 1)


def check_path(checker, start, end, resolution_deg, margin_m, *, deadline_ns=None, clock_ns=perf_counter_ns):
    if not math.isfinite(margin_m) or margin_m < 0:
        raise ValueError("Margin must be finite and nonnegative")
    samples = path_samples(start, end, resolution_deg)
    minimum = None
    for index, q in enumerate(samples):
        if deadline_ns is not None and clock_ns() >= deadline_ns:
            return {"accepted": False, "reason": "deadline", "checked_samples": index,
                    "total_samples": len(samples), "min_distance_m": minimum}
        if callable(getattr(checker, "query_with_margin", None)):
            result = checker.query_with_margin(q, margin_m)
            if result.get("margin_m") != margin_m:
                raise ValueError("Collision checker did not apply the requested margin")
            unsafe = result["colliding"]
        else:
            result = checker.query(q)
            distance = result["min_distance_m"]
            if not math.isfinite(distance):
                raise ValueError("Collision checker returned nonfinite clearance")
            minimum = distance if minimum is None else min(minimum, distance)
            unsafe = result["colliding"] or distance <= margin_m
        if unsafe:
            return {"accepted": False, "reason": "collision_or_margin",
                    "checked_samples": index + 1, "total_samples": len(samples),
                    "min_distance_m": minimum, "rejected_fraction": index / (len(samples) - 1)}
    if deadline_ns is not None and clock_ns() >= deadline_ns:
        return {"accepted": False, "reason": "deadline", "checked_samples": len(samples),
                "total_samples": len(samples), "min_distance_m": minimum}
    return {"accepted": True, "reason": "sampled_clear", "checked_samples": len(samples),
            "total_samples": len(samples), "min_distance_m": minimum}


class CollisionChecker:
    """Separate PlaCo model with explicit experimental link-pair selection."""

    def __init__(self, model=MODEL):
        import placo

        source = Path(model).resolve()
        tree = ET.parse(source)
        root = tree.getroot()
        links = sorted(n.attrib["name"] for n in root.findall("link") if n.find("collision") is not None)
        adjacent = {frozenset((n.find("parent").attrib["link"], n.find("child").attrib["link"]))
                    for n in root.findall("joint")}
        self.exclusions = [{"links": list(pair), "reason": "experimental directly connected/mating links; needs geometry review"}
                           for pair in combinations(links, 2) if frozenset(pair) in adjacent]
        self.pairs = [list(pair) for pair in combinations(links, 2) if frozenset(pair) not in adjacent]
        if not self.pairs:
            raise ValueError("No collision pairs configured")
        for node in root.findall(".//mesh"):
            path = (source.parent / node.attrib["filename"]).resolve()
            if not path.is_file():
                raise ValueError(f"Missing collision mesh: {path}")
            node.set("filename", str(path))
        self._temporary = TemporaryDirectory(prefix="lerobot-phase0-")
        try:
            directory = Path(self._temporary.name)
            tree.write(directory / "robot.urdf")
            pair_path = directory / "pairs.json"
            pair_path.write_text(json.dumps(self.pairs))
            self.robot = placo.RobotWrapper(str(directory / "robot.urdf"))
            self.robot.load_collision_pairs(str(pair_path))
            if not callable(getattr(self.robot, "self_collisions", None)) or not callable(getattr(self.robot, "distances", None)):
                raise RuntimeError("Collision/distance queries unavailable")
        except BaseException:
            self.close()
            raise

    def close(self):
        self._temporary.cleanup()

    def query(self, joints_deg):
        q = finite_joints(joints_deg)
        for name, value in zip((*ARM_JOINTS, "gripper"), np.deg2rad(q)):
            self.robot.set_joint(name, float(value))
        self.robot.update_kinematics()
        collisions = self.robot.self_collisions(False)
        distances = list(self.robot.distances())
        if not distances:
            raise RuntimeError("No collision distances returned; checker is not usable")
        values = [float(d.min_distance) for d in distances]
        if not np.isfinite(values).all():
            raise RuntimeError("Nonfinite collision distances")
        return {"colliding": bool(collisions), "collision_count": len(collisions),
                "min_distance_m": min(values), "distance_pair_count": len(values)}


class MarginCollisionChecker(CollisionChecker):
    """Alternative: Pinocchio/Coal collision requests with security margins.

    Unlike exact all-pair distances this computes a collision predicate. Clear
    results do not contain a measured minimum distance. Still experimental.
    """

    def __init__(self, model=MODEL):
        super().__init__(model)
        import pinocchio as pin

        self.pin = pin
        try:
            self.model, self.geometry, _ = pin.buildModelsFromUrdf(self._temporary.name + "/robot.urdf")
            self.geometry.removeAllCollisionPairs()
            included = {frozenset(pair) for pair in self.pairs}
            for i, j in combinations(range(len(self.geometry.geometryObjects)), 2):
                a, b = (self.geometry.geometryObjects[k] for k in (i, j))
                links = frozenset((self.model.frames[a.parentFrame].name, self.model.frames[b.parentFrame].name))
                if links in included:
                    self.geometry.addCollisionPair(pin.CollisionPair(i, j))
            self.data = self.model.createData()
            self.set_geometry(self.geometry)
            self.indices = [self.model.idx_qs[self.model.getJointId(n)] for n in (*ARM_JOINTS, "gripper")]
            if len(self.geometry.collisionPairs) != 85:
                raise RuntimeError("Expected all 85 explicitly mapped experimental geometry pairs")
            if not all(hasattr(r, "security_margin") for r in self.geometry_data.collisionRequests):
                raise RuntimeError("Native collision requests lack a margin API")
        except BaseException:
            self.close()
            raise

    def geometry_pair_names(self):
        objects = self.geometry.geometryObjects
        return [tuple(sorted((objects[pair.first].name, objects[pair.second].name)))
                for pair in self.geometry.collisionPairs]

    def set_geometry(self, geometry):
        geometry_data = self.pin.GeometryData(geometry)
        self.geometry = geometry
        self.geometry_data = geometry_data
        self._pair_margin_overrides = {}

    def set_pair_margin_overrides(self, overrides):
        """Set absolute meter thresholds for explicitly enabled mesh pairs."""
        indices = {pair: index for index, pair in enumerate(self.geometry_pair_names())}
        configured = {}
        for pair, margin in overrides.items():
            key = tuple(sorted(pair))
            if key not in indices:
                raise ValueError(f"Mesh pair is unknown or excluded: {key}")
            if not math.isfinite(margin) or margin < 0:
                raise ValueError(f"Mesh pair margin must be finite and nonnegative: {key}")
            index = indices[key]
            if index in configured:
                raise ValueError(f"Duplicate mesh pair margin: {key}")
            configured[index] = margin
        self._pair_margin_overrides = configured

    def query_with_margin(self, joints_deg, margin_m, *, all_hits=False):
        """Use the supplied default margin plus any configured mesh overrides."""
        values = finite_joints(joints_deg)
        if not math.isfinite(margin_m) or margin_m < 0:
            raise ValueError("Margin must be finite and nonnegative")
        q = self.pin.neutral(self.model)
        q[self.indices] = np.deg2rad(values)
        requests = self.geometry_data.collisionRequests
        for request in requests:
            request.security_margin = margin_m
        for index, margin in self._pair_margin_overrides.items():
            requests[index].security_margin = margin
        colliding = self.pin.computeCollisions(self.model, self.data, self.geometry,
                                               self.geometry_data, q, not all_hits)
        result = {"colliding": bool(colliding), "margin_m": margin_m,
                  "min_distance_m": None, "distance_pair_count": len(self.geometry.collisionPairs),
                  "backend": "Pinocchio/Coal security_margin predicate, stop at first collision"}
        if all_hits:
            result["backend"] = "Pinocchio/Coal security_margin predicate, all enabled pairs"
            result["geometry_pairs"] = [pair for pair, outcome in zip(
                self.geometry_pair_names(), self.geometry_data.collisionResults, strict=True
            ) if outcome.isCollision()]
        return result

    def query(self, joints_deg):
        return self.query_with_margin(joints_deg, 0.)


def native_margin_proof():
    import coal

    box = coal.Box(0.1, 0.1, 0.1)
    results = []
    for separation, margin, expected in ((0.05, 0., True), (0.3, 0., False),
                                         (0.101, 0., False), (0.101, 0.002, True),
                                         (0.104, 0.002, False), (0.104, 0.005, True)):
        request = coal.CollisionRequest()
        request.security_margin = margin
        result = coal.CollisionResult()
        coal.collide(box, coal.Transform3s(), box, coal.Transform3s(np.eye(3), np.array([separation, 0., 0.])), request, result)
        if result.isCollision() != expected:
            raise RuntimeError("Native security-margin analytic proof failed")
        results.append({"separation_m": separation, "margin_m": margin, "colliding": expected})
    return results


def native_mesh_margin_proof():
    """Repeat margin tests on triangle BVHs, the actual robot geometry type."""
    import pinocchio as pin

    vertices = np.array([[x, y, z] for x in (-0.05, 0.05) for y in (-0.05, 0.05) for z in (-0.05, 0.05)])
    triangles = [(0, 3, 1), (0, 2, 3), (4, 5, 7), (4, 7, 6),
                 (0, 1, 5), (0, 5, 4), (2, 6, 7), (2, 7, 3),
                 (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)]
    with TemporaryDirectory(prefix="phase0-mesh-boxes-") as directory:
        p = Path(directory)
        mesh = bytearray(b"Phase 0 synthetic cube".ljust(80, b"\0") + struct.pack("<I", len(triangles)))
        for indices in triangles:
            points = vertices[list(indices)]
            normal = np.cross(points[1] - points[0], points[2] - points[0])
            normal /= np.linalg.norm(normal)
            mesh.extend(struct.pack("<12fH", *normal, *points.flatten(), 0))
        (p / "cube.stl").write_bytes(mesh)
        geometry = f'<collision><geometry><mesh filename="{p / "cube.stl"}"/></geometry></collision>'
        (p / "robot.urdf").write_text(f'''<robot name="mesh_boxes"><link name="root"/>
          <link name="a">{geometry}</link><link name="b">{geometry}</link>
          <joint name="a_root" type="fixed"><parent link="root"/><child link="a"/></joint>
          <joint name="separation" type="prismatic"><parent link="root"/><child link="b"/>
          <axis xyz="1 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint></robot>''')
        model, collisions, _ = pin.buildModelsFromUrdf(str(p / "robot.urdf"))
        collisions.addCollisionPair(pin.CollisionPair(0, 1))
        data, geometry_data = model.createData(), pin.GeometryData(collisions)
        results = []
        for separation, margin, expected in ((0.05, 0., True), (0.3, 0., False),
                                             (0.101, 0., False), (0.101, 0.002, True),
                                             (0.104, 0.002, False), (0.104, 0.005, True)):
            geometry_data.collisionRequests[0].security_margin = margin
            actual = pin.computeCollisions(model, data, collisions, geometry_data, np.array([separation]), True)
            if bool(actual) != expected:
                raise RuntimeError("Triangle-mesh security-margin proof failed")
            results.append({"separation_m": separation, "margin_m": margin, "colliding": bool(actual)})
        return results


def synthetic_collision_proof():
    """Independent, analytic 10 cm boxes: overlap then 20 cm surface gap."""
    import placo

    xml = '''<robot name="phase0_boxes">
      <link name="root"/><link name="a"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
      <link name="b"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
      <joint name="a_root" type="fixed"><parent link="root"/><child link="a"/></joint>
      <joint name="separation" type="prismatic"><parent link="root"/><child link="b"/>
      <axis xyz="1 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint></robot>'''
    with TemporaryDirectory(prefix="phase0-boxes-") as directory:
        p = Path(directory)
        (p / "robot.urdf").write_text(xml)
        (p / "pairs.json").write_text('[["a", "b"]]')
        robot = placo.RobotWrapper(str(p / "robot.urdf"))
        robot.load_collision_pairs(str(p / "pairs.json"))
        results = []
        for position, expected in ((0.05, True), (0.3, False)):
            robot.set_joint("separation", position)
            robot.update_kinematics()
            collision = bool(robot.self_collisions(False))
            distances = list(robot.distances())
            if collision != expected or not distances:
                raise RuntimeError("Analytic collision fixture failed")
            distance = float(distances[0].min_distance)
            if not expected and not np.isclose(distance, 0.2, atol=1e-6):
                raise RuntimeError(f"Analytic separation distance failed: {distance}")
            results.append({"separation_m": position, "colliding": collision, "min_distance_m": distance})
        return results


def distribution(values):
    if not values or not np.isfinite(values).all():
        raise ValueError("Timing samples must be nonempty and finite")
    return {"samples": len(values), "median_ms": float(np.median(values)),
            "p95_ms": float(np.percentile(values, 95)), "p99_ms": float(np.percentile(values, 99)),
            "worst_ms": max(values), "variance_ms2": statistics.pvariance(values),
            "over_20ms": sum(v > 20 for v in values)}


def measure(operation, warmup=20, repetitions=500):
    if warmup < 0 or repetitions < 1:
        raise ValueError("Invalid benchmark iteration counts")
    for _ in range(warmup):
        operation()
    samples, outcomes, reasons = [], [], {}
    for _ in range(repetitions):
        start = perf_counter_ns()
        outcome = operation()
        samples.append((perf_counter_ns() - start) / 1e6)
        outcomes.append(outcome)
        reason = outcome["reason"]
        reasons[reason] = reasons.get(reason, 0) + 1
    accepted = sum(bool(o["accepted"]) for o in outcomes)
    return {**distribution(samples), "warmup": warmup, "accepted": accepted,
            "rejected": repetitions - accepted, "accepted_ratio": accepted / repetitions,
            "reasons": reasons, "last_outcome": outcomes[-1], "raw_ms": samples}


def ik_request(controller, case):
    q = finite_joints(case["joints_deg"], 5)
    target = np.array(controller.kinematics.forward_kinematics(q), copy=True)
    target[:3, 3] += np.array(case.get("translation_m", [0, 0, 0]), dtype=float)
    if "target_joints_deg" in case:
        target = np.array(controller.kinematics.forward_kinematics(finite_joints(case["target_joints_deg"], 5)), copy=True)
    return q, target


def ik_case(controller, case):
    q, target = ik_request(controller, case)
    result = controller.solve_pose(target, q)
    return {"accepted": result.accepted, "reason": result.reason,
            "joints_deg": result.joints_deg.tolist(), "target_xyz_m": target[:3, 3].tolist(),
            "achieved_xyz_m": result.pose[:3, 3].tolist(),
            "position_error_m": float(np.linalg.norm(target[:3, 3] - result.pose[:3, 3]))}


def retraction_replay(controller):
    from scripts.main_teleop_lerobot_ik import InputState, tick

    measured = np.array([0., 25., -12., 3., 0.])
    follower = controller.follower
    positions = {**dict(zip(ARM_JOINTS, measured)), "gripper": 20.}
    commands = []
    # Explicit fake feedback and command sink, with global physical entry points
    # still forbidden. No torque state, stop policy or motor registers simulated.
    follower.bus.sync_read = lambda *_: dict(positions)
    follower.send_action = lambda action: commands.append(dict(action))
    try:
        state = InputState(active=True)
        state.axes["LY"] = 1.
        for _ in range(8):
            result = tick(controller, state)
            if not result.accepted:
                raise RuntimeError(f"Retraction replay failed: {result.reason}")
        lead = float(np.max(np.abs(result.joints_deg - measured)))
        if len(commands) != 8 or lead <= 5:
            raise RuntimeError("Full-command retraction regression failed")
        return {"accepted": True, "reason": "eight_full_commands", "commands": commands,
                "max_command_lead_deg": lead, "measured_joints_deg": measured.tolist(),
                "physical_retraction": "UNVERIFIED"}
    finally:
        follower.send_action = forbidden_hardware
        del follower.bus.sync_read


def discover_robot_fixtures(controller, checker, count=2000):
    """Deterministic exploration; persist candidates, never infer exclusions."""
    rng = np.random.default_rng(0)
    clear, colliding, near = [], [], None
    searched = 0
    for index in range(count):
        searched = index + 1
        q = np.r_[rng.uniform(controller.limits_deg[:, 0], controller.limits_deg[:, 1]), rng.uniform(-10, 100)]
        result = checker.query(q)
        entry = {"joints_deg": q.tolist(), **result}
        if result["colliding"]:
            if len(colliding) < 8:
                colliding.append(entry)
        else:
            if len(clear) < 1000:
                clear.append(entry)
            if near is None or checker.query_with_margin(q, 0.005)["colliding"]:
                near = entry
        if len(clear) >= 1000 and colliding and near:
            break
        if (index + 1) % 200 == 0:
            print(f"Discovery: {index + 1} poses; clear={len(clear)} collision={len(colliding)}", flush=True)
    if not clear or not colliding:
        raise RuntimeError("Could not demonstrate both clear and colliding robot configurations")
    interior = None
    for _ in range(30000):
        a, b = (clear[i] for i in rng.choice(len(clear), size=2, replace=False))
        # First find a colliding midpoint cheaply; only then pay for a dense
        # path. This is fixture discovery, never a motion-validation shortcut.
        midpoint = (np.array(a["joints_deg"]) + np.array(b["joints_deg"])) / 2
        if not checker.query(midpoint)["colliding"]:
            continue
        result = check_path(checker, a["joints_deg"], b["joints_deg"], 2., 0.)
        if not result["accepted"] and 0 < result["rejected_fraction"] < 1:
            interior = {"start": a["joints_deg"], "end": b["joints_deg"], "result": result}
            break
    if interior is None:
        raise RuntimeError("No endpoint-clear/interior-colliding robot path found")
    return {"seed": 0, "search_limit": count, "poses_searched": searched, "clear": clear[0], "colliding": colliding[0],
            "near": near, "interior_collision": interior,
            "joint_order": [*ARM_JOINTS, "gripper"],
            "jaw_units": "URDF degrees only; NOT a normalized motor mapping"}
