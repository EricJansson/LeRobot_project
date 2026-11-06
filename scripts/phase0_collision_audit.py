"""Offline sensitivity study for the experimental SO-101 collision model."""

import argparse
from datetime import datetime, timezone
from itertools import product
import json
from pathlib import Path
from time import perf_counter_ns
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

from robot_program import phase0_feasibility as f


JOINTS = (*f.ARM_JOINTS, "gripper")
MARGINS_M = (0., 0.002, 0.005)
RESOLUTIONS_DEG = (2., 1., 0.5, 0.125)
SEED = 42
UNIFORM_PATHS = 512
PATHS_PER_BOUNDARY = 128
MAX_DELTA_DEG = 4.
FIXTURES = f.ROOT / "tests/fixtures/phase0_robot_collisions.json"
STUDY_RESOLUTIONS_DEG = (0.5, 0.125)
ELBOW_PAIRS = tuple((f"lower_arm_link_{index}", "upper_arm_link_1") for index in range(3))
GRIPPER_PAIRS = (("gripper_link_1", "moving_jaw_so101_v1_link_0"),
                 ("gripper_link_1", "wrist_link_1"))


def audit_excluded_pairs(checker, urdf, limits_deg):
    """Sweep each excluded hinge with only its adjacent meshes enabled."""
    geometry_objects = checker.geometry.geometryObjects
    groups = {}
    for index, geometry in enumerate(geometry_objects):
        name = checker.model.frames[geometry.parentFrame].name
        groups.setdefault(name, []).append(index)
    joints = {frozenset((joint.find("parent").attrib["link"], joint.find("child").attrib["link"])):
              joint.attrib["name"] for joint in urdf.findall("joint")}

    audit = []
    for exclusion in checker.exclusions:
        links = exclusion["links"]
        joint = joints[frozenset(links)]
        index = JOINTS.index(joint)
        lower, upper = limits_deg[index]
        angles = np.unique(np.r_[0., np.linspace(lower, upper, int(np.ceil(upper - lower)) + 1)])
        geometry = checker.geometry.copy()
        geometry.removeAllCollisionPairs()
        pairs = list(product(groups[links[0]], groups[links[1]]))
        for a, b in pairs:
            geometry.addCollisionPair(checker.pin.CollisionPair(a, b))
        data = checker.pin.GeometryData(geometry)
        q = checker.pin.neutral(checker.model)
        hits = [[] for _ in pairs]
        for angle in angles:
            q[checker.indices[index]] = np.deg2rad(angle)
            checker.pin.computeCollisions(checker.model, checker.data, geometry, data, q, False)
            for hit_angles, result in zip(hits, data.collisionResults, strict=True):
                if result.isCollision():
                    hit_angles.append(float(angle))

        mesh_pairs = []
        for (a, b), hit_angles in zip(pairs, hits, strict=True):
            mesh_pairs.append({
                "geometries": [geometry_objects[i].name for i in (a, b)],
                "meshes": [Path(geometry_objects[i].meshPath).name for i in (a, b)],
                "hits_at_neutral": 0. in hit_angles,
                "hit_angles_deg": hit_angles,
            })
        audit.append({"joint": joint, "links": links, "angles_deg": angles.tolist(), "mesh_pairs": mesh_pairs})
    return audit


def build_corpus(checker, limits_deg, transition):
    rng = np.random.default_rng(SEED)
    lower, upper = limits_deg.T
    starts = rng.uniform(lower, upper, size=(UNIFORM_PATHS, len(JOINTS)))
    corpus = [("uniform", start, np.clip(start + rng.uniform(-MAX_DELTA_DEG, MAX_DELTA_DEG, len(JOINTS)), lower, upper))
              for start in starts]
    fixture_samples = f.path_samples(transition["start"], transition["end"], RESOLUTIONS_DEG[-1])
    boundaries = []
    for margin in MARGINS_M:
        flags = [checker.query_with_margin(q, margin)["colliding"] for q in fixture_samples]
        edges = np.flatnonzero(np.diff(flags))
        if not len(edges):
            raise RuntimeError(f"Saved fixture has no collision boundary at {margin:g} m")
        for edge in edges:
            center = (fixture_samples[edge] + fixture_samples[edge + 1]) / 2.
            boundaries.append({"margin_m": margin, "center_deg": center.tolist()})
            # Nearby poses on both sides of the sampled boundary, including
            # short moves that graze it rather than following the saved path.
            for _ in range(PATHS_PER_BOUNDARY):
                start = np.clip(center + rng.uniform(-2., 2., len(JOINTS)), lower, upper)
                end = np.clip(start + rng.uniform(-MAX_DELTA_DEG, MAX_DELTA_DEG, len(JOINTS)), lower, upper)
                corpus.append(("boundary", start, end))
    return corpus, boundaries


def audit_sampling(checker, corpus):
    stats = []
    for margin in MARGINS_M:
        timings = [[] for _ in RESOLUTIONS_DEG]
        rejected = [0] * len(RESOLUTIONS_DEG)
        missed = [0] * (len(RESOLUTIONS_DEG) - 1)
        fine_only_clear = [0] * (len(RESOLUTIONS_DEG) - 1)
        witnesses = [None] * (len(RESOLUTIONS_DEG) - 1)
        for group, start, end in corpus:
            outcomes = []
            for index, resolution in enumerate(RESOLUTIONS_DEG):
                tick = perf_counter_ns()
                outcome = f.check_path(checker, start, end, resolution, margin)
                timings[index].append((perf_counter_ns() - tick) / 1e6)
                outcomes.append(outcome)
                rejected[index] += not outcome["accepted"]
            fine = outcomes[-1]
            for index, coarse in enumerate(outcomes[:-1]):
                if not coarse["accepted"] and fine["accepted"]:
                    fine_only_clear[index] += 1
                if coarse["accepted"] and not fine["accepted"]:
                    missed[index] += 1
                    if witnesses[index] is None:
                        witnesses[index] = {"group": group, "start_deg": start.tolist(),
                                            "end_deg": end.tolist(), "coarse": coarse, "fine": fine}
        stats.append({
            "margin_m": margin,
            "results": [{"resolution_deg": resolution, "rejected_paths": rejected[index],
                         "timing": f.distribution(timings[index]), "raw_ms": timings[index]}
                        for index, resolution in enumerate(RESOLUTIONS_DEG)],
            "coarse_clear_fine_reject": dict(zip(RESOLUTIONS_DEG[:-1], missed, strict=True)),
            "fine_clear_coarse_reject": dict(zip(RESOLUTIONS_DEG[:-1], fine_only_clear, strict=True)),
            "first_coarse_miss": dict(zip(RESOLUTIONS_DEG[:-1], witnesses, strict=True)),
        })
    return stats


def mesh_pair_mask(checker, exclusions):
    geometry = checker.geometry.copy()
    indices = {obj.name: index for index, obj in enumerate(geometry.geometryObjects)}
    added = []
    for group in exclusions:
        for pair in group["mesh_pairs"]:
            if len(pair["hit_angles_deg"]) < len(group["angles_deg"]):
                names = tuple(sorted(pair["geometries"]))
                a, b = (indices[name] for name in pair["geometries"])
                geometry.addCollisionPair(checker.pin.CollisionPair(a, b))
                added.append(names)
    return geometry, added


def retraction_paths(replay):
    start = [*replay["measured_joints_deg"], 0.]
    targets = [[*(command[f"{joint}.pos"] for joint in f.ARM_JOINTS), 0.]
               for command in replay["commands"]]
    return start, targets


def compare_retraction_masks(checker, exclusions, replay):
    """Test the narrower mask on fixed-feedback retraction at URDF jaw zero."""
    start, targets = retraction_paths(replay)
    comparisons = []
    for policy in ("link_exclusions", "persistent_mesh_exclusions"):
        if policy == "persistent_mesh_exclusions":
            geometry, _ = mesh_pair_mask(checker, exclusions)
            checker.set_geometry(geometry)
        results = []
        for margin in MARGINS_M:
            paths = [f.check_path(checker, start, end, 0.5, margin) for end in targets]
            first_rejection = None
            for end, path in zip(targets, paths, strict=True):
                if path["accepted"]:
                    continue
                rejected_pose = f.path_samples(start, end, 0.5)[path["checked_samples"] - 1]
                q = checker.pin.neutral(checker.model)
                q[checker.indices] = np.deg2rad(rejected_pose)
                checker.pin.computeCollisions(checker.model, checker.data, checker.geometry,
                                              checker.geometry_data, q, False)
                names = checker.geometry.geometryObjects
                hits = [[names[pair.first].name, names[pair.second].name]
                        for pair, result in zip(checker.geometry.collisionPairs,
                                                checker.geometry_data.collisionResults, strict=True)
                        if result.isCollision()]
                first_rejection = {"pose_deg": rejected_pose.tolist(), "geometry_pairs": hits}
                break
            results.append({"margin_m": margin, "paths": paths, "first_rejection": first_rejection})
        comparisons.append({"policy": policy, "geometry_pairs": len(checker.geometry.collisionPairs),
                            "results": results})
    return {"start_deg": start, "targets_deg": targets, "resolution_deg": 0.5,
            "policies": comparisons}


def inspect_path(checker, start, end, resolution, default_margin):
    outcome = f.check_path(checker, start, end, resolution, default_margin)
    if not outcome["accepted"]:
        pose = f.path_samples(start, end, resolution)[outcome["checked_samples"] - 1]
        outcome["first_rejection_deg"] = pose.tolist()
        outcome["geometry_pairs"] = checker.query_with_margin(
            pose, default_margin, all_hits=True)["geometry_pairs"]
    return outcome


def inspect_profile(checker, default_margin, overrides, start, targets, fixtures):
    checker.set_pair_margin_overrides(overrides)
    transition = fixtures["interior_collision"]
    poses = {name: fixtures[name]["joints_deg"] for name in ("clear", "colliding", "near")}
    poses.update(transition_start=transition["start"], transition_end=transition["end"])
    return {
        "geometry_pair_count": len(checker.geometry.collisionPairs),
        "default_margin_m": default_margin,
        "overrides": [{"geometry_pair": pair, "margin_m": margin} for pair, margin in sorted(overrides.items())],
        "fixture_hits": {name: checker.query_with_margin(pose, default_margin, all_hits=True)["geometry_pairs"]
                         for name, pose in poses.items()},
        "interior_fixture": {str(res): inspect_path(checker, transition["start"], transition["end"], res, default_margin)
                             for res in STUDY_RESOLUTIONS_DEG},
        "retraction": {str(res): [inspect_path(checker, start, end, res, default_margin) for end in targets]
                       for res in STUDY_RESOLUTIONS_DEG},
    }


def study_hinges(checker, exclusions, limits_deg, margin):
    evidence, boundaries = [], {}
    target_pairs = {*ELBOW_PAIRS, GRIPPER_PAIRS[1]}
    for group in exclusions:
        pairs = {tuple(sorted(pair["geometries"])): pair for pair in group["mesh_pairs"]}
        relevant = sorted(target_pairs.intersection(pairs))
        if not relevant:
            continue
        joint = group["joint"]
        index = JOINTS.index(joint)
        angles = np.asarray(group["angles_deg"])
        hits = []
        for angle in angles:
            pose = np.zeros(len(JOINTS))
            pose[index] = angle
            hits.append(set(checker.query_with_margin(pose, margin, all_hits=True)["geometry_pairs"]))
        for pair in relevant:
            observed = np.asarray([pair in sample for sample in hits])
            baseline = np.isin(angles, pairs[pair]["hit_angles_deg"])
            evidence.append({"joint": joint, "geometry_pair": pair, "meshes": pairs[pair]["meshes"],
                             "zero_margin_hit_angles_deg": angles[baseline].tolist(),
                             "profile_hit_angles_deg": angles[observed].tolist(),
                             "margin_only_hit_angles_deg": angles[observed & ~baseline].tolist(),
                             "lost_hit_angles_deg": angles[baseline & ~observed].tolist()})
            for flags in (baseline, observed):
                for edge in np.flatnonzero(np.diff(flags)):
                    center = float((angles[edge] + angles[edge + 1]) / 2)
                    boundaries.setdefault((joint, center), set()).add(pair)
    paths, witnesses = [], []
    for (joint, center), pairs in sorted(boundaries.items()):
        index = JOINTS.index(joint)
        start, end = np.zeros(len(JOINTS)), np.zeros(len(JOINTS))
        start[index], end[index] = np.clip([center - 2, center + 2], *limits_deg[index])
        paths.append((joint, start, end))
        witnesses.append({"joint": joint, "center_deg": center, "pairs": sorted(pairs),
                          "start_deg": start.tolist(), "end_deg": end.tolist()})
    return evidence, paths, witnesses


def study_sampling(checker, corpus, margin):
    coarse, fine = STUDY_RESOLUTIONS_DEG
    samples = {str(res): [] for res in STUDY_RESOLUTIONS_DEG}
    disagreements = {"coarse_clear_fine_reject": [], "fine_clear_coarse_reject": []}
    for index, (group, start, end) in enumerate(corpus):
        outcomes = []
        for resolution in STUDY_RESOLUTIONS_DEG:
            tick = perf_counter_ns()
            outcome = f.check_path(checker, start, end, resolution, margin)
            elapsed = (perf_counter_ns() - tick) / 1e6
            samples[str(resolution)].append({"ms": elapsed, "accepted": outcome["accepted"]})
            outcomes.append(outcome)
        if outcomes[0]["accepted"] != outcomes[1]["accepted"]:
            direction = "coarse_clear_fine_reject" if outcomes[0]["accepted"] else "fine_clear_coarse_reject"
            disagreements[direction].append({"index": index, "group": group,
                "start_deg": start.tolist(), "end_deg": end.tolist(),
                "coarse": inspect_path(checker, start, end, coarse, margin),
                "fine": inspect_path(checker, start, end, fine, margin)})
    report = {}
    for resolution, rows in samples.items():
        raw_ms = [row["ms"] for row in rows]
        accepted = sum(row["accepted"] for row in rows)
        overruns = [{"index": index, "group": group, "start_deg": start.tolist(), "end_deg": end.tolist(),
                     "accepted": row["accepted"], "ms": row["ms"]}
                    for index, ((group, start, end), row) in enumerate(zip(corpus, rows, strict=True)) if row["ms"] > 20]
        report[resolution] = {**f.distribution(raw_ms), "accepted": accepted, "rejected": len(rows) - accepted,
                              "raw_ms": raw_ms, "overruns": overruns}
    return {"timing": report, "disagreements": disagreements}


def study_timing(checker, controller, replay_requests, start, targets, fixtures, margin, *, path_check=f.check_path):
    cases = json.loads(f.FIXTURES.read_text(encoding="utf-8"))["cases"]
    cases += [{"name": "collision_near", "joints_deg": fixtures["near"]["joints_deg"][:5]},
              {"name": "retraction_16mm", "joints_deg": start[:5], "translation_m": [-0.016, 0., 0.]}]
    requests = []
    for case in cases:
        measured, target = f.ik_request(controller, case)
        jaw = fixtures["near"]["joints_deg"][5] if case["name"] == "collision_near" else 0.
        requests.append((case["name"], [*measured, jaw], target, None))
    requests.extend((f"replay_{index:02}", start, request, end) for index, (request, end) in
                    enumerate(zip(replay_requests, targets, strict=True), start=1))
    resolution = STUDY_RESOLUTIONS_DEG[0]
    timings = {}
    for name, path_start, target, replay_end in requests:
        measured, jaw = np.asarray(path_start[:5]), path_start[5]
        candidate = controller.solve_pose(target, measured)
        path_end = [*candidate.joints_deg, jaw] if candidate.accepted else None
        if replay_end is not None:
            if path_end is None:
                raise RuntimeError(f"{name} no longer solves: {candidate.reason}")
            np.testing.assert_allclose(path_end, replay_end, rtol=0., atol=1e-8)

        def combined():
            result = controller.solve_pose(target, measured)
            if not result.accepted:
                return {"accepted": False, "reason": f"ik: {result.reason}"}
            return path_check(checker, path_start, [*result.joints_deg, jaw], resolution, margin)

        timings[name] = {"start_deg": list(map(float, path_start)), "target_pose": target.tolist(),
                         "candidate_deg": path_end, "combined": f.measure(combined)}
        if path_end is not None:
            timings[name]["path_only"] = f.measure(lambda: path_check(checker, path_start, path_end, resolution, margin))
        print(f"Completed timing for {name}", flush=True)
    transition = fixtures["interior_collision"]
    timings["interior_collision"] = {"path_only": f.measure(lambda: path_check(
        checker, transition["start"], transition["end"], resolution, margin))}
    return {"resolution_deg": resolution, "cases": timings}


def pair_margin_study(checker, controller, exclusions, limits_deg, fixtures):
    with patch.object(controller, "solve_pose", wraps=controller.solve_pose) as solve:
        replay = f.retraction_replay(controller)
        requests = [call.args[0].copy() for call in solve.call_args_list]
    start, targets = retraction_paths(replay)
    default = 0.002
    corpus, boundaries = build_corpus(checker, limits_deg, fixtures["interior_collision"])
    profiles = {"link_uniform_2mm": inspect_profile(checker, default, {}, start, targets, fixtures)}
    geometry, added = mesh_pair_mask(checker, exclusions)
    if len(geometry.collisionPairs) != 115 or len(added) != 30 or not {*ELBOW_PAIRS, *GRIPPER_PAIRS} <= set(added):
        raise RuntimeError("The expected 115-pair experimental mask has changed")
    checker.set_geometry(geometry)
    for margin in MARGINS_M:
        profiles[f"mesh_uniform_{margin * 1000:g}mm"] = inspect_profile(checker, margin, {}, start, targets, fixtures)
    broad = dict.fromkeys(GRIPPER_PAIRS, 0.00025)
    focused = {pair: 0.00025 for pair in added if pair not in ELBOW_PAIRS}
    profiles["broad"] = inspect_profile(checker, default, broad, start, targets, fixtures)
    profile = profiles["focused"] = inspect_profile(checker, default, focused, start, targets, fixtures)
    probes = []
    for pair in GRIPPER_PAIRS:
        for margin in (0., 0.00025, 0.0005, 0.001):
            checker.set_pair_margin_overrides({**focused, pair: margin})
            paths = {str(res): [inspect_path(checker, start, end, res, default) for end in targets]
                     for res in STUDY_RESOLUTIONS_DEG}
            probes.append({"geometry_pair": pair, "margin_m": margin, "retraction": paths})
    checker.set_pair_margin_overrides(focused)
    evidence, hinge_paths, hinge_boundaries = study_hinges(checker, exclusions, limits_deg, default)
    print(f"Comparing sampling grids over {len(corpus)} seeded paths and {len(hinge_paths)} hinge paths", flush=True)
    for _ in range(20):
        checker.query_with_margin(start, default)
    sampling = study_sampling(checker, corpus + hinge_paths, default)
    timing = study_timing(checker, controller, requests, start, targets, fixtures, default)
    fixture_hits = profile["fixture_hits"]
    checks = {
        "all_known_hinge_hits_retained": len(evidence) == 4 and all(
            pair["zero_margin_hit_angles_deg"] and not pair["lost_hit_angles_deg"] for pair in evidence),
        "all_retractions_clear": all(outcome["accepted"] for grid in profile["retraction"].values() for outcome in grid),
        "clear_and_colliding_fixtures_preserved": not fixture_hits["clear"] and bool(fixture_hits["colliding"]),
        "interior_collision_with_clear_endpoints": not fixture_hits["transition_start"] and
            not fixture_hits["transition_end"] and all(not outcome["accepted"] and 0 < outcome["rejected_fraction"] < 1
                                                      for outcome in profile["interior_fixture"].values()),
        "all_combined_replays_clear": all(timing["cases"][f"replay_{i:02}"]["combined"]["rejected"] == 0 for i in range(1, 9)),
    }
    return {"timestamp_utc": datetime.now(timezone.utc).isoformat(), "checks": checks, "profiles": profiles,
            "retraction": {"start_deg": start, "targets_deg": targets, "max_command_lead_deg": replay["max_command_lead_deg"],
                           "jaw_deg": 0.},
            "gripper_override_probes": probes, "hinge_evidence": evidence,
            "path_corpus": {"seed": SEED, "max_joint_delta_deg": MAX_DELTA_DEG, "uniform_paths": UNIFORM_PATHS,
                            "paths_per_boundary": PATHS_PER_BOUNDARY, "seeded_paths": len(corpus),
                            "seeded_boundaries": boundaries, "hinge_paths": hinge_boundaries},
            "sampling": sampling, "timing": timing}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--study", choices=("exclusions", "pair-margins"), default="exclusions")
    args = parser.parse_args()
    model = f.model_inventory()
    urdf = ET.parse(f.MODEL).getroot()
    jaw = urdf.find("joint[@name='gripper']/limit")
    jaw_limits = np.rad2deg([float(jaw.attrib[key]) for key in ("lower", "upper")])
    with f.hardware_forbidden():
        controller = f.offline_controller()
        limits_deg = np.vstack((controller.limits_deg, jaw_limits))
        checker = f.MarginCollisionChecker()
        try:
            exclusions = audit_excluded_pairs(checker, urdf, limits_deg)
            print("Adjacent mesh sweeps complete", flush=True)
            fixtures = json.loads(FIXTURES.read_text(encoding="utf-8"))
            if args.study == "pair-margins":
                findings = pair_margin_study(checker, controller, exclusions, limits_deg, fixtures)
                findings["ik_fixture_sha256"] = f.sha256(f.FIXTURES)
            else:
                corpus, boundaries = build_corpus(checker, limits_deg, fixtures["interior_collision"])
                print(f"Comparing sampling grids over {len(corpus)} short paths", flush=True)
                for _ in range(20):
                    checker.query_with_margin(np.zeros(len(JOINTS)), 0.)
                findings = {
                    "path_corpus": {"seed": SEED, "max_joint_delta_deg": MAX_DELTA_DEG,
                                    "uniform_paths": UNIFORM_PATHS, "paths_per_boundary": PATHS_PER_BOUNDARY,
                                    "boundaries": boundaries, "total_paths": len(corpus)},
                    "sampling": audit_sampling(checker, corpus),
                    "retraction_mask_comparison": compare_retraction_masks(checker, exclusions, f.retraction_replay(controller)),
                }
        finally:
            checker.close()
    report = {
        "status": "OFFLINE VERIFIED", "environment": f.environment(),
        "model_sha256": model["urdf_sha256"], "calibration_sha256": model["calibration_sha256"],
        "fixture_sha256": f.sha256(FIXTURES), "audit_sha256": f.sha256(__file__),
        "checker_sha256": f.sha256(Path(f.__file__)),
        "joints": JOINTS, "limits_deg": limits_deg.tolist(), "jaw_units": "URDF geometry degrees",
        "excluded_pair_sweeps": exclusions,
        **findings,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return 0 if all(findings.get("checks", {}).values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
