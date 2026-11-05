"""Offline sensitivity study for the experimental SO-101 collision model."""

import argparse
from itertools import product
import json
from pathlib import Path
from time import perf_counter_ns
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


def compare_retraction_masks(checker, exclusions, replay):
    """Test the narrower mask on fixed-feedback retraction at URDF jaw zero."""
    start = [*replay["measured_joints_deg"], 0.]
    targets = [[*(command[f"{joint}.pos"] for joint in f.ARM_JOINTS), 0.]
               for command in replay["commands"]]
    comparisons = []
    for policy in ("link_exclusions", "persistent_mesh_exclusions"):
        if policy == "persistent_mesh_exclusions":
            geometry = checker.geometry.copy()
            indices = {obj.name: index for index, obj in enumerate(geometry.geometryObjects)}
            for group in exclusions:
                for pair in group["mesh_pairs"]:
                    if len(pair["hit_angles_deg"]) < len(group["angles_deg"]):
                        a, b = (indices[name] for name in pair["geometries"])
                        geometry.addCollisionPair(checker.pin.CollisionPair(a, b))
            checker.geometry = geometry
            checker.geometry_data = checker.pin.GeometryData(geometry)
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
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
            transition = json.loads(FIXTURES.read_text(encoding="utf-8"))["interior_collision"]
            corpus, boundaries = build_corpus(checker, limits_deg, transition)
            print(f"Comparing sampling grids over {len(corpus)} short paths", flush=True)
            # Warm the same query used by the timed sweeps.
            for _ in range(20):
                checker.query_with_margin(np.zeros(len(JOINTS)), 0.)
            sampling = audit_sampling(checker, corpus)
            retraction_masks = compare_retraction_masks(checker, exclusions, f.retraction_replay(controller))
        finally:
            checker.close()
    report = {
        "status": "OFFLINE VERIFIED", "environment": f.environment(),
        "model_sha256": model["urdf_sha256"], "calibration_sha256": model["calibration_sha256"],
        "fixture_sha256": f.sha256(FIXTURES), "audit_sha256": f.sha256(__file__),
        "checker_sha256": f.sha256(Path(f.__file__)),
        "joints": JOINTS, "limits_deg": limits_deg.tolist(), "jaw_units": "URDF geometry degrees",
        "excluded_pair_sweeps": exclusions,
        "path_corpus": {"seed": SEED, "max_joint_delta_deg": MAX_DELTA_DEG,
                        "uniform_paths": UNIFORM_PATHS, "paths_per_boundary": PATHS_PER_BOUNDARY,
                        "boundaries": boundaries, "total_paths": len(corpus)},
        "sampling": sampling,
        "retraction_mask_comparison": retraction_masks,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
