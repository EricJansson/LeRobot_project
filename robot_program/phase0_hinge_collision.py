"""Experimental acceleration for the offline, fixed-profile collision study."""

import math

import numpy as np

from robot_program import phase0_feasibility as f


# This controls extra BVH traversal for the distance lower bound, independently
# of the pair's security margin. Keep a small positive search tolerance.
HINGE_BREAK_DISTANCE_M = 1e-6
# A stricter query can certify reuse across tiny numerical angle changes.
REUSE_PADDING_M = 1e-5


class HingeCollisionChecker:
    """Reuse an adjacent pair's verdict for the same relative geometry."""

    def __init__(self, reference, added_pairs, default_margin_m, overrides):
        self.pin = reference.pin
        self.model = reference.model
        self.geometry = reference.geometry
        self.indices = reference.indices
        self.data = self.model.createData()
        self.geometry_data = self.pin.GeometryData(self.geometry)
        self.pair_names = reference.geometry_pair_names()
        if not set(added_pairs) <= set(self.pair_names) or not set(overrides) <= set(self.pair_names):
            raise ValueError("Expected enabled mesh pairs in the hinge profile")

        self.margin_m = default_margin_m
        base_pairs, groups = [], {}
        objects = self.geometry.geometryObjects
        for index, (pair, name, request) in enumerate(zip(
            self.geometry.collisionPairs, self.pair_names, self.geometry_data.collisionRequests, strict=True
        )):
            margin = overrides.get(name, default_margin_m)
            if not math.isfinite(margin) or margin < 0:
                raise ValueError(f"Invalid margin for {name}")
            request.security_margin = margin
            if name not in added_pairs:
                base_pairs.append(index)
                continue

            parent_obj, child_obj = sorted((objects[pair.first], objects[pair.second]), key=lambda obj: obj.parentJoint)
            parent, child = parent_obj.parentJoint, child_obj.parentJoint
            joint_index = self.model.idx_qs[child]
            if (self.model.parents[child] != parent or self.model.nqs[child] != 1 or joint_index not in self.indices
                    or not self.model.joints[child].shortname().startswith("JointModelR")):
                raise ValueError(f"Expected a single modeled hinge between {name}")
            # Only the child moves in the parent frame. Every child point's
            # displacement is at most radius * angular change in radians.
            child_obj.geometry.computeLocalAABB()
            bounds = child_obj.geometry.aabb_local
            radius = np.linalg.norm(child_obj.placement.translation) + np.linalg.norm(
                np.maximum(np.abs(bounds.min_), np.abs(bounds.max_))
            )
            if not math.isfinite(radius) or radius <= 0:
                raise ValueError(f"Invalid child mesh bounds for {name}")
            request.break_distance = HINGE_BREAK_DISTANCE_M
            self.geometry_data.deactivateCollisionPair(index)
            groups.setdefault(joint_index, []).append((index, margin, REUSE_PADDING_M / radius))

        self.base_pairs = base_pairs
        self.hinge_pairs = groups
        self.reset_cache()

    def reset_cache(self):
        self.hinge_cache = {}

    def query_with_margin(self, joints_deg, margin_m, *, all_hits=False):
        if margin_m != self.margin_m:
            raise ValueError("Hinge checker requires the configured default margin")
        q = self.pin.neutral(self.model)
        q[self.indices] = np.deg2rad(f.finite_joints(joints_deg))
        base_hit = self.pin.computeCollisions(
            self.model, self.data, self.geometry, self.geometry_data, q, not all_hits
        )
        result = {"colliding": bool(base_hit), "margin_m": margin_m, "min_distance_m": None}
        if base_hit and not all_hits:
            return result

        hits = []
        for joint_index, pairs in self.hinge_pairs.items():
            angle = q[joint_index]
            for index, margin, max_shift in pairs:
                cached = self.hinge_cache.get(index)
                if cached is None or (angle != cached[0] and abs(angle - cached[0]) >= cached[2]):
                    # Direct pair queries use the placements computed above even
                    # though those pairs are disabled in the bulk query.
                    request = self.geometry_data.collisionRequests[index]
                    request.security_margin = margin + REUSE_PADDING_M
                    near = self.pin.computeCollision(self.geometry, self.geometry_data, index)
                    if near:
                        request.security_margin = margin
                        hit = self.pin.computeCollision(self.geometry, self.geometry_data, index)
                        cached = (angle, hit, 0.)
                    else:
                        cached = (angle, False, max_shift)
                    self.hinge_cache[index] = cached
                if cached[1]:
                    hits.append(index)
            if hits and not all_hits:
                break

        if all_hits:
            hits.extend(index for index in self.base_pairs if self.geometry_data.collisionResults[index].isCollision())
            result["geometry_pairs"] = [self.pair_names[index] for index in sorted(hits)]
        result["colliding"] = bool(hits)
        return result
