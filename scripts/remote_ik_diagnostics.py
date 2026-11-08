"""Summarize an FK-enriched trace without opening hardware or modifying it."""
import argparse
from collections import Counter
import json
from math import dist
from pathlib import Path


def summarize(trace):
    ticks = trace["ticks"]
    def maximum(pairs):
        values = [dist(a, b) * 1000 for a, b in pairs if a is not None and b is not None]
        return max(values, default=None)
    errors = {}
    for label, a, b in (("candidate_to_requested_mm", "candidate_xyz_m", "requested_xyz_m"),
                        ("issued_to_candidate_mm", "issued_xyz_m", "candidate_xyz_m"),
                        ("measured_to_prior_issued_mm", "measured_xyz_m", "issued_before_xyz_m")):
        errors[label] = maximum((t.get(a), t.get(b)) for t in ticks)
    resets = []
    for i, t in enumerate(ticks):
        if t.get("xyz_reset"):
            prior = t.get("desired_before_m")
            measured = t.get("measured_xyz_m")
            resets.append({"tick": i, "reason": t["xyz_reset"],
                           "prior_desired_minus_measured_z_mm": (prior[2] - measured[2]) * 1000
                           if prior is not None and measured is not None else None})
    joint_lag = {}
    for i, name in enumerate(("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")):
        joint_lag[name] = max((abs(t["issued_before"][i] - t["measured"][i]) for t in ticks
                              if t.get("issued_before") and t.get("measured")), default=None)
    durations = sorted((t["ack_s"] - t["tick_s"]) * 1000 for t in ticks)
    provenance = trace.get("provenance")
    return {"provenance": provenance or {"classification": "unknown version; supporting evidence only"},
            "model": trace.get("model"), "ticks": len(ticks), "errors": errors,
            "joint_lag_max_deg": joint_lag, "reference_changes": resets,
            "operations": dict(Counter(t.get("op") for t in ticks)),
            "rejections": dict(Counter(t.get("ik_reason") for t in ticks if t.get("ik_reason"))),
            "rate_limited_ticks": sum(bool(t.get("rate_limited")) for t in ticks),
            "round_trip_p95_ms": durations[int(.95 * (len(durations) - 1))] if durations else None,
            "measurement_alignment": "Measured sample precedes current command; compare with issued_before.",
            "physical_deflection": "Requires independent visual measurement; FK cannot establish this."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    args = parser.parse_args()
    trace = json.loads(args.trace.read_text(encoding="utf-8-sig"))
    print(json.dumps(summarize(trace), indent=2))


if __name__ == "__main__":
    main()
