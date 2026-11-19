"""Saved tool workspace, expressed in the IK base frame (never joint limits)."""

from dataclasses import dataclass
import json
import os
from math import isfinite
from pathlib import Path
from tempfile import NamedTemporaryFile
from threading import Lock


WORKSPACE_PATH = Path(__file__).resolve().parents[1] / "mobile-teleop.workspace.json"
FIELDS = ("x_min_cm", "x_max_cm", "y_min_cm", "y_max_cm",
          "height_min_cm", "height_max_cm", "table_z_cm")


@dataclass(frozen=True)
class RobotWorkspace:
    x_min_cm: float
    x_max_cm: float
    y_min_cm: float
    y_max_cm: float
    height_min_cm: float
    height_max_cm: float
    table_z_cm: float

    @classmethod
    def parse(cls, data):
        if (not isinstance(data, dict) or set(data) != {"version", *FIELDS}
                or type(data["version"]) is not int or data["version"] != 1):
            raise ValueError("Expected all seven workspace fields and version 1")
        if any(type(data[key]) not in (int, float) or not isfinite(data[key]) for key in FIELDS):
            raise ValueError("Enter finite numbers for every workspace field")
        workspace = cls(**{key: float(data[key]) for key in FIELDS})
        if workspace.x_min_cm <= 0 or workspace.height_min_cm < 0:
            raise ValueError("Minimum X must be positive and minimum height must be nonnegative")
        if any(low >= high for low, high in zip(workspace.minimum_m, workspace.maximum_m)):
            raise ValueError("Every minimum must be less than its maximum")
        if not all(isfinite(value) for value in (*workspace.minimum_m, *workspace.maximum_m)):
            raise ValueError("Workspace coordinates overflow the robot frame")
        return workspace

    @property
    def minimum_m(self):
        return (self.x_min_cm / 100, self.y_min_cm / 100,
                self.table_z_cm / 100 + self.height_min_cm / 100)

    @property
    def maximum_m(self):
        return (self.x_max_cm / 100, self.y_max_cm / 100,
                self.table_z_cm / 100 + self.height_max_cm / 100)

    def payload(self):
        return {"version": 1, **{key: getattr(self, key) for key in FIELDS}}

    def limits(self):
        return {"min": list(self.minimum_m), "max": list(self.maximum_m)}

    def contains(self, xyz):
        return (xyz is not None and len(xyz) == 3 and all(isfinite(value) and low <= value <= high
                for value, low, high in zip(xyz, self.minimum_m, self.maximum_m)))

    def map_normalized(self, xyz):
        if len(xyz) != 3 or not all(isfinite(value) and -1 <= value <= 1 for value in xyz):
            raise ValueError("Normalized XYZ must be in [-1, 1]")
        # Preserve endpoints exactly, including the configured table clearance.
        return [low if value == -1 else high if value == 1 else low + (value + 1) * .5 * (high - low)
                for value, low, high in zip(xyz, self.minimum_m, self.maximum_m)]


class WorkspaceStore:
    def __init__(self, path=WORKSPACE_PATH):
        self.path = Path(path)
        self.lock = Lock()
        self.workspace = None
        self.revision = 0
        self.error = "Review and save all robot workspace bounds before enabling control"
        try:
            self.workspace = RobotWorkspace.parse(json.loads(self.path.read_text(encoding="utf-8")))
            self.error = ""
        except FileNotFoundError:
            pass
        except (OSError, ValueError, TypeError, OverflowError) as exc:
            self.error = f"Robot workspace unavailable: {exc}"

    def snapshot(self):
        with self.lock:
            return self.workspace, self.revision, self.error

    def telemetry(self):
        workspace, revision, error = self.snapshot()
        return {"workspace": workspace.payload() if workspace else None, "revision": revision, "error": error}

    def save(self, data):
        workspace = RobotWorkspace.parse(data)
        temporary = None
        with self.lock:
            try:
                with NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.path.parent,
                                        prefix=self.path.name + ".", suffix=".tmp", delete=False) as output:
                    temporary = Path(output.name)
                    json.dump(workspace.payload(), output, indent=2, allow_nan=False)
                    output.write("\n")
                    output.flush()
                    os.fsync(output.fileno())
                temporary.replace(self.path)
                self.workspace, self.error = workspace, ""
                self.revision += 1
            finally:
                if temporary is not None:
                    temporary.unlink(missing_ok=True)
        return self.telemetry()
