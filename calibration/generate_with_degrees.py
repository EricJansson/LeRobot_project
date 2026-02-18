import json
from pathlib import Path

CALIB_DIR = Path(__file__).parent
INPUT  = CALIB_DIR / "lerobot_arm.json"
OUTPUT = CALIB_DIR / "lerobot_arm_with_degrees.json"

RESOLUTION  = 4096
TOTAL_DEG   = 360
CENTER      = 2048

def raw_to_deg(raw):
    return ((raw - CENTER) / RESOLUTION) * TOTAL_DEG

with open(INPUT) as f:
    data = json.load(f)

motors = []
for name, m in data.items():
    deg_min = round(raw_to_deg(m["range_min"]), 2)
    deg_max = round(raw_to_deg(m["range_max"]), 2)
    motors.append({
        "name":         name,
        "id":           m["id"],
        "drive_mode":   m["drive_mode"],
        "homing_offset":m["homing_offset"],
        "range_min":    m["range_min"],
        "range_max":    m["range_max"],
        "degree_min":   deg_min,
        "degree_max":   deg_max,
        "degree_range": round(deg_max - deg_min, 2),
    })

output = {
    "motor_resolution":  RESOLUTION,
    "total_degree_range": TOTAL_DEG,
    "center_position":   CENTER,
    "note": "degrees = ((raw_value - 2048) / 4096) * 360",
    "motors": motors,
}

with open(OUTPUT, "w") as f:
    json.dump(output, f, indent=4)

print(f"Written to {OUTPUT}")
for m in motors:
    print(f"  {m['name']:<15} {m['degree_min']:>8.2f}° → {m['degree_max']:>8.2f}°  (range: {m['degree_range']:.2f}°)")