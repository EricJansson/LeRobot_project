# Joint Limits Implementation

## Overview
The `RobotArm` class has been updated to automatically enforce joint limits based on your motor specifications from the calibration JSON file. This prevents the inverse kinematics from generating invalid motor commands.

## What Changed

### 1. **Automatic Limit Loading** 
The `RobotArm` class now loads joint limits from `calibration/lerobot_arm_with_degrees.json` by default:

```python
arm = RobotArm(model=model)  # Automatically loads limits from calibration file
print(arm.joint_limits_deg)
# Output: [(-68.09, 71.89), (-70.84, 58.89), (-67.27, 69.84)]
```

### 2. **Motor-to-Joint Mapping**
The first 4 motors in your calibration file are mapped to the 3-DOF planar arm:

| Motor ID | Motor Name | Mapped Joint | Degree Limits |
|----------|------------|--------------|---------------|
| 1 | shoulder_pan | base_yaw_deg | (handled separately) |
| 2 | shoulder_lift | theta1_deg | -68.09° to 71.89° |
| 3 | elbow_flex | theta2_deg | -70.84° to 58.89° |
| 4 | wrist_flex | theta3_deg | -67.27° to 69.84° |

### 3. **Limit Enforcement in IK**
Joint limits are now enforced in the `_solve_ik_for_phi()` method:
- IK solutions are filtered using `select_ik_solution_deg()` 
- Any solution violating the limits is rejected
- If no valid solution exists within the limits, the method returns `None`

## Usage

### Default Behavior (Recommended)
```python
from robot_program.RobotArm import RobotArm, ArmModel

model = ArmModel()
arm = RobotArm(model=model)  # Limits auto-load from calibration file

# Attempt to move - will only succeed if within joint limits
ok = arm.move_end_effector(
    target_xyz=(18.0, 5.0, 12.0),
    phi_deg=0.0
)
```

### Manual Limit Override
```python
# Custom limits if needed
custom_limits = [(-30.0, 30.0), (-40.0, 40.0), (-35.0, 35.0)]
arm = RobotArm(
    model=model,
    joint_limits_deg=custom_limits,
    auto_load_limits=False
)
```

### Disable Automatic Loading
```python
# No limits enforced (dangerous!)
arm = RobotArm(
    model=model,
    auto_load_limits=False
)
```

### Custom Calibration Path
```python
from pathlib import Path

custom_calib_path = Path("path/to/custom/calibration.json")
arm = RobotArm(
    model=model,
    calibration_path=custom_calib_path
)
```

## Implementation Details

### New Function: `load_joint_limits_from_calibration()`
```python
def load_joint_limits_from_calibration(calibration_path=None) -> List[Tuple[float, float]]:
    """
    Load joint limits from the calibration JSON file.
    
    Returns:
        List of (min_deg, max_deg) tuples for theta1, theta2, theta3
    """
```

### Updated `RobotArm.__init__()`
```python
def __init__(
    self,
    model: ArmModel,
    joint_limits_deg: Optional[List[Tuple[float, float]]] = None,
    calibration_path: Optional[Path] = None,
    auto_load_limits: bool = True
):
```

**Parameters:**
- `joint_limits_deg`: Manual limits (overrides auto-loading if provided)
- `calibration_path`: Custom path to calibration JSON
- `auto_load_limits`: If True, loads from calibration file automatically

**Error Handling:**
- If auto-load fails, prints a warning and continues without limits
- Allows graceful fallback if calibration file is missing

## Current Motor Constraints

From your calibration file, here are the actual constraints now enforced:

```
theta1 (shoulder_lift): -68.09° to 71.89°  (range: 139.98°)
theta2 (elbow_flex):    -70.84° to 58.89°  (range: 129.73°)
theta3 (wrist_flex):    -67.27° to 69.84°  (range: 137.11°)
```

These are quite restrictive and may reduce the workspace. If a target position is unreachable with these limits, `move_end_effector()` will return `False`.

## Testing

Run the joint limits test to verify the implementation:
```bash
python test_joint_limits.py
```

This tests:
- ✅ Limits load from calibration file
- ✅ Auto-loading works by default
- ✅ Manual overrides work
- ✅ Solutions respect joint limits

## Files Modified

- **robot_program/RobotArm.py**
  - Added `load_joint_limits_from_calibration()` function
  - Updated `RobotArm.__init__()` to support auto-loading
  - Simplified `_solve_ik_for_phi()` to use `select_ik_solution_deg()` for filtering

- **test_joint_limits.py** (NEW)
  - Test suite for joint limits functionality
