"""
Validation script to check the consistency of motor range and degree values
in the calibration file. Verifies that degree_min/degree_max correctly
correspond to range_min/range_max using the motor resolution formula.
"""

import json
from pathlib import Path


def load_calibration(filepath):
    """Load calibration data from JSON file."""
    with open(filepath, 'r') as f:
        return json.load(f)


def degrees_from_raw(raw_value, motor_resolution, center_position, total_degree_range):
    """Convert raw motor value to degrees using the calibration formula."""
    return ((raw_value - center_position) / motor_resolution) * total_degree_range


def validate_calibration(calibration_file):
    """
    Validate that degree values match the expected values computed from
    range values using the calibration formula.
    """
    data = load_calibration(calibration_file)
    
    motor_resolution = data['motor_resolution']
    total_degree_range = data['total_degree_range']
    center_position = data['center_position']
    
    print("=" * 80)
    print("CALIBRATION VALIDATION REPORT")
    print("=" * 80)
    print(f"Motor Resolution: {motor_resolution}")
    print(f"Total Degree Range: {total_degree_range}°")
    print(f"Center Position: {center_position}")
    print(f"Formula: degrees = ((raw_value - {center_position}) / {motor_resolution}) * {total_degree_range}")
    print("=" * 80)
    
    all_valid = True

    # Support both array format {"motors": [...]} and flat dict format
    if "motors" in data and isinstance(data["motors"], list):
        motors_iter = {m["name"]: m for m in data["motors"]}.items()
    else:
        motors_iter = {
            k: v for k, v in data.items()
            if k not in ['motor_resolution', 'total_degree_range', 'center_position', 'note']
        }.items()

    # Process each motor (skip metadata keys)
    for motor_name, motor_data in motors_iter:
        range_min = motor_data['range_min']
        range_max = motor_data['range_max']
        degree_min_stored = motor_data['degree_min']
        degree_max_stored = motor_data['degree_max']
        
        # Calculate expected degrees from raw values
        degree_min_calculated = degrees_from_raw(
            range_min, motor_resolution, center_position, total_degree_range
        )
        degree_max_calculated = degrees_from_raw(
            range_max, motor_resolution, center_position, total_degree_range
        )
        
        # Check if values match (with small tolerance for floating point)
        tolerance = 0.01
        min_valid = abs(degree_min_stored - degree_min_calculated) < tolerance
        max_valid = abs(degree_max_stored - degree_max_calculated) < tolerance
        
        motor_valid = min_valid and max_valid
        all_valid = all_valid and motor_valid
        
        status = "✓ VALID" if motor_valid else "✗ INVALID"
        
        print(f"\n{motor_name.upper()} {status}")
        print(f"  Range:     {range_min} → {range_max}")
        print(f"  Dégrees (stored):      {degree_min_stored:8.2f}° → {degree_max_stored:8.2f}°")
        print(f"  Degrees (calculated):  {degree_min_calculated:8.2f}° → {degree_max_calculated:8.2f}°")
        
        if not min_valid:
            diff = degree_min_stored - degree_min_calculated
            print(f"  ⚠ degree_min mismatch: {diff:+.2f}°")
        if not max_valid:
            diff = degree_max_stored - degree_max_calculated
            print(f"  ⚠ degree_max mismatch: {diff:+.2f}°")
    
    print("\n" + "=" * 80)
    if all_valid:
        print("✓ ALL CALIBRATION VALUES ARE CORRECT")
    else:
        print("✗ SOME CALIBRATION VALUES ARE INCORRECT - SEE ABOVE FOR DETAILS")
    print("=" * 80)
    
    return all_valid


if __name__ == "__main__":
    calibration_file = Path(__file__).parent / "calibration" / "lerobot_arm_with_degrees.json"
    
    if not calibration_file.exists():
        print(f"Error: Calibration file not found at {calibration_file}")
        exit(1)
    
    is_valid = validate_calibration(calibration_file)
    exit(0 if is_valid else 1)
