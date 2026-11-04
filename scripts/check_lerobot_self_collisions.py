"""Inspect SO-101 URDF mesh overlaps at a pose without connecting to the arm."""

import argparse
import math

from robot_program.lerobot_ik_controller import ARM_JOINTS, DEFAULT_URDF


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--arm-deg",
        nargs=len(ARM_JOINTS),
        type=float,
        default=(0.0,) * len(ARM_JOINTS),
        metavar="DEG",
        help=f"Arm angles in this order: {', '.join(ARM_JOINTS)} (default: all zero)",
    )
    parser.add_argument(
        "--jaw-deg", type=float, default=0.0,
        help="Jaw angle in URDF degrees (default: zero)",
    )
    args = parser.parse_args()
    joint_angles = dict(zip((*ARM_JOINTS, "gripper"), (*args.arm_deg, args.jaw_deg)))
    if not all(math.isfinite(angle) for angle in joint_angles.values()):
        parser.error("Joint angles must be finite.")

    try:
        import placo

        robot = placo.RobotWrapper(str(DEFAULT_URDF))
        neutral = {(hit.bodyA, hit.bodyB) for hit in robot.self_collisions(False)}
        for joint, angle_deg in joint_angles.items():
            angle_rad = math.radians(angle_deg)
            lower, upper = robot.get_joint_limits(joint)
            if not lower <= angle_rad <= upper:
                parser.error(
                    f"{joint} must be between {math.degrees(lower):.6f} and "
                    f"{math.degrees(upper):.6f} degrees in the URDF."
                )
            robot.set_joint(joint, angle_rad)
        robot.update_kinematics()
        contacts = {(hit.bodyA, hit.bodyB) for hit in robot.self_collisions(False)}
    except (ImportError, RuntimeError, ValueError) as exc:
        parser.exit(1, f"Collision check: {exc}\n")

    print(f"Pose in URDF degrees ({', '.join(f'{joint}={angle:g}' for joint, angle in joint_angles.items())})")
    print(f"Model mesh overlaps at this pose ({len(contacts)})")
    for first, second in sorted(contacts):
        label = "also at neutral" if (first, second) in neutral else "additional at this pose"
        print(f"  {first} / {second} ({label})")


if __name__ == "__main__":
    main()
