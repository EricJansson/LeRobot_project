# Historical browser IK experiment

This standalone HTML/CSS/JavaScript visualization was used early in the project to explore custom forward and inverse kinematics. It is **outdated** and retained for reference.

Open index.html in a browser to inspect the old experiment. It does not import the current Python model, use LeRobot's URDF solver, communicate with the physical arm, or validate current motion/collision behavior. Its geometry and conventions should not be treated as authoritative for new implementations.

Use the [project README](../README.md) and [legacy architecture reference](../readme_files/ARCHITECTURE.md) for the working controls, including the existing standalone Pi URDF IK mode. The next target is the separate [PC-led architecture](../readme_files/PC_PI_ARCHITECTURAL_REFACTOR.md), developed on Windows with provisional WSL2 IK/collision checks and Pi motor execution. This browser experiment is not that runtime or its collision validator.
