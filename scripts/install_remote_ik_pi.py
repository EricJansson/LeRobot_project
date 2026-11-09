"""Reinstall isolated Pi teleop services over SSH. Starts neither service."""

import argparse
import subprocess

from scripts.remote_ik_deploy import bundle as bundle, ensure_installed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi", default="eric@rpi")
    args = parser.parse_args()
    try:
        ensure_installed(args.pi, force=True)
    except ValueError as exc:
        parser.error(str(exc))
    except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
        parser.exit(1, f"Pi installation failed: {exc}\n")
    print(f"Installed on {args.pi}. Run python.exe -m scripts.main_teleop_remote_ik --pi-sim to test.")


if __name__ == "__main__":
    main()
