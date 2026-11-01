#!/usr/bin/env python3
import os, sys, shutil, subprocess, glob

###                                                                                     ###
###     ---  Diagnostic / Environment validation tool for the robot workspace  ---      ###
###                                                                                     ###

def safe(cmd):
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
        return out.strip()
    except Exception as e:
        return f"(error: {e})"

print("=== Python / Env ===")
print("python exe :", sys.executable)
print("python vers:", sys.version.split()[0])
print("venv active:", sys.prefix != sys.base_prefix)
print("base Python:", sys.base_prefix)
print("VIRTUAL_ENV       :", os.environ.get("VIRTUAL_ENV", "(none; activation is optional)"))

print("\n=== lerobot ===")
try:
    import lerobot  # type: ignore
    print("lerobot __file__ :", getattr(lerobot, "__file__", "(unknown)"))
    # Get version via importlib.metadata if available
    try:
        from importlib.metadata import version, PackageNotFoundError
        print("lerobot version  :", version("lerobot"))
    except Exception:
        print("lerobot version  :", "(unknown)")
except Exception as e:
    print("lerobot import   : FAILED ->", e)

print("\n=== ffmpeg ===")
print("which ffmpeg :", shutil.which("ffmpeg") or "(not found)")
if shutil.which("ffmpeg"):
    print("ffmpeg version:", safe(["ffmpeg", "-hide_banner", "-version"]).splitlines()[0])

print("\n=== Serial devices (WSL) ===")
devs = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
print("found:", ", ".join(devs) if devs else "(none)")

print("\n=== sys.path[0:5] ===")
for p in sys.path[:5]:
    print(" -", p)
