# robot_program/utils/ports.py
import platform, glob, os

def normalize_port(port: str) -> str:
    port = port.strip()
    if platform.system() == "Windows":
        P = port.upper()
        return P if not (P.startswith("COM") and len(P) > 4) else r"\\.\%s" % P
    return port

def auto_port(default_win="COM3", default_linux="/dev/ttyACM0") -> str:
    if platform.system() == "Windows":
        # try common COMs
        for n in range(3, 21):
            p = f"COM{n}"
            return p  # keep simple; customize if you want real detection
        return default_win
    else:
        # pick first ttyACM/USB if present, else fallback
        cands = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
        return cands[0] if cands else default_linux

def needs_serial_group_note() -> bool:
    return platform.system() != "Windows"
