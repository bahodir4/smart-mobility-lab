import os
import subprocess
from pathlib import Path
from datetime import datetime

ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT / "logs"
LOG_DIR.mkdir(exist_ok=True)

def log(msg: str):
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    line = f"[{ts}] {msg}"
    print(line)
    with open(LOG_DIR / "tb3_auto.log", "a", encoding="utf-8") as f:
        f.write(line + "\n")

def run(cmd: str, sudo: bool = False, check: bool = True, env=None):
    if sudo:
        cmd = f"sudo bash -lc {cmd!r}"
    log(f"RUN: {cmd}")
    return subprocess.run(cmd, shell=True, check=check, env=env)

def ensure_ubuntu():
    if not Path("/etc/os-release").exists():
        raise RuntimeError("This project expects Ubuntu Linux with ROS 2 installed.")
