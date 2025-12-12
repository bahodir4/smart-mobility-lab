from .common import run, log
from pathlib import Path
import subprocess

def ros2_doctor_report():
    # Not required by docs, but great for demos & troubleshooting.
    run("ros2 doctor --report", check=False)

def check_required_topics(required=None):
    required = required or ["/tf", "/scan", "/cmd_vel"]
    try:
        out = subprocess.check_output("ros2 topic list", shell=True, text=True)
        topics = set([t.strip() for t in out.splitlines() if t.strip()])
    except Exception as e:
        log(f"Could not list topics (is ROS running?): {e}")
        return False

    ok = True
    for t in required:
        if t not in topics:
            ok = False
            log(f"WARNING: missing topic: {t}")
        else:
            log(f"OK: topic present: {t}")
    return ok

def start_health_monitor(project_root: Path):
    run(f"python3 {project_root/'ros_nodes/health_monitor_node.py'}", check=False)
