from pathlib import Path
from .common import run

def start_qr_follower(project_root: Path, image_topic: str, cmd_vel_topic: str, target_text: str = ""):
    cmd = f"python3 {project_root/'ros_nodes/qr_follower_node.py'} --image {image_topic} --cmd-vel {cmd_vel_topic}"
    if target_text:
        cmd += f" --target {target_text!r}"
    run(cmd, check=False)
