from pathlib import Path
from .common import run

def start_yolo(project_root: Path, image_topic: str, model_path: str):
    run(f"python3 {project_root/'ros_nodes/yolo_detector_node.py'} "
        f"--image {image_topic} --model {model_path}", check=False)
