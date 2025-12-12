# Video Script (2–5 minutes)

Use this as your voice-over while screen-recording.

## 0:00–0:15 Hook
Hi, this is my TurtleBot3 ROS2 automation assignment.
I built a modular automation system that includes setup automation, maintenance health checks,
autonomous navigation with SLAM using Nav2, AI-based object detection with YOLOv8 published to ROS2 topics,
and one custom feature: a QR-code follower.

## 0:15–0:45 Show entrypoint
Everything is controlled through one script: `tb3_auto.py`.
It provides separate commands for setup, maintenance, navigation, object detection, and custom features.

(Show terminal)
```bash
python3 tb3_auto.py --help
```

## 0:45–1:15 Setup automation proof
For setup automation, the project includes scripts to install ROS2 Humble and TurtleBot3 dependencies,
plus a Python virtual environment for AI perception.

(Show `scripts/` and mention they automate installation.)

## 1:15–1:45 Navigation automation
I’m using the TurtleBot3 Gazebo simulation and Nav2 with SLAM so the robot can build a map while navigating.

Terminal 1:
```bash
python3 tb3_auto.py navigation --sim --model burger
```

Terminal 2:
```bash
python3 tb3_auto.py navigation --nav2-slam
```

(Show Gazebo + RViz mapping.)

## 1:45–2:15 Maintenance automation
For maintenance, I created a health monitor node that checks battery and diagnostics and logs warnings.

Terminal 3:
```bash
python3 tb3_auto.py maintenance --monitor
```

(Show `logs/tb3_auto.log`.)

## 2:15–2:50 Object detection integration
For AI object detection, I integrated YOLOv8.
It publishes `/detections` and an annotated image stream `/yolo/annotated`.

Terminal 4:
```bash
source .venv/bin/activate
python3 tb3_auto.py detect --image /camera/image_raw --model yolov8n.pt
```

Verify:
```bash
ros2 topic echo /detections --once
```

(Show annotated stream in RViz/ImageView.)

## 2:50–3:30 Custom feature
My custom feature is a QR-code follower.
It detects a QR code and commands `/cmd_vel` to rotate until centered, then move forward.

Terminal 5:
```bash
python3 tb3_auto.py extra --qr-follow --image /camera/image_raw --cmd-vel /cmd_vel
```

## 3:30–4:00 Wrap-up
To summarize:
- setup automation
- maintenance monitoring
- navigation with SLAM
- object detection integrated into ROS2
- custom QR follower feature

All modules and commands are documented in the README.
