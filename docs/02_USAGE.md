# Usage Guide

> Tip: Open **multiple terminals**. Always ensure ROS is sourced:
```bash
source /opt/ros/humble/setup.bash
```

## A) Setup
Create a workspace (optional for this assignment):
```bash
python3 tb3_auto.py setup --ws ~/tb3_ws
```

## B) Navigation (Simulation)
Terminal 1:
```bash
python3 tb3_auto.py navigation --sim --model burger
```

Terminal 2 (Nav2 + SLAM):
```bash
python3 tb3_auto.py navigation --nav2-slam
```

Save a map:
```bash
python3 tb3_auto.py navigation --save-map my_map
```

## C) Maintenance
Health monitor:
```bash
python3 tb3_auto.py maintenance --monitor
```

Quick checks:
```bash
python3 tb3_auto.py maintenance --doctor
python3 tb3_auto.py maintenance --check-topics
```

## D) Object Detection (YOLOv8)
Terminal:
```bash
source .venv/bin/activate
python3 tb3_auto.py detect --image /camera/image_raw --model yolov8n.pt
```

Verify topics:
```bash
ros2 topic list | grep -E "detections|yolo"
ros2 topic echo /detections --once
```

## E) Custom Feature: QR Follower
Terminal:
```bash
python3 tb3_auto.py extra --qr-follow --image /camera/image_raw --cmd-vel /cmd_vel
```

Optional: follow only a specific QR payload:
```bash
python3 tb3_auto.py extra --qr-follow --target "FOLLOW_ME"
```
