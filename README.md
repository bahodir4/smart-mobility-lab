# TurtleBot3 Automation (ROS 2) — Assignment Project

This repo is a complete, **modular automation script** for TurtleBot3 using **ROS 2**:
- Setup automation (install ROS2 + TB3 deps + workspace)
- Maintenance automation (health monitor + checks)
- Navigation automation (Gazebo simulation + Nav2 with SLAM)
- Object detection integration (YOLOv8 + publish detections + annotated image)
- Custom feature: **QR-code follower** (OpenCV) that commands the robot via `cmd_vel`

> Target environment: **Ubuntu 22.04 + ROS 2 Humble**. ROS 2 Humble deb packages are available for Ubuntu Jammy (22.04). citeturn0search0

---

## 0) Folder Structure

```
tb3_automation/
  tb3_auto.py
  config.yaml
  requirements.txt
  modules/
  ros_nodes/
  scripts/
  logs/
  maps/
```

---

## 1) Setup (Installation + Workspace)

### A) Install ROS 2 Humble (binary packages)
You can run the automation:
```bash
python3 tb3_auto.py setup --install-ros2
```

Or run the script:
```bash
./scripts/install_ros2_humble.sh
```

This follows the official ROS 2 Humble Ubuntu (deb packages) instructions. citeturn0search0

### B) Install TurtleBot3 dependent packages
TurtleBot3 Quick Start recommends installing Gazebo, Cartographer, and Navigation2 packages: citeturn0search1
```bash
python3 tb3_auto.py setup --install-tb3
# or
./scripts/install_tb3_deps.sh
```

### C) Python dependencies (YOLO + OpenCV)
```bash
./scripts/bootstrap_python.sh
source .venv/bin/activate
```

### D) Workspace creation
```bash
python3 tb3_auto.py setup --ws ~/tb3_ws
```

(Optional) Append ROS + model variables to your `~/.bashrc`:
```bash
python3 tb3_auto.py setup --bashrc
```

---

## 2) Maintenance Automation

### A) ROS doctor report
```bash
python3 tb3_auto.py maintenance --doctor
```

### B) Topic presence checks
```bash
python3 tb3_auto.py maintenance --check-topics
```

### C) Health monitor node
Monitors `/battery_state` and `/diagnostics` and logs warnings. TurtleBot3 publishes battery information on `/battery_state`. citeturn0search1
```bash
python3 tb3_auto.py maintenance --monitor
```

Logs go into `logs/tb3_auto.log`.

---

## 3) Navigation Automation (Simulation + SLAM)

### A) Gazebo simulation (recommended for demo)
TurtleBot3 provides official simulation docs for Gazebo on ROS2 Humble. citeturn0search13
```bash
python3 tb3_auto.py navigation --sim --model burger
```

### B) Nav2 + SLAM
Nav2 has an official tutorial for “Navigating while Mapping (SLAM)”. citeturn0search2
```bash
python3 tb3_auto.py navigation --nav2-slam
```

### C) Save a map
```bash
python3 tb3_auto.py navigation --save-map my_map
# outputs: maps/my_map.yaml and maps/my_map.pgm
```

---

## 4) Object Detection (YOLOv8)

This project uses Ultralytics YOLOv8 and publishes:
- `/detections` (vision_msgs/Detection2DArray)
- `/yolo/annotated` (sensor_msgs/Image)

Ultralytics provides a ROS quickstart guide for subscribing to camera topics and publishing detection results. citeturn0search3

Run:
```bash
source .venv/bin/activate
python3 tb3_auto.py detect --image /camera/image_raw --model yolov8n.pt
```

In RViz/ImageView, visualize `/yolo/annotated`.

---

## 5) Custom Feature: QR Code Follower

This extra feature detects a QR code using OpenCV and commands the robot:
- If the QR center is left/right → rotate to center it
- If centered → move forward

Run:
```bash
python3 tb3_auto.py extra --qr-follow --image /camera/image_raw --cmd-vel /cmd_vel
```

(Optional) Follow only QR with specific payload:
```bash
python3 tb3_auto.py extra --qr-follow --target "FOLLOW_ME"
```
---

## 6) Troubleshooting

- If `/camera/image_raw` doesn’t exist, run:
  ```bash
  ros2 topic list | grep image
  ```
  then use that topic with `--image`.

- If `cv_bridge` missing:
  ```bash
  sudo apt install ros-humble-cv-bridge
  ```

- If Gazebo launch fails, ensure TB3 simulation packages are installed and ROS 2 Humble is sourced.

---

## License
Educational use for coursework.
