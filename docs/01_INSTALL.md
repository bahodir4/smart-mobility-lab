# Installation Guide (Ubuntu 22.04 + ROS 2 Humble)

## 1) System requirements
- Ubuntu 22.04 (recommended)
- ROS 2 Humble installed via apt (binary packages)
- Internet access for apt + pip installs

## 2) Clone repository
```bash
git clone <YOUR_GITHUB_REPO_URL>.git
cd tb3_automation
```

## 3) Install ROS 2 and TurtleBot3 dependencies
Option A (recommended): use included scripts:
```bash
./scripts/install_ros2_humble.sh
source /opt/ros/humble/setup.bash
./scripts/install_tb3_deps.sh
```

Option B: use the Python automation:
```bash
python3 tb3_auto.py setup --install-ros2 --install-tb3
```

## 4) Install Python dependencies (YOLO/OpenCV)
```bash
./scripts/bootstrap_python.sh
source .venv/bin/activate
```

## 5) Verify installation
```bash
python3 tb3_auto.py --help
ros2 --help
```
