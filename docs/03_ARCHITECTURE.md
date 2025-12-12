# Architecture

## Entry Script
- `tb3_auto.py`
  - Provides CLI commands:
    - `setup`
    - `maintenance`
    - `navigation`
    - `detect`
    - `extra`

## Modules (Python)
- `modules/setup.py`
  - Installs ROS2 Humble + TurtleBot3 dependencies
  - Creates workspace
- `modules/maintenance.py`
  - Topic checks
  - Launches health monitor node
- `modules/navigation.py`
  - Launches Gazebo simulation
  - Launches Nav2 with SLAM enabled
  - Saves maps using `map_saver_cli`
- `modules/perception.py`
  - Launches YOLO detector node
- `modules/extra_qr.py`
  - Launches QR follower node

## ROS Nodes (Python)
- `ros_nodes/health_monitor_node.py`
  - Subscribes to `/battery_state`, `/diagnostics`
  - Logs warnings/errors
- `ros_nodes/yolo_detector_node.py`
  - Subscribes to camera image topic
  - Runs YOLOv8 inference
  - Publishes:
    - `/detections` (vision_msgs/Detection2DArray)
    - `/yolo/annotated` (annotated image)
- `ros_nodes/qr_follower_node.py`
  - Detects QR via OpenCV QRCodeDetector
  - Publishes Twist to `/cmd_vel`

## Configuration
- `config.yaml`
  - ROS distro, model, default topics, YOLO model path, QR follower params
