# Project Overview

This repository contains a **modular automation system** for TurtleBot3 using **ROS 2**.
It is designed to satisfy coursework requirements for an “automation script” that integrates:

- Setup automation (ROS 2 + TurtleBot3 dependencies)
- Maintenance automation (health checks, logging, alerts)
- Navigation automation (simulation + Nav2 + SLAM)
- AI perception (YOLOv8 object detection → ROS 2 topics)
- Custom feature (QR-code follower → cmd_vel)

The system is controlled via a single entry script:

- `tb3_auto.py` (CLI entrypoint)

Configuration is centralized in:

- `config.yaml`
