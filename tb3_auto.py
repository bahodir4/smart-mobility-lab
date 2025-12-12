#!/usr/bin/env python3
import argparse
from pathlib import Path

from modules.common import ensure_ubuntu, log
from modules.config import load_config
from modules import setup as m_setup
from modules import maintenance as m_maint
from modules import navigation as m_nav
from modules import perception as m_perc
from modules import extra_qr as m_qr

ROOT = Path(__file__).resolve().parent

def main():
    ensure_ubuntu()
    ap = argparse.ArgumentParser(description="TurtleBot3 Automation (ROS2) - modular assignment project")
    ap.add_argument("--config", default=None, help="Path to config.yaml (optional)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("setup", help="Install ROS2/TB3 deps and create workspace")
    s.add_argument("--install-ros2", action="store_true")
    s.add_argument("--install-tb3", action="store_true")
    s.add_argument("--ws", default=str(Path.home() / "tb3_ws"))
    s.add_argument("--bashrc", action="store_true", help="Append ROS + TURTLEBOT3_MODEL exports to ~/.bashrc")

    m = sub.add_parser("maintenance", help="Health checks / monitoring")
    m.add_argument("--doctor", action="store_true")
    m.add_argument("--check-topics", action="store_true")
    m.add_argument("--monitor", action="store_true")

    n = sub.add_parser("navigation", help="Simulation / bringup / SLAM+Nav2")
    n.add_argument("--model", default=None, help="burger | waffle | waffle_pi")
    n.add_argument("--sim", action="store_true", help="Launch TB3 Gazebo simulation")
    n.add_argument("--rviz", action="store_true", help="Launch RViz2 only")
    n.add_argument("--bringup", action="store_true", help="Launch robot bringup (real robot)")
    n.add_argument("--nav2-slam", action="store_true", help="Launch Nav2 with SLAM enabled")
    n.add_argument("--save-map", default=None, help="Save map to maps/<name> (requires nav2_map_server)")

    p = sub.add_parser("detect", help="YOLO detection node")
    p.add_argument("--image", default=None, help="Camera topic (default from config)")
    p.add_argument("--model", default=None, help="YOLO model path (default from config)")

    e = sub.add_parser("extra", help="Custom feature")
    e.add_argument("--qr-follow", action="store_true")
    e.add_argument("--image", default=None)
    e.add_argument("--cmd-vel", default=None)
    e.add_argument("--target", default=None)

    args = ap.parse_args()
    cfg = load_config(args.config)

    ros_distro = cfg.get("ros_distro", "humble")
    tb3_model = args.model if getattr(args, "model", None) else cfg.get("tb3_model", "burger")

    if args.cmd == "setup":
        if args.install_ros2:
            m_setup.install_ros2_humble_debs()
        if args.install_tb3:
            m_setup.install_tb3_deps_humble()
        m_setup.create_workspace(Path(args.ws))
        if args.bashrc:
            m_setup.add_bashrc_lines(ros_distro, tb3_model)
            log("Appended to ~/.bashrc (if missing). Open a new terminal afterwards.")

    elif args.cmd == "maintenance":
        if args.doctor:
            m_maint.ros2_doctor_report()
        if args.check_topics:
            m_maint.check_required_topics()
        if args.monitor:
            m_maint.start_health_monitor(ROOT)

    elif args.cmd == "navigation":
        m_nav.set_tb3_model(tb3_model)
        if args.sim:
            m_nav.launch_sim_gazebo()
        if args.rviz:
            m_nav.launch_rviz_only()
        if args.bringup:
            m_nav.bringup_robot()
        if args.nav2_slam:
            m_nav.nav2_with_slam_toolbox()
        if args.save_map:
            m_nav.save_map(args.save_map)

    elif args.cmd == "detect":
        image = args.image or cfg.get("camera_topic", "/camera/image_raw")
        model = args.model or cfg.get("yolo_model", "yolov8n.pt")
        m_perc.start_yolo(ROOT, image, model)

    elif args.cmd == "extra":
        if args.qr_follow:
            image = args.image or cfg.get("camera_topic", "/camera/image_raw")
            cmd_vel = args.cmd_vel or cfg.get("cmd_vel_topic", "/cmd_vel")
            target = args.target if args.target is not None else cfg.get("qr_target_text", "")
            m_qr.start_qr_follower(ROOT, image, cmd_vel, target)

if __name__ == "__main__":
    main()
