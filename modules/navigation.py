import os
from .common import run, log

def set_tb3_model(model: str):
    os.environ["TURTLEBOT3_MODEL"] = model
    log(f"TURTLEBOT3_MODEL={model}")

def launch_sim_gazebo():
    # TB3 official simulation page describes Gazebo simulation on ROS2 Humble.
    run("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py", check=False)

def launch_rviz_only():
    run("ros2 launch turtlebot3_bringup rviz2.launch.py", check=False)

def bringup_robot():
    run("ros2 launch turtlebot3_bringup robot.launch.py", check=False)

def nav2_with_slam_toolbox():
    # Works well in many setups; users can replace with Cartographer if desired.
    run("ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True slam:=True", check=False)

def save_map(map_name="tb3_map"):
    run(f"ros2 run nav2_map_server map_saver_cli -f maps/{map_name}", check=False)
