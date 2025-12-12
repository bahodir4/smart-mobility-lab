from pathlib import Path
from .common import run, log

def install_ros2_humble_debs():
    # Based on ROS 2 Humble deb install docs.
    run("apt update", sudo=True)
    run("apt install -y software-properties-common curl", sudo=True)
    run("add-apt-repository -y universe", sudo=True)

    run("curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key "
        "-o /usr/share/keyrings/ros-archive-keyring.gpg", sudo=True)

    run('echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] '
        'http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" '
        '| tee /etc/apt/sources.list.d/ros2.list > /dev/null', sudo=True)

    run("apt update", sudo=True)
    run("apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep", sudo=True)
    run("rosdep init || true", sudo=True, check=False)
    run("rosdep update", sudo=False)

def install_tb3_deps_humble():
    # Matches TB3 Quick Start Guide section "Install Dependent ROS 2 Packages"
    run("apt update", sudo=True)
    run("apt install -y ros-humble-gazebo-*", sudo=True)
    run("apt install -y ros-humble-cartographer ros-humble-cartographer-ros", sudo=True)
    run("apt install -y ros-humble-navigation2 ros-humble-nav2-bringup", sudo=True)

    # helpful packages for perception
    run("apt install -y ros-humble-vision-msgs ros-humble-cv-bridge", sudo=True, check=False)

def create_workspace(ws: Path):
    (ws / "src").mkdir(parents=True, exist_ok=True)
    log(f"Workspace ready: {ws}")

def add_bashrc_lines(ros_distro: str, tb3_model: str):
    bashrc = Path.home() / ".bashrc"
    lines = bashrc.read_text(encoding="utf-8") if bashrc.exists() else ""
    additions = [
        f"source /opt/ros/{ros_distro}/setup.bash",
        f"export TURTLEBOT3_MODEL={tb3_model}",
    ]
    with open(bashrc, "a", encoding="utf-8") as f:
        for a in additions:
            if a not in lines:
                f.write("\n" + a + "\n")
