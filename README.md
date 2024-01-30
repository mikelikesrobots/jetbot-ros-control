# JetBot ROS Control

ROS2 Node for controlling a JetBot via ROS Control interfaces.

## Setup

Install ROS2 Humble as per the instructions.

Clone this repository into a ROS2 workspace, e.g.:

```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
git clone https://github.com/mikelikesrobots/jetbot-ros-control
```

Then build the code using `colcon build` from the workspace root:

```bash
source /opt/ros/humble/setup.bash
cd ~/dev_ws
colcon build
```

## Running the code

Once built, it should be possible to run the node as a standard ROS2 Node. Note that this will only run on a JetBot with the i2c-1 bus available.

```bash
cd ~/dev_ws
source install/setup.bash
ros2 run jetbot_control jetbot_control
```

This should cause the JetBot to spin its motors forward for half a second, then stop for half a second.
