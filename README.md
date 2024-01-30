# JetBot ROS Control

ROS2 Node for controlling a JetBot via ROS Control interfaces.

## Setup

Install ROS2 Humble using VSCode Remote Containers extension. Simply open this folder using the extension and allow it to build the Docker container.

Then build the code using `colcon build` from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build
```

## Running the code

Once built, it should be possible to run the node as a standard ROS2 Node. Note that this will only run on a JetBot with the i2c-1 bus available.

```bash
source install/setup.bash
ros2 run jetbot_control jetbot_control
```

This should cause the JetBot to spin its motors forward for half a second, then stop for half a second.
