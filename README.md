# JetBot ROS Control

ROS2 Node for controlling a JetBot via ROS Control interfaces.

## Setup

From a WaveShare JetBot robot, clone this repository and open it over SSH. Install the Dev Containers extension and allow it to build the Docker container and open the workspace in that container.

Then build the code using `colcon build` from the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build
```

## Testing the code

Once built, you can run the test function to check that the library can be loaded correctly as follows:

```bash
source install/setup.bash
./build/jetbot_control/test_jetbot_system
```

The test should pass with log messages showing a successful initialization of the control library.

## Running the code

Once built, it should be possible to launch the jetbot controller using the provided launch file. Note that this will only run on a JetBot with the i2c-1 bus available.

```bash
source install/setup.bash
ros2 launch jetbot_control jetbot.launch.py
```

After this, the robot will respond to commands sent with `TwistStamped` type sent on the `/cmd_vel` topic.

## License

The code in this repository is covered by the MIT license in the [LICENSE](./LICENSE) file. However, four files are included from the [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) repository, and so are covered by the [ROS2_CONTROL_LICENSE](./ROS2_CONTROL_LICENSE) file instead. These four files are as follows:

1. [jetbot.launch.py](./bringup/launch/jetbot.launch.py)
2. [jetbot_system.cpp](./hardware/src/jetbot_system.cpp)
3. [jetbot_system.hpp](./hardware/include/jetbot_control/jetbot_system.hpp)
4. [visibility_control.h](./hardware/include/jetbot_control/visibility_control.h)
