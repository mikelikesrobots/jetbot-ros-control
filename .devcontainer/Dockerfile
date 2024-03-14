FROM ros:humble-ros-base

RUN apt update && \
    apt upgrade -y && \
    apt install -y \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-xacro \
        ros-humble-teleop-twist-keyboard \
        gdb && \
    apt clean

ENTRYPOINT /ros_entrypoint.sh
