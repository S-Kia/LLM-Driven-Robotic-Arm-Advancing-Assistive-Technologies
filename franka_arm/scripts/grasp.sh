#!/usr/bin/env bash

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

ros2 action send_goal /gripper_controller/gripper_cmd \
control_msgs/action/GripperCommand "{ command: { position: 0.0, max_effort: 60.0 } }"

