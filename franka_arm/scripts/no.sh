#!/usr/bin/env bash

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
'{
  "trajectory": {
    "joint_names": ["fr3v2_joint1","fr3v2_joint2","fr3v2_joint3","fr3v2_joint4","fr3v2_joint5","fr3v2_joint6","fr3v2_joint7"],
    "points": [
      { "positions": [0.0, -0.5, 0.0, -1.5, 0.0, 1.5, 0.785], "time_from_start": { "sec": 2, "nanosec": 0 } },
      { "positions": [0.4, -0.5, 0.0, -1.5, 0.0, 1.5, 0.785], "time_from_start": { "sec": 3, "nanosec": 0 } },
      { "positions": [-0.4, -0.5, 0.0, -1.5, 0.0, 1.5, 0.785], "time_from_start": { "sec": 4, "nanosec": 0 } },
      { "positions": [0.4, -0.5, 0.0, -1.5, 0.0, 1.5, 0.785], "time_from_start": { "sec": 5, "nanosec": 0 } },
      { "positions": [0.0, -0.5, 0.0, -1.5, 0.0, 1.5, 0.785], "time_from_start": { "sec": 6, "nanosec": 0 } },
      { "positions": [0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.000], "time_from_start": { "sec": 8, "nanosec": 0 } }
    ]
  }
}'
