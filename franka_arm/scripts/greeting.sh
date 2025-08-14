#!/usr/bin/env bash

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Send the FollowJointTrajectory goal
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
'{
  trajectory: {
    joint_names: ["fr3v2_joint1","fr3v2_joint2","fr3v2_joint3","fr3v2_joint4","fr3v2_joint5","fr3v2_joint6","fr3v2_joint7"],
    points: [
      { positions: [0.1, 0.2, 0.0, -1.5, 0.0, 1.8, 0.785], time_from_start: { sec: 2, nanosec: 0 } },
      { positions: [0.1, 0.2, 0.0, -1.5, 0.0, 1.5, 0.785], time_from_start: { sec: 3, nanosec: 0 } },
      { positions: [0.1, 0.2, 0.0, -1.5, 0.0, 2.1, 0.785], time_from_start: { sec: 4, nanosec: 0 } },
      { positions: [0.1, 0.2, 0.0, -1.5, 0.0, 1.5, 0.785], time_from_start: { sec: 5, nanosec: 0 } },
      { positions: [0.1, 0.2, 0.0, -1.5, 0.0, 1.8, 0.785], time_from_start: { sec: 6, nanosec: 0 } },
      { positions: [0.0, -0.40, 0.0, -1.90, 0.0, 1.57, 0.8], time_from_start: { sec: 8, nanosec: 0 } }
    ]
  }
}'


