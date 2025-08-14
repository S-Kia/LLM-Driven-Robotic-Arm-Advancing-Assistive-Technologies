#!/bin/bash
# object_retrieve_coords.sh — Move FR3 arm to given XYZ + quaternion (qx qy qz qw)

# Check arguments
if [ "$#" -ne 7 ]; then
    echo "Usage: $0 x y z qx qy qz qw"
    echo "Example: $0 0.40 -0.40 0.20 0.0 0.0 0.0 1.0"
    exit 1
fi

X="$1"
Y="$2"
Z="$3"
QX="$4"
QY="$5"
QZ="$6"
QW="$7"

# Source ROS 2 environment (and your workspace if needed)
source /opt/ros/humble/setup.bash
# source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Send goal to MoveGroup (position + orientation constraints)
ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{
  request: {
    group_name: 'arm',
    goal_constraints: [{
      position_constraints: [{
        header: { frame_id: 'base' },
        link_name: 'fr3v2_hand',
        constraint_region: {
          primitives: [{
            type: 1,
            dimensions: [0.01, 0.01, 0.01]
          }],
          primitive_poses: [{
            position: { x: ${X}, y: ${Y}, z: ${Z} },
            orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
          }]
        },
        weight: 1.0
      }],
      orientation_constraints: [{
        header: { frame_id: 'base' },
        orientation: { x: ${QX}, y: ${QY}, z: ${QZ}, w: ${QW} },
        link_name: 'fr3v2_hand',
        absolute_x_axis_tolerance: 0.1,
        absolute_y_axis_tolerance: 0.1,
        absolute_z_axis_tolerance: 0.1,
        weight: 1.0
      }]
    }]
  }
}"

