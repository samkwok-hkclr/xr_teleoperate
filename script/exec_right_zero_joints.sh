#!/bin/bash

ros2 service call /right_arm/execute_joints robot_controller_msgs/srv/ExecuteJoints "{ \
    "joint_names": ["arm_J1_r", "arm_J2_r", "arm_J3_r", "arm_J4_r", "arm_J5_r", "arm_J6_r", "arm_J7_r"], \
    "joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], \
    "async_execute": true \
}"