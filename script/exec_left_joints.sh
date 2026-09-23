#!/bin/bash

ros2 service call /left_arm/execute_joints robot_controller_msgs/srv/ExecuteJoints "{ \
    "joint_names": ["arm_J1_l", "arm_J2_l", "arm_J3_l", "arm_J4_l", "arm_J5_l", "arm_J6_l", "arm_J7_l"], \
    "position": [-1.15, 0.41, 0.0, 1.15, 0.0, 0.0, 0.0] \
}"