#!/bin/bash

# Pose parameters
X=0.057 # 0.057
Y=0.234
Z=0.3

# RPY (radian) → Quaternion
# RPY: [3.14, 0.0, 1]  ≈ 180° around X-axis
# Resulting quaternion: (x=1.0, y=0.0, z=0.0, w=0.0)
QX=0.7071
QY=0.7071
QZ=0.0
QW=0.0

# Speed 
SPEED=1.0

# Call the service
ros2 service call /piper_arm/execute_pose robot_controller_msgs/srv/ExecutePose \
"{
  pose: {
    position: {x: $X, y: $Y, z: $Z},
    orientation: {x: $QX, y: $QY, z: $QZ, w: $QW}
  },
  speed: $SPEED
}"