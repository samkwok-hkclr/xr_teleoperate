#! /bin/bash 
# Basic entrypoint Docker containers

# Source ROS 2
if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]
then
  source /opt/ros/${ROS_DISTRO}/setup.bash
fi

# Source the base workspace, if built
if [ -f /root/${WS_NAME}/install/setup.bash ]
then
  source /root/${WS_NAME}/install/setup.bash
fi

# Execute the command passed into this entrypoint
exec "$@"