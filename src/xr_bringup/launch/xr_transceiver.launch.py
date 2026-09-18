import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

from lifecycle_msgs.msg import Transition


def generate_launch_description():
    ld = LaunchDescription()

    xr_transceiver_node = LifecycleNode(
        package="xr_transceiver",
        executable="xr_transceiver",
        name="xr_transceiver",
        namespace="",
        parameters=[],
        output="screen",
        emulate_tty=True,
    )

    # ------------------------------------------------------------------
    # Lifecycle manager — stays outside the container
    # ------------------------------------------------------------------
    lifecycle_nodes = [
        "xr_transceiver",
    ]

    teleop_lifecycle_manager = Node(
        package="xr_teleop",
        executable="teleop_lifecycle_manager",
        name="teleop_lifecycle_manager",
        namespace="xr_transceiver",
        parameters=[
            {"managed_nodes": lifecycle_nodes},
            {"auto_start": True},
        ],
        output="screen",
        emulate_tty=True,
    )

    ld.add_action(xr_transceiver_node)
    ld.add_action(teleop_lifecycle_manager)

    return ld