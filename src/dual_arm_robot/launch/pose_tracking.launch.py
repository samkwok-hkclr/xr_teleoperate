import os
import yaml

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import LifecycleNode
from launch_ros.descriptions import ComposableNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

from lifecycle_msgs.msg import Transition

def generate_launch_description():
    ld = LaunchDescription()

    left_arm_sim = LaunchConfiguration("left_arm_sim")
    right_arm_sim = LaunchConfiguration("right_arm_sim")

    ld.add_action(
        DeclareLaunchArgument(
            "left_arm_sim",
            default_value="true",
            description="Use simulation for left arm"
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "right_arm_sim",
            default_value="true",
            description="Use simulation for right arm"
        )
    )

    ld.add_action(
        DeclareLaunchArgument('auto_configure', default_value='true'),
    )

    ld.add_action(
        DeclareLaunchArgument('auto_activate', default_value='true'),
    )

    moveit_config = (
        MoveItConfigsBuilder("dual_arm_robot", package_name="dual_arm_robot")
        .robot_description(file_path="urdf/dual_arm_robot.urdf.xacro", mappings={"left_arm_sim": left_arm_sim, "right_arm_sim": right_arm_sim})
        .robot_description_semantic(file_path="config/dual_arm_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_dict()
    )

    servo_left_params = {
        "moveit_servo": ParameterBuilder("dual_arm_robot")
        .yaml("config/pose_tracking_settings.yaml")
        .yaml("config/moveit_servo_pose_tracking_left.yaml")
        .to_dict()
    }
    servo_right_params = {
        "moveit_servo": ParameterBuilder("dual_arm_robot")
        .yaml("config/pose_tracking_settings.yaml")
        .yaml("config/moveit_servo_pose_tracking_right.yaml")
        .to_dict()
    }

    # Define the Composable Nodes
    left_pose_tracking_node = LifecycleNode(
        package="xr_teleop",
        executable="pose_tracker",
        name="servo_pose_tracking",
        namespace="left_arm",
        parameters=[
            {"side": "left"},
            servo_left_params,
            moveit_config,
        ],
    )

    right_pose_tracking_node = LifecycleNode(
        package="xr_teleop",
        executable="pose_tracker",
        name="servo_pose_tracking",
        namespace="right_arm",
        parameters=[
            {"side": "right"},
            servo_right_params,
            moveit_config,
        ],
    )

    xr_teleop_node = LifecycleNode(
        package="xr_teleop",
        executable="xr_teleop",
        name="xr_teleop",
        namespace=TextSubstitution(text=''),
        parameters=[
            {"x_scale_factor": 1.2},
            {"y_scale_factor": 1.2},
            {"z_scale_factor": 1.1},
        ],
    )

    xr_transceiver_node = LifecycleNode(
        package='xr_transceiver',
        executable='xr_transceiver',
        name='xr_transceiver',
        namespace=TextSubstitution(text=''),
        parameters=[],
        output='screen',  
        emulate_tty=True,
    )
    
    nodes = [left_pose_tracking_node, right_pose_tracking_node, xr_teleop_node, xr_transceiver_node]
    for node in nodes:
        configure_event_handler = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node,
                on_start=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(node),
                            transition_id=Transition.TRANSITION_CONFIGURE,
                        ),
                    ),
                ],
            ),
            condition=IfCondition(LaunchConfiguration('auto_configure')),
        )

        activate_event_handler = RegisterEventHandler(
            event_handler=OnStateTransition(
                target_lifecycle_node=node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        ),
                    ),
                ],
            ),
            condition=IfCondition(LaunchConfiguration('auto_activate')),
        )
        
        ld.add_action(node)
        ld.add_action(configure_event_handler)
        ld.add_action(activate_event_handler)

    return ld