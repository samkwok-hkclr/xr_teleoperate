import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
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

    # ------------------------------------------------------------------
    # Launch Configurations
    # ------------------------------------------------------------------
    left_arm_sim = LaunchConfiguration("left_arm_sim")
    right_arm_sim = LaunchConfiguration("right_arm_sim")
    left_gripper_sim  = LaunchConfiguration("left_gripper_sim")
    right_gripper_sim = LaunchConfiguration("right_gripper_sim")
    use_respawn = LaunchConfiguration("use_respawn")
    use_rviz = LaunchConfiguration("use_rviz")
    params_file = LaunchConfiguration("params_file")
    use_telemanip  = LaunchConfiguration("use_telemanip")
    cell_id = LaunchConfiguration("cell_id")
    period_ms = LaunchConfiguration("period_ms")
    zenoh_config = LaunchConfiguration("zenoh_config")
    agent_wait_ms = LaunchConfiguration("agent_wait_ms")

    # ------------------------------------------------------------------
    # Declare Launch Arguments
    # ------------------------------------------------------------------
    ld.add_action(
        DeclareLaunchArgument(
            "left_arm_sim",
            default_value="true",
            description="Use simulation for left arm",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "right_arm_sim",
            default_value="true",
            description="Use simulation for right arm",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "left_gripper_sim",
            default_value="true",
            description="Use simulation for left gripper",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "right_gripper_sim",
            default_value="true",
            description="Use simulation for right gripper",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            description="Whether to respawn if a node crashes",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Whether to start RViz2",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("dual_arm_robot"),
                "config",
                "manipulation_config.yaml",
            ),
            description="Path to robot controller parameters file",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_telemanip",
            default_value="false",
            description="Use the new telemanip Zenoh hardware interface instead of the legacy UDP one"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "cell_id",
            default_value="cell-cuarm1",
            description="Telemanip cell id (must match the agent)"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "period_ms",
            default_value="20",
            description="Telemanip command period in ms (must match the agent)"
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "zenoh_config",
            default_value="",
            description="Absolute path to zenoh.json5. "
                        "Empty -> auto-derived from "
                        "share/dual_arm_hardware_interface/config/zenoh.json5",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "agent_wait_ms",
            default_value="10000",
            description="Max wait for the telemanip agent liveliness token (ms)",
        )
    )

    # ------------------------------------------------------------------
    # Paths and MoveIt Configurations
    # ------------------------------------------------------------------
    rviz_config = os.path.join(
        get_package_share_directory("dual_arm_robot"), "rviz", "view.rviz"
    )
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_arm_robot"),
        "config",
        "ros2_controllers.yaml",
    )

    moveit_config = (
        MoveItConfigsBuilder("dual_arm_robot", package_name="dual_arm_robot")
        .robot_description(
            file_path="urdf/dual_arm_robot.urdf.xacro",
            mappings={
                "left_arm_sim":   left_arm_sim,
                "right_arm_sim":  right_arm_sim,
                "left_gripper_sim": left_gripper_sim,
                "right_gripper_sim": right_gripper_sim,
                "config_prefix":  os.path.join(
                    get_package_share_directory("cuarm_configuration"),
                    "dual_v2_2",
                ),
                "use_telemanip":  use_telemanip,
                "cell_id":        cell_id,
                "period_ms":      period_ms,
                "zenoh_config":     zenoh_config,
                "agent_wait_ms":    agent_wait_ms,
            },
        )
        .robot_description_semantic(file_path="config/dual_arm_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ------------------------------------------------------------------
    # 1. Robot State Publisher
    # ------------------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )
    ld.add_action(robot_state_publisher)

    # ------------------------------------------------------------------
    # 2. MoveGroup Node
    # ------------------------------------------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    ld.add_action(move_group_node)

    # ------------------------------------------------------------------
    # 3. Robot Controllers (Left & Right Arms)
    # ------------------------------------------------------------------
    controllers = ["left_arm", "right_arm"]
    for controller in controllers:
        node = Node(
            package="robot_controller",
            executable="robot_controller_node",
            namespace=controller,
            parameters=[
                params_file,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
            respawn=use_respawn,
            respawn_delay=3.0,
            output="screen",
            remappings=[
                ("robot_description", "/robot_description"),
                ("robot_description_semantic", "/robot_description_semantic"),
                ("get_planning_scene", "/get_planning_scene"),
                ("apply_planning_scene", "/apply_planning_scene"),
                ("joint_states", "/joint_states"),
            ],
        )
        ld.add_action(node)

    # ------------------------------------------------------------------
    # 4. ROS2 Control Node & Spawners
    # ------------------------------------------------------------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
            moveit_config.robot_description,
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )
    ld.add_action(ros2_control_node)

    basic_controller_names = [
        "left_arm_controller",
        "right_arm_controller",
        "joint_state_broadcaster",
        "left_hand_controller",
        "right_gripper_controller",
    ]

    tracking_controller_names = [
        "left_cartesian_motion_controller",
        "right_cartesian_motion_controller",
    ]

    for controller in basic_controller_names:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                    "--controller-manager",
                    "/controller_manager",
                ],
            )
        )

    for controller in tracking_controller_names:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                    "--controller-manager",
                    "/controller_manager",
                    "--inactive"
                ],
            )
        )

    # ------------------------------------------------------------------
    # 5. RViz2 Node (Conditional)
    # ------------------------------------------------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )
    ld.add_action(rviz_node)

    return ld