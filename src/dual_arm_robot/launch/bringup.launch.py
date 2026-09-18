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


def _build_conditional_nodes(context, *args, **kwargs):
    """
    Everything that depends on the runtime value of `all_in_one` lives here.

    `LaunchConfiguration` is a *substitution* — it is only resolved once the
    launch system has a context.  Wrapping this code in an OpaqueFunction lets
    us call `.perform(context)` to get the actual string and branch in plain
    Python.
    """
    # --- resolve the launch arg to a real Python bool -----------------
    all_in_one_str = LaunchConfiguration("all_in_one").perform(context)
    all_in_one = all_in_one_str.strip().lower() in ("1", "true", "yes", "on")

    # --- 6b. XR transceiver (lifecycle node, only when all_in_one) ----
    xr_transceiver_node = LifecycleNode(
        package="xr_transceiver",
        executable="xr_transceiver",
        name="xr_transceiver",
        namespace="",
        parameters=[],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("all_in_one")),
    )

    # --- lifecycle node list ------------------------------------------
    lifecycle_nodes = [
        "/left_arm/servo_pose_tracking",
        "/right_arm/servo_pose_tracking",
        "xr_teleop",
    ]
    if all_in_one:
        lifecycle_nodes.append("xr_transceiver")

    # --- lifecycle manager --------------------------------------------
    teleop_lifecycle_manager = Node(
        package="xr_teleop",
        executable="teleop_lifecycle_manager",
        name="teleop_lifecycle_manager",
        parameters=[
            {"managed_nodes": lifecycle_nodes},
            {"auto_start": True},
        ],
        output="screen",
        emulate_tty=True,
    )

    return [xr_transceiver_node, teleop_lifecycle_manager]


def generate_launch_description():
    ld = LaunchDescription()

    # ------------------------------------------------------------------
    # Launch Configurations
    # ------------------------------------------------------------------
    left_arm_sim = LaunchConfiguration("left_arm_sim")
    right_arm_sim = LaunchConfiguration("right_arm_sim")
    use_respawn = LaunchConfiguration("use_respawn")
    use_rviz = LaunchConfiguration("use_rviz")
    params_file = LaunchConfiguration("params_file")
    auto_configure = LaunchConfiguration("auto_configure")
    auto_activate = LaunchConfiguration("auto_activate")
    all_in_one = LaunchConfiguration("all_in_one")

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
            "auto_configure",
            default_value="true",
            description="Automatically configure lifecycle nodes",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "auto_activate",
            default_value="true",
            description="Automatically activate lifecycle nodes",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "all_in_one",
            default_value="false",
            description="Also launch xr_transceiver and manage it alongside teleop nodes",
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
                "left_arm_sim": left_arm_sim,
                "right_arm_sim": right_arm_sim,
            },
        )
        .robot_description_semantic(file_path="config/dual_arm_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
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
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
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

    # ------------------------------------------------------------------
    # 6. Servo & Teleop Lifecycle Nodes
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Composable lifecycle nodes (inside the container)
    # ------------------------------------------------------------------
    left_pose_tracking = ComposableNode(
        package="xr_teleop",
        plugin="pose_tracker::PoseTrackerWrapper",
        name="servo_pose_tracking",
        namespace="left_arm",
        parameters=[
            {"side": "left"},
            servo_left_params,
            moveit_config.to_dict(),
        ],
        # extra_arguments=[{"use_intra_process_comms": True}],
    )

    right_pose_tracking = ComposableNode(
        package="xr_teleop",
        plugin="pose_tracker::PoseTrackerWrapper",
        name="servo_pose_tracking",
        namespace="right_arm",
        parameters=[
            {"side": "right"},
            servo_right_params,
            moveit_config.to_dict(),
        ],
        # extra_arguments=[{"use_intra_process_comms": True}],
    )

    xr_teleop_node = ComposableNode(
        package="xr_teleop",
        plugin="xr_teleop::XrTeleop",
        name="xr_teleop",
        namespace="",
        parameters=[
            {"x_scale_factor": 1.0},
            {"y_scale_factor": 1.0},
            {"z_scale_factor": 1.0},
            {"target_pose_max_age_s": 0.1},
            {"tcp": ["tcp_l", "tcp_r"]},
            {"k_sources": ["left_hand_frame", "right_hand_frame"]},
            {
                "xr_reference_data": [
                    "arm_base_link_l", "xr_base_link",
                    "0.07", "0.0", "0.16", "0.0", "0.0", "0.0",
                ]
            },
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    # ------------------------------------------------------------------
    # Container hosting the composable lifecycle nodes
    # ------------------------------------------------------------------
    container = ComposableNodeContainer(
        name="teleop_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # multi-threaded executor
        composable_node_descriptions=[
            left_pose_tracking,
            right_pose_tracking,
            xr_teleop_node,
        ],
        output="screen",
        emulate_tty=True,
    )
    ld.add_action(container)

    # ------------------------------------------------------------------
    # 7. Conditional nodes (xr_transceiver + lifecycle manager)
    #
    #  These depend on `all_in_one`, so they must be built inside an
    #  OpaqueFunction where the launch context is available.
    # ------------------------------------------------------------------
    ld.add_action(OpaqueFunction(function=_build_conditional_nodes))

    return ld