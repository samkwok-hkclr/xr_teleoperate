import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def _build_conditional_nodes(context, *args, **kwargs):
    """
    Everything that depends on the runtime value of `all_in_one` lives here.

    `LaunchConfiguration` is a *substitution* — it is only resolved once the
    launch system has a context.  Wrapping this code in an OpaqueFunction lets
    us call `.perform(context)` to get the actual string and branch in plain
    Python.
    """
    # --- resolve the launch arg to a real Python bool -----------------
    auto_start_str = LaunchConfiguration("auto_start").perform(context)
    auto_start = auto_start_str.strip().lower() in ("1", "true", "yes", "on")
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
        "/left_arm/pose_tracking_wrapper",
        "/right_arm/pose_tracking_wrapper",
        "/xr_teleop",
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
            {"auto_start": auto_start},
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
    auto_start = LaunchConfiguration("auto_start")

    # ------------------------------------------------------------------
    # Declare Launch Arguments
    # ------------------------------------------------------------------
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
            "auto_start",
            default_value="true",
            description="Automatically start lifecycle nodes",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "all_in_one",
            default_value="false",
            description="Also launch xr_transceiver and manage it alongside teleop nodes",
        )
    )

    left_pose_tracking = ComposableNode(
        package="xr_teleop",
        plugin="xr_teleop::PoseTrackingWrapper",
        name="pose_tracking_wrapper",
        namespace="left_arm",
        parameters=[
            {"side": "left"},
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    right_pose_tracking = ComposableNode(
        package="xr_teleop",
        plugin="xr_teleop::PoseTrackingWrapper",
        name="pose_tracking_wrapper",
        namespace="right_arm",
        parameters=[
            {"side": "right"},
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
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