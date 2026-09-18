"""
All-in-one launch: brings up piper_cpp_ros (controller_manager + JTC) AND MoveIt's move_group
+ RViz with the MoveIt motion planning plugin.

Use this when you want a single command to go from a powered Piper to a MoveIt session.

If you already have piper_cpp_ros running (e.g. you started it from another launch), use
`move_group.launch.py` instead -- that one only spawns move_group + RViz and assumes
ros2_control is already up.

Pass `with_gripper:=true` to plan and execute on the parallel-jaw gripper as well as the
arm. This swaps in the gripper-enabled URDF, SRDF, and moveit_controllers, and spawns the
gripper_controller alongside the arm's joint_trajectory_controller.

Pass `use_real_hardware:=false` to swap PiperHardware for mock_components/GenericSystem.
The whole MoveIt + RViz stack still comes up; the joints just simulate position-following
with no CAN connection. Useful for planning-side development without a connected arm.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def _build_actions(context, *args, **kwargs):
    use_real_hardware = LaunchConfiguration("use_real_hardware").perform(context)
    can_interface = LaunchConfiguration("can_interface").perform(context)
    speed_pct = LaunchConfiguration("speed_pct").perform(context)
    go_to_zero_on_activate = LaunchConfiguration("go_to_zero_on_activate").perform(context)
    gripper_max_effort = LaunchConfiguration("gripper_max_effort").perform(context)
    home_gripper_on_activate = LaunchConfiguration("home_gripper_on_activate").perform(
        context
    )

    moveit_share = get_package_share_directory("piper_cpp_moveit_remote")
    dog_share = get_package_share_directory("xr_bringup")
    # piper_ros_share = get_package_share_directory("piper_cpp_ros")
    joint_limits_yaml = os.path.join(moveit_share, "config", "joint_limits.yaml")
    moveit_controllers_yaml = os.path.join(
        moveit_share, "config", "moveit_controllers.yaml"
    )

    urdf_file = "piper_arm.urdf.xacro"
    urdf_xacro = os.path.join(get_package_share_directory("xr_bringup"), "urdf", urdf_file)
    srdf_xacro = os.path.join(get_package_share_directory("xr_bringup"), "srdf", "piper_arm.srdf.xacro")
    urdf_mappings = {
        "use_real_hardware": use_real_hardware,
        "can_interface": can_interface,
        "speed_pct": speed_pct,
        "go_to_zero_on_activate": go_to_zero_on_activate,
    }

    moveit_config = (
        MoveItConfigsBuilder("piper", package_name="piper_cpp_moveit")
        .robot_description(file_path=urdf_xacro, mappings=urdf_mappings)
        .robot_description_semantic(file_path=srdf_xacro)
        .robot_description_kinematics(
            file_path=os.path.join(moveit_share, "config", "kinematics.yaml")
        )
        .joint_limits(file_path=joint_limits_yaml)
        .trajectory_execution(file_path=moveit_controllers_yaml)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Bring up piper_cpp_ros (controller_manager, robot_state_publisher, JSB, JTC, plus
    # gripper_controller when with_gripper is true).
    piper_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("xr_bringup"), "launch", "piper_control.launch.py"]
            )
        ),
        launch_arguments={
            "use_real_hardware": use_real_hardware,
            "can_interface": can_interface,
            "speed_pct": speed_pct,
            "description_package": "xr_bringup",
            "description_file": urdf_file,
            "go_to_zero_on_activate": go_to_zero_on_activate,
            "gripper_max_effort": gripper_max_effort,
            "home_gripper_on_activate": home_gripper_on_activate,
        }.items(),
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_config = os.path.join(dog_share, "rviz", "piper_moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            moveit_config.planning_scene_monitor,
        ],
    )

    return [piper_ros_launch, move_group_node, rviz_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_real_hardware",
                default_value="true",
                description="false runs ros2_control with mock_components/GenericSystem instead "
                "of the real CAN-driven PiperHardware. Useful for MoveIt/RViz development without "
                "a connected Piper.",
            ),
            DeclareLaunchArgument("can_interface", default_value="can0"),
            DeclareLaunchArgument("speed_pct", default_value="30"),
            DeclareLaunchArgument("go_to_zero_on_activate", default_value="false"),
            DeclareLaunchArgument(
                "with_gripper",
                default_value="false",
                description="Plan and execute on the parallel-jaw gripper as well as the arm.",
            ),
            DeclareLaunchArgument(
                "gripper_max_effort",
                default_value="1.0",
                description="Max gripper torque in N*m (range 0..5). Ignored unless with_gripper:=true.",
            ),
            DeclareLaunchArgument(
                "home_gripper_on_activate",
                default_value="false",
                description="Latch the current jaw position as gripper zero on activate. "
                "Only enable if you always activate from a known reference.",
            ),
            OpaqueFunction(function=_build_actions),
        ]
    )
