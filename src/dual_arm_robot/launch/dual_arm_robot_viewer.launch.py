import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from launch_param_builder import ParameterBuilder

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    ld = LaunchDescription()

    left_arm_sim = LaunchConfiguration("left_arm_sim")
    right_arm_sim = LaunchConfiguration("right_arm_sim")
    use_respawn = LaunchConfiguration("use_respawn")
    params_file = LaunchConfiguration("params_file")

    ld.add_action(
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            description="Whether to respawn if a node crashes",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory("dual_arm_robot"), "config", "manipulation_config.yaml"),
        description="",
        )
    )

    # Declare arguments (optional but useful)
    ld.add_action(
        DeclareLaunchArgument(
            "xacro_file",
            default_value="dual_arm_robot.urdf.xacro",
            description="Name of the xacro file inside the urdf/ folder"
        )
    )

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

    rviz_config = os.path.join(get_package_share_directory("dual_arm_robot"), "rviz", "view.rviz")

    # Package and xacro path
    pkg_name = "dual_arm_robot"
    pkg_share = FindPackageShare(pkg_name)

    xacro_file = LaunchConfiguration("xacro_file")
    urdf_xacro_path = PathJoinSubstitution([
        pkg_share,
        "urdf",
        xacro_file
    ])

    # Process the xacro file into robot_description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                get_package_share_directory("dual_arm_robot"),
                "urdf",
                xacro_file
            ]),
            " ",
            "left_arm_sim:=", left_arm_sim,
            " ",
            "right_arm_sim:=", right_arm_sim,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    srdf_file = os.path.join(get_package_share_directory("dual_arm_robot"), "config", "dual_arm.srdf")
    kinematics_file = os.path.join(get_package_share_directory("dual_arm_robot"), "config", "kinematics.yaml")
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{
            "robot_description": ParameterValue(robot_description_content, value_type=str)
        }],
    )
    ld.add_action(robot_state_publisher)

    # joint_state_publisher_gui = Node(
    # package="joint_state_publisher_gui",
    # executable="joint_state_publisher_gui",
    # name="joint_state_publisher_gui",
    # output="screen",
    # )
    # ld.add_action(joint_state_publisher_gui)

    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_arm_robot"),
        "config",
        "ros2_controllers.yaml",
    )

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
                    "--controller-manager", "/controller_manager"
                ]
            )
        )

    moveit_config = (
        MoveItConfigsBuilder("dual_arm_robot", package_name="dual_arm_robot")
        .robot_description(file_path="urdf/dual_arm_robot.urdf.xacro", mappings={"left_arm_sim": left_arm_sim, "right_arm_sim": right_arm_sim})
        .robot_description_semantic(file_path="config/dual_arm_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )
    ld.add_action(move_group_node)

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
            ]
        )
        ld.add_action(node)

    # servo_yaml = load_yaml("dual_arm_robot", "config/moveit_servo.yaml")
    # servo_params = {"moveit_servo": servo_yaml}

    # Path to your servo yaml configuration file
    # servo_yaml_path = os.path.join(get_package_share_directory("dual_arm_robot"), "config", "moveit_servo.yaml")
    
    # with open(servo_yaml_path, "r") as f:
    #     servo_params = yaml.safe_load(f)


    # left_servo_node = Node(
    #     package="moveit_servo",
    #     executable="servo_node_main",
    #     name="servo_node",
    #     namespace="left_arm",
    #     output="screen",
    #     parameters=[
    #         servo_params,
    #         moveit_config.to_dict(),
    #     ],
    # )
    # ld.add_action(left_servo_node)

    # servo_params = {
    #     "moveit_servo": ParameterBuilder("dual_arm_robot")
    #     .yaml("config/pose_tracking_settings.yaml")
    #     .yaml("config/moveit_servo_pose_tracking.yaml")
    #     .to_dict()
    # }
    # pose_tracking_node  = Node(
    #     package="moveit_servo",
    #     executable="servo_pose_tracking_demo",
    #     name="servo_pose_tracking",
    #     namespace="left_arm",
    #     output="screen",
    #     parameters=[
    #         servo_params,
    #         moveit_config,
    #     ],
    # )
    # ld.add_action(pose_tracking_node)

    # xr_teleop_node = Node(
    #     package="xr_teleop",                     # change if your package name is different
    #     executable="xr_teleop_node",
    #     name="xr_teleop_node",
    #     output="screen",
    #     parameters=[
    #         {
    #             "robot_description": robot_description_content,
    #             "robot_description_semantic": ParameterFile(srdf_file, allow_substs=True),
    #             "planning_group": "dual_arm",                
    #             "left_ee_link": "arm_end_effector_l",
    #             "right_ee_link": "arm_end_effector_r",
    #             "left_group": "left_arm",
    #             "right_group": "right_arm",
    #             "use_collision_check": LaunchConfiguration("use_collision_check"),
    #             "ik_timeout": 0.05,
    #             "publish_rate": 50.0,
    #         }
    #     ],
    # )

    # RViz (optional)
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
        arguments=["-d", rviz_config],   # uncomment when you have a config
    )
    ld.add_action(rviz_node)

    return ld