from launch import LaunchDescription

from launch_ros.actions import Node, LifecycleNode

def generate_launch_description():
    ld = LaunchDescription()

    xr_transceiver_node = LifecycleNode(
        package="xr_transceiver",
        executable="xr_transceiver",
        name="xr_transceiver",
        namespace="",
        parameters=[
            {'server_ip': "10.30.4.142"}, # FIXME
        ],
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
        namespace="",
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