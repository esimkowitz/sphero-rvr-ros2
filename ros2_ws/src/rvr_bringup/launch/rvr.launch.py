from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    rvr_server_node = Node(
        package="rvr_server",
        executable="server"
    )

    ld.add_action(rvr_server_node)
    return ld