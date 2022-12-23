from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    rvr_node = Node(
        package="rvr_node",
        executable="rvr_node"
    )

    ld.add_action(rvr_node)
    return ld