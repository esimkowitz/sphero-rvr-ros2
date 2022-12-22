from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    rvr_server_node = Node(
        package="rvr_server",
        executable="sphero_node"
    )

    robot_control = Node(
        package="robot_control",
        executable="portal_app"
    )

    ld.add_action(rvr_server_node)
    ld.add_action(robot_control)
    return ld