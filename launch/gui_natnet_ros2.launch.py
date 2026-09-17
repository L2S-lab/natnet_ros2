from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    marker_poses_server = Node(
        package="natnet_ros2",
        executable="marker_poses_server",
        output="screen",
        emulate_tty=True,
    )

    helper_node = Node(
        package="natnet_ros2",
        executable="helper_node_r2.py",
        output="screen",
        emulate_tty=True,
    )

    ld.add_action(marker_poses_server)
    ld.add_action(helper_node)

    return ld