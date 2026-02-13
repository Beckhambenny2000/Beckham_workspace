import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

pkg = 'gv_ros2'

def generate_launch_description():
    ld = LaunchDescription()
    config_path = os.path.join(get_package_share_directory(pkg), 'config')

    node = Node(
        package = pkg,
        name = pkg + '_node',
        executable = pkg + '_node',
        output="screen",
        emulate_tty=True,
        parameters = [
            { 'config_path': config_path },
        ]
    )
    ld.add_action(node)
    return ld
