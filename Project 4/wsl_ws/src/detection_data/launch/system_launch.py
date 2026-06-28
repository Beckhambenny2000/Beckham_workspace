from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments (override from CLI if needed) ────────────────────────
    tcp_host_arg = DeclareLaunchArgument(
        "tcp_host",
        default_value="127.0.0.1",
        description="TCP host for ZED detection server",
    )
    tcp_port_arg = DeclareLaunchArgument(
        "tcp_port",
        default_value="5005",
        description="TCP port for ZED detection server",
    )
    reconnect_delay_arg = DeclareLaunchArgument(
        "reconnect_delay",
        default_value="3.0",
        description="Seconds to wait before reconnecting on TCP failure",
    )

    # ── ZED detection subscriber node ─────────────────────────────────────────
    detection_node = Node(
        package="detection_data",
        executable="detectiondata",
        name="zed_detection_subscriber",
        output="screen",
        parameters=[
            {
                "tcp_host":        LaunchConfiguration("tcp_host"),
                "tcp_port":        LaunchConfiguration("tcp_port"),
                "reconnect_delay": LaunchConfiguration("reconnect_delay"),
            }
        ],
    )

    # ── Manual handler node (keyboard S/M/L publisher) ────────────────────────
    # NOTE: Needs its own terminal for keyboard input to work.
    #       Requires xterm to be installed: sudo apt install xterm
    manual_handler_node = Node(
        package="detection_data",
        executable="manual_handler",
        name="zed_detection_publisher",
        output="screen",
        prefix="xterm -e",  # Opens in a separate terminal window
    )

    return LaunchDescription([
        tcp_host_arg,
        tcp_port_arg,
        reconnect_delay_arg,
        detection_node,
        manual_handler_node,
    ])
