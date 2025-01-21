from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="db_beta_interface",
            executable="position_controller",
            name="db_beta_interface",
            output="screen",
            emulate_tty=True,
            parameters=[
                {'baudrate': 57600,
                  'portname': "/dev/ttyUSB0",
                  'scanRange': 73}
            ]
        )
    ])