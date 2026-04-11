"""Launch file for rosaria2 node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/serial/by-id/usb-1a86_USB2.0-Ser_-if00-port0',
                              description='Serial port or hostname:port for robot connection'),
        DeclareLaunchArgument('baud', default_value='0',
                              description='Serial baud rate (0 = default)'),
        DeclareLaunchArgument('publish_aria_lasers', default_value='false',
                              description='Connect to and publish ARIA laser data'),
        DeclareLaunchArgument('odom_frame', default_value='odom',
                              description='Odometry frame ID'),
        DeclareLaunchArgument('base_link_frame', default_value='base_link',
                              description='Base link frame ID'),
        DeclareLaunchArgument('cmd_vel_timeout', default_value='0.6',
                              description='Timeout (s) before stopping if no cmd_vel received'),

        Node(
            package='rosaria2',
            executable='rosaria2_node',
            name='rosaria2',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'publish_aria_lasers': LaunchConfiguration('publish_aria_lasers'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_link_frame': LaunchConfiguration('base_link_frame'),
                'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
            }],
        ),
    ])
