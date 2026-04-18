"""Launch file for rosaria2 node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
        DeclareLaunchArgument('sonar_frame', default_value='sonar',
                              description='Sonar frame ID'),
        DeclareLaunchArgument('cmd_vel_timeout', default_value='0.6',
                              description='Timeout (s) before stopping if no cmd_vel received'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz2 for visualization'),

        Node(
            package='rosaria2',
            executable='rosaria2_node',
            name='rosaria2',
            # namespace='ROSaria2',
            output='screen',
            respawn=True,
            respawn_delay=0.5,
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'publish_aria_lasers': LaunchConfiguration('publish_aria_lasers'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_link_frame': LaunchConfiguration('base_link_frame'),
                'cmd_vel_timeout': LaunchConfiguration('cmd_vel_timeout'),
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('rosaria2'), 'rviz', 'rosaria2.rviz'
            ])],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),

        # Add static transform for sonar so RViz can locate it relative to base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_sonar',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.2',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', LaunchConfiguration('base_link_frame'),
                '--child-frame-id', LaunchConfiguration('sonar_frame')
            ]
        ),
    ])
