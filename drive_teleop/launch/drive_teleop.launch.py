from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('drive_teleop'),
        'config',
        'drive_teleop.yaml'
    ])

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),
        Node(
            package='drive_teleop',
            executable='joy_to_twist',
            name='joy_to_twist',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='drive_teleop',
            executable='cmd_mux',
            name='cmd_mux',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='drive_teleop',
            executable='serial_sender',
            name='serial_sender',
            output='screen',
            parameters=[params],
        ),
    ])
