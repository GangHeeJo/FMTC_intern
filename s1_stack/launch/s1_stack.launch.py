from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('s1_stack'),
        'config',
        's1_stack.yaml'
    ])

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='s1_stack',
            executable='joy_to_twist',
            name='joy_to_twist',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='s1_stack',
            executable='cmd_mux',
            name='cmd_mux',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='s1_stack',
            executable='serial_sender',
            name='serial_sender',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='s1_stack',
            executable='decision_auto',
            name='decision_auto',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[params],
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_lane',
            namespace='cam_lane',
            output='screen',
            parameters=[{
                'video_device': '/dev/cam_lane',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
            }],
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_front',
            namespace='cam_front',
            output='screen',
            parameters=[{
                'video_device': '/dev/cam_front',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'mjpeg2rgb',
            }],
        ),
    ])
