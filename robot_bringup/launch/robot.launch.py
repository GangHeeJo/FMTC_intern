from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('robot_bringup'),
        'config',
        'robot.yaml'
    ])

    return LaunchDescription([
        # joystick driver
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # manual teleop to Twist
        Node(
            package='drive_teleop',
            executable='joy_to_twist',
            name='joy_to_twist',
            output='screen',
            parameters=[params],
        ),

        # mode mux
        Node(
            package='drive_teleop',
            executable='cmd_mux',
            name='cmd_mux',
            output='screen',
            parameters=[params],
        ),

        # final serial output to Arduino
        Node(
            package='drive_teleop',
            executable='serial_sender',
            name='serial_sender',
            output='screen',
            parameters=[params],
        ),

        # lane -> /cmd_auto
        Node(
            package='perception',
            executable='cam_lane_auto',
            name='cam_lane_auto',
            output='screen',
            parameters=[params],
        ),

        # Camera for lane (Logitech C920 #2)
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
                'pixel_format': 'mjpeg2rgb',  # C920에서 CPU 부담 줄이기(대개 잘 됨)
            }],
        ),

        # Camera for front (Logitech C920 #1, YOLO용)
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

        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
        

    ])
