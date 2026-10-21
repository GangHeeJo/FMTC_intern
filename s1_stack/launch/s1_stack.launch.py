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
            # 10/21: 예전엔 설정 없이 기본값(stop_line_mode 등)으로 떠서, front_1019.yaml로 따로 다시 띄우고
            # launch 쪽은 pkill로 꺼야 했음(둘 다 /cmd_auto 발행). 지금 카메라 정면 장착 기준 설정을 바로 줌
            parameters=[PathJoinSubstitution([
                FindPackageShare('s1_stack'), 'config', 'front_1019.yaml'
            ])],
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
                # 640x480은 DDS 전달 중 프레임이 많이 빠짐. lane_trace가 640x480으로 늘려서 처리함
                'image_width': 320,
                'image_height': 240,
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
