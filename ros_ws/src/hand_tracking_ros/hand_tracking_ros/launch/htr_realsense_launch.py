from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#424x240x30
def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ),
        launch_arguments={
            'depth_module.depth_profile': '424,240,30',
            'rgb_camera.color_profile': '424,240,30',
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
        }.items(),
    )

    hand_tracker_node = Node(
        package='hand_tracking_ros',
        executable='hand_tracker',
        name='hand_tracker',
        output='screen',
        parameters=[
            {'color_topic': '/camera/camera/color/image_raw'}
        ],
    )

    hand_landmark_3d_node = Node(
        package='hand_tracking_ros',
        executable='hand_landmark_3d',
        name='hand_landmark_3d',
        output='screen',
    )

    return LaunchDescription([
        realsense_launch,
        hand_tracker_node,
        hand_landmark_3d_node,
    ])
