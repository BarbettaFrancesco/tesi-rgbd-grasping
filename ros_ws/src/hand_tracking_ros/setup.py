from setuptools import find_packages, setup

package_name = 'hand_tracking_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['hand_tracking_ros/launch/htr_realsense_launch.py']),
        ('share/' + package_name, ['models/hand_landmarker.task']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francesco Barbe',
    maintainer_email='root@todo.todo',
    description='ROS 2 node for RGB hand tracking using MediaPipe.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hand_tracker = hand_tracking_ros.hand_tracker:main',
            'hand_landmark_3d = hand_tracking_ros.hand_landmark_3d:main',
            'hand_landmark_3d_tasks = hand_tracking_ros.HandLandMartk:main',
            'realsense_node = hand_tracking_ros.realsense_node:main',
        ],
    },
)
