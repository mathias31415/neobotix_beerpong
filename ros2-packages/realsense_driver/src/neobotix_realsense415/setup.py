from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'neobotix_realsense415'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Include all launch files (same as Cmakes InstallDirectory)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

        # Include rviz config
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        # Include nodes executables
        (os.path.join('share', package_name, package_name), glob('neobotix_realsense415/*.py')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='docker_realsense',
    maintainer_email='docker_realsense@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],

    # Add all scripts and nodes to execute with ros2 run ...
    entry_points={
        'console_scripts': [
            'camera_subscriber_node = neobotix_realsense415.camera_subscriber_node:main',
            'save_image = neobotix_realsense415.save_image:main',
            'print_calibration_data = neobotix_realsense415.print_calibration_data:main',
            'calibration = neobotix_realsense415.calibration:main',
            'get_aruco_pose = neobotix_realsense415.get_aruco_pose:main',
            'print_detected_aruco_pose = neobotix_realsense415.print_detected_aruco_pose:main',
            'get_aruco_pose_server = neobotix_realsense415.get_aruco_pose_server:main',
            'capture_image_server = neobotix_realsense415.capture_image_server:main',
            'get_aruco_pose_client = neobotix_realsense415.get_aruco_pose_client:main',
            'capture_image_client = neobotix_realsense415.capture_image_client:main'
        ],
    },
)
