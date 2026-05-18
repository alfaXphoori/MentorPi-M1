from setuptools import setup
import os
from glob import glob

package_name = 'mycar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='My Car ROS 2 Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drive_node = mycar.drive_node:main',
            'square_node = mycar.square_move:main',
            'hand_control_node = mycar.hand_control:main',
            'color_control_node = mycar.color_control:main',
            'lane_detect_node = mycar.lane_detect_node:main',
            'mission_control_node = mycar.mission_control:main',
            'camera_tilt_node = mycar.camera_tilt_node:main',
        ],
    },
)
