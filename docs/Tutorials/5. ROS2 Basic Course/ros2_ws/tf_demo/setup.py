from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tf_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_broadcaster = tf_demo.static_tf_broadcaster:main',
            'tf_listener = tf_demo.tf_listener:main',
            'turtle_tf_broadcaster = tf_demo.turtle_tf_broadcaster:main',
            'turtle_following = tf_demo.turtle_following:main',

        ],
    },
)
