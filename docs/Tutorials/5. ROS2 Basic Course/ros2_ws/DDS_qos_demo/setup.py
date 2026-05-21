from setuptools import find_packages, setup

package_name = 'DDS_qos_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'DDS_qos_pub = DDS_qos_demo.DDS_qos_pub:main',  
            'DDS_qos_sub = DDS_qos_demo.DDS_qos_sub:main'  
        ],
    },
)
