from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_assignment'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anyeh Ndi-Tah',
    maintainer_email='27455742@students.lincoln.ac.uk',
    description='ROS2 Package for the Robot Programming - CMP9767 course at the University of Lincoln, UK: color and shape classification, storing results in SQLite, and hosting a web server.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_classifier = robot_assignment.camera_classifier:main',
            'color_3d_detection = robot_assignment.color_3d_detection:main',
            'counter_3d = robot_assignment.counter_3d:main',
            'demo_inspection = robot_assignment.demo_inspection:main',
            'web_server = robot_assignment.web_server:main',
        ],
    },
     extras_require={
        'test': ['pytest'],
    },
)
