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
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mover = robot_assignment.mover:main',
            'image_projection_1 = robot_assignment.image_projection_1:main',
            'image_projection_2 = robot_assignment.image_projection_2:main',
            'detector_basic = robot_assignment.detector_basic:main',
            'detector_3d = robot_assignment.detector_3d:main',
            'counter_3d = robot_assignment.counter_3d:main',
            'tf_listener = robot_assignment.tf_listener:main',            
            'demo_inspection = robot_assignment.demo_inspection:main',
            'camera_classifier_node = robot_assignment.camera_classifier:main',
            'web_server_node = robot_assignment.web_server:main',
        ],
    },
)
