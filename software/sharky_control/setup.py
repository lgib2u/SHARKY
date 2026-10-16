from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'sharky_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    (os.path.join('share', 'sharky_control', 'templates'),
        glob('sharky_control/templates/*.html')),

    (os.path.join('share', 'sharky_control', 'static'),
        glob('sharky_control/static/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SHARKY contributors',
    maintainer_email='maintainer@example.com',
    description='ROS 2 motion, gait, camera, and dashboard package for SHARKY',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gait_controller = sharky_control.gait_controller:main',
            'motion_node = sharky_control.motion_node:main',
            'web_interface = sharky_control.web_interface:main',
            'edge_web_interface = sharky_control.edge_web_interface:main',
            'camera_node = sharky_control.camera_node:main',
        ],
    },
)
