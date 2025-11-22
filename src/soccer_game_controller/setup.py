from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'soccer_game_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('lib', package_name), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Siddharth',
    maintainer_email='siddharth.vaghela@tufts.edu',
    description='Soccer game controller for multi-robot soccer with ArUco SLAM-based field calibration',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'soccer_game_controller = soccer_game_controller.soccer_game_controller_node:main',
        ],
    },
)
