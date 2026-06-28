from glob import glob

from setuptools import find_packages, setup

package_name = 'kinect_field_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Siddharth',
    maintainer_email='siddharth.vaghela@tufts.edu',
    description='Kinect v1 overhead field calibration, registration, and '
                'Kalman-fused multi-Sphero field tracking.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'field_tracker_node = kinect_field_tracking.field_tracker_node:main',
        ],
    },
)
