from setuptools import find_packages, setup

package_name = 'aruco_slam'

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
    maintainer='Siddharth',
    maintainer_email='siddharth.vaghela@tufts.edu',
    description='ArUco marker-based SLAM for soccer field robot tracking',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_slam_node = aruco_slam.aruco_slam_node:main',
            'marker_generator = aruco_slam.marker_generator:main'
        ],
    },
)
