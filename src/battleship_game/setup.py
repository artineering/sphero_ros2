from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'battleship_game'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools', 'spherov2'],
    zip_safe=True,
    maintainer='Siddharth',
    maintainer_email='siddharth.vaghela@tufts.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'game_controller_node = battleship_game.game_controller_node:main',
            'human_controller_node = battleship_game.human_controller_node:main',
            'sphero_controller_node = battleship_game.sphero_controller_node:main',
            'test_publisher = battleship_game.test_publisher:main'
        ],
    },
)
