from setuptools import setup
import os
from glob import glob

package_name = 'sphero_worker_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install systemd unit template
        (os.path.join('share', package_name, 'systemd'), glob('systemd/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Siddharth Vaghela',
    maintainer_email='siddharth.vaghela@tufts.edu',
    description='Per-Pi HTTP launcher agent that spawns and tears down Sphero '
                'instance trees on a remote BLE worker node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent = sphero_worker_agent.agent:main',
        ],
    },
)
