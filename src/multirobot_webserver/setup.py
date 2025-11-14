from setuptools import setup
import os
from glob import glob

package_name = 'multirobot_webserver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install web templates
        (os.path.join('share', package_name, 'templates'), glob('templates/*.html')),
        # Install static files
        (os.path.join('share', package_name, 'static/css'), glob('static/css/*')),
        (os.path.join('share', package_name, 'static/js'), glob('static/js/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='svaghela',
    maintainer_email='svaghela@todo.todo',
    description='Multi-robot web interface for managing multiple Sphero instances',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multirobot_webapp = multirobot_webserver.multirobot_webapp:main',
        ],
    },
)
