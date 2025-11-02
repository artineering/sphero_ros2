from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sphero_web_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include templates
        (os.path.join('share', package_name, 'templates'),
            glob('sphero_web_interface/templates/*.html')),
        # Include static CSS files
        (os.path.join('share', package_name, 'static/css'),
            glob('sphero_web_interface/static/css/*.css')),
        # Include static JS files
        (os.path.join('share', package_name, 'static/js'),
            glob('sphero_web_interface/static/js/*.js')),
    ],
    install_requires=[
        'setuptools',
        'flask',
        'flask-socketio',
        'python-socketio',
    ],
    zip_safe=True,
    maintainer='Siddharth',
    maintainer_email='siddharth.vaghela@tufts.edu',
    description='Web-based interface for controlling and monitoring Sphero robots',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'web_server = sphero_web_interface.web_server_node:main',
        ],
    },
)
