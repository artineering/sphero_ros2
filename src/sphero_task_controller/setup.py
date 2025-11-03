from setuptools import find_packages, setup

package_name = 'sphero_task_controller'

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
    maintainer='Siddharth Vaghela',
    maintainer_email='siddharth.vaghela@tufts.edu',
    description='High-level task controller for Sphero robots. Accepts tasks from a dedicated topic and executes them using sphero_controller_node.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_controller = sphero_task_controller.task_controller_node:main',
        ],
    },
)
