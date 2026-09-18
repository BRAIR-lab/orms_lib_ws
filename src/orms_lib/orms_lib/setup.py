from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'orms_lib'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Taskboard manipulation package using franka_ros2 and tum_tb_perception2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task_server_node = orms_lib.nodes.task_server_node:main',
            'sequencer_node = orms_lib.nodes.sequencer_node:main',
        ],
    },
)
