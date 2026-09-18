from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'agent_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
		(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dav',
    maintainer_email='d.borghini3@studenti.unipi.it',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
			'task_handler = agent_server.task_handler:main',
			'config_node = agent_server.config_node:main',
            'dummy_camera = agent_server.dummy_camera:main',
            'manager_node = agent_server.manager_node:main',
        ],
    },
)
