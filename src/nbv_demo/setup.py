from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nbv_demo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NeRFly',
    maintainer_email='todo@todo.com',
    description='Autonomous NBV demo: circular UAV orbit + image capture',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run nbv_demo mission_node
            'mission_node      = nbv_demo.mission_node:main',
            # ros2 run nbv_demo image_saver_node
            'image_saver_node  = nbv_demo.image_saver_node:main',
            # ros2 run nbv_demo snapshot_node  (Phase 2 camera verification)
            'snapshot_node     = nbv_demo.snapshot_node:main',
        ],
    },
)
