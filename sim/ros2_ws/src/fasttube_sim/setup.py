from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fasttube_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('../../worlds/*.world')),
        (os.path.join('share', package_name, 'models'),
            glob('../../models/**/*', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FastTube Team',
    maintainer_email='user@example.com',
    description='Livox HAP TX bridge for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'livox_bridge = fasttube_sim.livox_bridge_node:main',
        ],
    },
)
