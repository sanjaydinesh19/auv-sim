from setuptools import setup
import os
from glob import glob

package_name = 'pipeline_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanjay',
    maintainer_email='sanjay@omen.local',
    description='Autonomous underwater pipeline follower with ArUco detection.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipeline_follower_node = pipeline_follower.pipeline_follower_node:main',
        ],
    },
)
