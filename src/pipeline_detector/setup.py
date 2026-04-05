from setuptools import setup

package_name = 'pipeline_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanjay',
    maintainer_email = 'sanjaydinesh2006@gmail.com',
    description='Pipeline follower node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'follower_node = pipeline_detector.pipeline_follower:main',
        ],
    },
)
