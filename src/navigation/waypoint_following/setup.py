from setuptools import setup
import os
from glob import glob

package_name = 'waypoint_following'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
    ],
    install_requires=['setuptools', 'simple-pid', 'numpy'],
    zip_safe=True,
    maintainer='Haoru Xue',
    maintainer_email='hxue@ucsd.edu',
    description='Run Waypoint Following',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'follow_wp = waypoint_following.wp_follow_node:main',
                'dynamic_follow_wp = waypoint_following.dynamic_wp_follow_node:main'
        ],
    },
)
