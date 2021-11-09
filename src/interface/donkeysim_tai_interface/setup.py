from setuptools import setup
import os
from glob import glob

package_name = 'donkeysim_tai_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Haoru Xue',
    maintainer_email='hxue@ucsd.edu',
    description='Interface between Donkeysim and TritonAIRacer',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'donkeysim_client_node = donkeysim_tai_interface.donkeysim_client_node:main'
        ],
    },
)
