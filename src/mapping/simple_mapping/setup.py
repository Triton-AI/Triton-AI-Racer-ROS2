from setuptools import setup

package_name = 'simple_mapping'

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
    maintainer='Haoru Xue',
    maintainer_email='hxue@ucsd.edu',
    description='simple 2D LiDAR mapping package',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_mapping_node = simple_mapping.simple_mapping_node:main'
        ],
    },
)
