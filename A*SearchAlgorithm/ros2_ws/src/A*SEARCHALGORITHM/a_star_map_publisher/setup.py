import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'a_star_map_publisher'

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
    maintainer='rodrigo',
    maintainer_email='A01704287@itesm.mx',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_publisher = a_star_map_publisher.map_publisher:main',
            'a_star_node = a_star_map_publisher.a_star_node:main',
        ],
    },
)
