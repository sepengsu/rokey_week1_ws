#!/usr/bin/env python3
from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'week1'

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
    maintainer='jaenote',
    maintainer_email='na06219@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kitchen_gui = week1.kitchen_gui:main',
            'order_gui = week1.order_gui:main',
            'turtlebot3_gui = week1.turtlebot3_gui:main',
        ],
    },
)
