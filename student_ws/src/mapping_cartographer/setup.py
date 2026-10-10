import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mapping_cartographer'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ldaowen',
    maintainer_email='ldaowen@gmail.com',
    description='Student Cartographer mapping launch and baseline parameters for RB3302',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_time_normalizer = mapping_cartographer.scan_time_normalizer:main',
        ],
    },
)
