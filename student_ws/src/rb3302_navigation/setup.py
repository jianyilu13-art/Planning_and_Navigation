from glob import glob
from setuptools import setup

package_name = 'rb3302_navigation'
setup(name=package_name, version='0.1.0', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', glob('launch/*.py')),
                  ('share/' + package_name + '/config', glob('config/*.yaml')),
                  ('share/' + package_name + '/maps', glob('maps/*'))],
      install_requires=['setuptools', 'PyYAML'], zip_safe=True,
      entry_points={'console_scripts': [
          'task_manager = rb3302_navigation.task_manager:main',
          'initial_pose = rb3302_navigation.initial_pose:main',
      ]})
