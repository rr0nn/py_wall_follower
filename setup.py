from setuptools import find_packages, setup
from pathlib import Path
from glob import glob

package_name = 'py_wall_follower'

# Use Pathlib for directory construction
package_share_dir = Path('share') / package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        # ROS 2 package index entry
        (str(Path('share') / 'ament_index' / 'resource_index' / 'packages'),
         [f'resource/{package_name}']),

        # Package manifest
        (str(Path('share') / package_name), ['package.xml']),

        # Launch files
        (str(package_share_dir / 'launch'), glob('launch/*.py')),

        # Parameter and map directories
        (str(package_share_dir / 'param'), glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='zhanggaorong6@gmail.com',
    description='Wall following and navigation package for TurtleBot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wf1 = py_wall_follower.wall_follower_v1:main',
            'wf2 = py_wall_follower.wall_follower_v2:main',
            'see_marker = py_wall_follower.see_marker:main',
            'point_transformer = py_wall_follower.point_transformer:main',
        ],
    },
)