from setuptools import find_packages, setup
import os, glob

package_name = 'py_wall_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['py_wall_follower', 'py_wall_follower.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='zhanggaorong6@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wf1 = py_wall_follower.wall_follower_v1:main',
            'see_marker = py_wall_follower.see_marker:main',
            'point_transformer = py_wall_follower.point_transformer:main',
        ],
    },
)
