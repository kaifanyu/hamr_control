"""Install the HAMR reference-trajectory ROS package."""

from glob import glob

from setuptools import find_packages, setup

package_name = 'reference_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            'share/' + package_name + '/config/trajectories',
            glob('config/trajectories/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cedric',
    maintainer_email='cedrich@seas.upenn.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_traj_sequence = '
            'reference_trajectory.waypoint_traj_sequence:main',
            'waypoint_traj_simple = '
            'reference_trajectory.waypoint_traj_simple:main',
            'study_trajectory = '
            'reference_trajectory.study_trajectory:main',
        ],
    },
)
