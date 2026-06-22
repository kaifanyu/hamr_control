import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hamr_control_exp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml') + glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hamr',
    maintainer_email='ranaudo@seas.upenn.edu',
    description='Experimental motion control stack (smooth trajectories, LQR, MPC, metrics)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traj_gen = hamr_control_exp.traj_gen_node:main',
            'lqr_controller = hamr_control_exp.lqr_controller_node:main',
            'mpc_controller = hamr_control_exp.mpc_controller_node:main',
            'traj_metrics = hamr_control_exp.metrics_node:main',
        ],
    },
)
