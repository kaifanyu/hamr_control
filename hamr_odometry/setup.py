from setuptools import find_packages, setup

package_name = 'hamr_odometry'

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
    maintainer='cedric',
    maintainer_email='cedrich@seas.upenn.edu',
    description='Holonomic wheel odometry for HAMR.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'holonomic_odom_node = hamr_odometry.holonomic_odom_node:main',
        ],
    },
)
