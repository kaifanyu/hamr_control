from setuptools import find_packages
from setuptools import setup

setup(
    name='hamr_interfaces',
    version='0.1.0',
    packages=find_packages(
        include=('hamr_interfaces', 'hamr_interfaces.*')),
)
