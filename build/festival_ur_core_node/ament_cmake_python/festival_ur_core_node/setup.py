from setuptools import find_packages
from setuptools import setup

setup(
    name='festival_ur_core_node',
    version='0.0.0',
    packages=find_packages(
        include=('festival_ur_core_node', 'festival_ur_core_node.*')),
)
