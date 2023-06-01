from setuptools import setup

import os
from glob import glob
from urllib.request import urlretrieve
from setuptools import find_packages

package_name = 'yolo_people'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('./launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Angledsugar',
    maintainer_email='angledsugar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_people = yolo_people.yolo_people:main'
        ],
    },
)
