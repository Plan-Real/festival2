# -*- coding: utf-8 -*-
from setuptools import setup

package_name = "festival_ur_wrapper"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="joenghan",
    maintainer_email="kimjh9813@naver.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "festival_ur_wrapper = \
                festival_ur_wrapper.wrapper:main"
        ],
    },
)
