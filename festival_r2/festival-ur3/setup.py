from setuptools import setup

package_name = "festival_ur3"

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
    maintainer="jeonghan",
    maintainer_email="kimjh9813@naver.com",
    description="ur3 festival waypoint node",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "waypoint = festival_ur3.waypoint_test:main",
            "control = festival_ur3.joint_control:main",
        ],
    },
)
