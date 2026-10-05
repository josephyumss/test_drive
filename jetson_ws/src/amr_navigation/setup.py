from glob import glob
from setuptools import find_packages, setup

package_name = "amr_navigation"
setup(
    name=package_name, version="0.1.0", packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=["setuptools", "PyYAML"], zip_safe=True,
    entry_points={"console_scripts": [
        "wheel_odometry_node = amr_navigation.wheel_odometry:main",
        "waypoint_mission_node = amr_navigation.waypoint_mission:main",
    ]},
)
