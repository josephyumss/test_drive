from glob import glob
from setuptools import find_packages, setup

package_name = "amr_mcu_bridge"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    entry_points={"console_scripts": ["mcu_bridge_node = amr_mcu_bridge.mcu_bridge_node:main"]},
)
