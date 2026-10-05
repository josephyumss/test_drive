from glob import glob
from setuptools import find_packages, setup

package_name = "amr_vision"
setup(name=package_name, version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"]),
                  ("share/" + package_name + "/config", glob("config/*.yaml"))],
      install_requires=["setuptools"], zip_safe=True,
      entry_points={"console_scripts": [
          "yolo_node = amr_vision.yolo_node:main",
          "yolo_udp_bridge_node = amr_vision.yolo_udp_bridge_node:main",
      ]})
