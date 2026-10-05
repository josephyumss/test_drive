from glob import glob
from setuptools import find_packages, setup

package_name = "amr_perception"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "range_fusion_node = amr_perception.range_fusion_node:main",
            "camera_path_node = amr_perception.camera_path_node:main",
            "path_guidance_node = amr_perception.path_guidance_node:main",
        ]
    },
)
