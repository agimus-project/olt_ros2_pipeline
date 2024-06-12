import os
from glob import glob
from setuptools import find_packages, setup

package_name = "olt_ros2_pipeline"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/composed_tracker"],
        ),
        (
            "share/ament_index/resource_index/packages",
            ["resource/keyboard_monitor"],
        ),
        (
            "share/ament_index/resource_index/packages",
            ["resource/detection_filter"],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Guilhem Saurel",
    maintainer_email="guilhem.saurel@laas.fr",
    description="Examples for m3t_tracker_ros package",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "composed_tracker = olt_ros2_pipeline.composed_tracker:main",
            "keyboard_monitor = olt_ros2_pipeline.keyboard_monitor:main",
            "detection_filter = olt_ros2_pipeline.detection_filter:main",
        ],
    },
)
