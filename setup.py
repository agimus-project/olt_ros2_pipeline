import os
from glob import glob
from setuptools import find_packages, setup

from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = "olt_ros2_pipeline"

module_name = "detection_pose_filter_parameters"
yaml_file = "olt_ros2_pipeline/detection_pose_filter_parameters.yaml"
generate_parameter_module(module_name, yaml_file)


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
            ["resource/happypose_labeler"],
        ),
        (
            "share/ament_index/resource_index/packages",
            ["resource/detection_pose_filter"],
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
    description="ROS 2 reimplementation of Object Localization and Tracking vision pipeline.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "composed_tracker = olt_ros2_pipeline.composed_tracker:main",
            "keyboard_monitor = olt_ros2_pipeline.keyboard_monitor:main",
            "detection_pose_filter = olt_ros2_pipeline.detection_pose_filter:main",
        ],
    },
)
