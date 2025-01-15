from pathlib import Path
from typing import List

from setuptools import find_packages, setup

from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = "olt_ros2_pipeline"
project_source_dir = Path(__file__).parent

module_name = "detection_pose_filter_parameters"
yaml_file = "olt_ros2_pipeline/detection_pose_filter_parameters.yaml"
generate_parameter_module(module_name, yaml_file)

module_name = "happypose_labeler_parameters"
yaml_file = "olt_ros2_pipeline/happypose_labeler_parameters.yaml"
generate_parameter_module(module_name, yaml_file)


def get_files(dir: Path, pattern: str) -> List[str]:
    return [x.as_posix() for x in (dir).glob(pattern) if x.is_file()]


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
        (
            "share/ament_index/resource_index/packages",
            ["resource/track_visualizer"],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            f"share/{package_name}/config",
            get_files(project_source_dir / "config", "*.yaml"),
        ),
        (
            f"share/{package_name}/launch",
            get_files(project_source_dir / "launch", "*.launch.py"),
        ),
        (
            f"share/{package_name}/rviz",
            get_files(project_source_dir / "rviz", "*.rviz"),
        ),
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
            "happypose_labeler = olt_ros2_pipeline.happypose_labeler:main",
            "detection_pose_filter = olt_ros2_pipeline.detection_pose_filter:main",
            "track_visualizer = olt_ros2_pipeline.track_visualizer:main",
        ],
    },
)
