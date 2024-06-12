from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    # Start ROS node of happypose
    happypose_node = Node(
        package="happypose_ros",
        executable="happypose_node",
        name="happypose_node",
        parameters=[
            ParameterFile(
                param_file=PathJoinSubstitution(
                    [
                        FindPackageShare("olt_ros2_pipeline"),
                        "config",
                        "happypose_config.yaml",
                    ]
                ),
                allow_substs=True,
            ),
        ],
        remappings=[
            # Remapped topics have to match the names from
            # happypose_examples/config/cosypose_params.yaml
            ("/cam_1/image_color", "/camera/color/image_raw"),
            ("/cam_1/camera_info", "/camera/color/camera_info"),
        ],
    )

    return [happypose_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "dataset_name",
            default_value="tless",
            description="Name of the dataset to be used in the pipeline.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
