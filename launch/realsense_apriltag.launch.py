import pathlib
from math import radians

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import UnlessCondition
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    use_sim = LaunchConfiguration("use_sim")

    apriltag_params = (
        PathJoinSubstitution(
            [
                FindPackageShare("olt_ros2_pipeline"),
                "config",
                "tags_36h11.yaml",
            ]
        ),
    )

    container = ComposableNodeContainer(
        name="image_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        parameters=[{"use_sim_time": use_sim}],
        composable_node_descriptions=[
            ComposableNode(
                package="realsense2_camera",
                namespace="",
                plugin="realsense2_camera::RealSenseNodeFactory",
                name="camera",
                parameters=[{"use_sim_time": use_sim}],
                # parameters=[set_configurable_parameters(realsense_node_params)],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify",
                namespace="rectify",
                parameters=[{"use_sim_time": use_sim}],
                remappings=[
                    ("/rectify/camera_info", "/camera/color/camera_info"),
                    ("/rectify/image", "/camera/color/image_raw"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="apriltag_ros",
                plugin="AprilTagNode",
                name="apriltag",
                namespace="apriltag",
                remappings=[
                    ("/rectify/camera_info", "/camera/color/camera_info"),
                    ("/apriltag/image_rect", "/rectify/image_rect"),
                ],
                parameters=[
                    {"history": "keep_last"},
                    {"use_sim_time": use_sim},
                    apriltag_params,
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="both",
    )

    apriltag_tf_to_world_pub = Node(
        package="olt_ros2_pipeline",
        executable="apriltag_tf_to_world",
        name="apriltag_tf_to_world",
        parameters=[{"use_sim_time": use_sim}],
        output="screen",
    )

    return [container, apriltag_tf_to_world_pub]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_sim",
            default_value="false",
            description="Enables `use_sim` flag in all nodes. Disables Realsense node.",
            choices=["true", "false"],
        )
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
