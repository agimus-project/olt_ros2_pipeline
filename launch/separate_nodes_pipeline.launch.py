import pathlib

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    m3t_data_path = LaunchConfiguration("m3t_data_path")
    m3t_data_path = pathlib.Path(m3t_data_path.perform(context)).absolute()

    dataset_name = LaunchConfiguration("dataset_name")
    dataset_name = dataset_name.perform(context)
    class_id_regex = f"^{dataset_name}-(.*?)$"

    happypose_labeler = Node(
        package="olt_ros2_pipeline",
        executable="happypose_labeler",
        name="happypose_labeler_node",
        remappings=[
            ("/unlabeled/detections", "/happypose/detections"),
            ("/unlabeled/vision_info", "/happypose/vision_info"),
        ],
    )

    detection_pose_filter = Node(
        package="olt_ros2_pipeline",
        executable="detection_pose_filter",
        name="detection_pose_filter_node",
        remappings=[
            ("/unfiltered/detections", "/m3t_tracker/detections"),
            ("/unfiltered/vision_info", "/m3t_tracker/vision_info"),
            ("/filtered/detections", "/m3t_tracker/filtered/detections"),
            ("/filtered/vision_info", "/m3t_tracker/filtered/vision_info"),
        ],
    )

    # Start ROS node for M3T tracker
    trackers = [
        Node(
            package="m3t_tracker_ros",
            executable=node_name,
            output="screen",
            parameters=[
                {
                    "dataset_path": m3t_data_path.as_posix(),
                    "class_id_regex": class_id_regex,
                    "filename_format": "${class_id}.${file_fmt}",
                },
                PathJoinSubstitution(
                    [
                        FindPackageShare("olt_ros2_pipeline"),
                        "config",
                        "tracker_config.yaml",
                    ]
                ),
            ],
            remappings=[
                ("/m3t_tracker/detections", f"{out_topic}/detections"),
                ("/m3t_tracker/vision_info", f"{out_topic}/vision_info"),
                ("/reference/detections", f"{in_topic}/detections"),
                ("/reference/vision_info", f"{in_topic}/vision_info"),
                ("/color/image_raw", "/camera/color/image_raw"),
                ("/color/camera_info", "/camera/color/camera_info"),
                ("/depth/image_raw", "/camera/depth/image_rect_raw"),
                ("/depth/camera_info", "/camera/depth/camera_info"),
            ],
        )
        for node_name, in_topic, out_topic in [
            ("time_catchup_node", "labeled", "catchup"),
            ("real_time_tracker_node", "catchup", "m3t_tracker"),
        ]
    ]

    marker_publishers = [
        Node(
            package="happypose_marker_publisher",
            executable="marker_publisher",
            namespace=namespace,
            output="screen",
            parameters=[
                {
                    "class_id_regex": class_id_regex,
                    "filename_format": "${class_id}.obj",
                    "marker_lifetime": 4.0,
                    "mesh.use_vision_info_uri": False,
                    "mesh.uri": "file://" + m3t_data_path.as_posix(),
                    "mesh.scale": 1.0,
                    "mesh.color_overwrite": color,
                }
            ],
        )
        for namespace, color in [
            ("happypose", [1.0, 1.0, 1.0, 0.1]),
            ("m3t_tracker", [1.0, 0.3, 0.3, 0.2]),
            ("m3t_tracker/filtered", [0.3, 1.0, 0.3, 0.2]),
        ]
    ]

    # Start RViz2 ROS node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("olt_ros2_pipeline"),
                    "rviz",
                    "olt_ros2.rviz",
                ]
            ),
        ],
    )

    # Include launch file for RealSense camera
    realsense2_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("realsense2_camera"),
                        "launch",
                        "rs_launch.py",
                    ]
                )
            ]
        ),
    )

    return [
        detection_pose_filter,
        happypose_labeler,
        *marker_publishers,
        *trackers,
        rviz_node,
        realsense2_camera,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "dataset_name",
            default_value="tless",
            description="Name of the dataset to be used in the pipeline.",
        ),
        DeclareLaunchArgument(
            "m3t_data_path",
            description="Name of the dataset to be used in the pipeline.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
