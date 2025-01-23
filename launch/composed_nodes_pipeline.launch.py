import pathlib
from math import radians

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

    happypose_labeler_remappings = [
        ("/stage_1/unlabeled/detections", "/happypose/detections"),
        ("/stage_1/unlabeled/vision_info", "/happypose/vision_info"),
    ]

    time_catchup_node_remappings = [
        ("/stage_2/reference/detections", "/stage_1/labeled/detections"),
        ("/stage_2/reference/vision_info", "/stage_1/labeled/vision_info"),
        ("/stage_2/color/image_raw", "/camera/color/image_raw"),
        ("/stage_2/color/camera_info", "/camera/color/camera_info"),
        ("/stage_2/depth/image_raw", "/camera/depth/image_rect_raw"),
        ("/stage_2/depth/camera_info", "/camera/depth/camera_info"),
    ]

    real_time_tracker_node_remappings = [
        ("/stage_3/reference/detections", "/stage_2/m3t_tracker/detections"),
        ("/stage_3/reference/vision_info", "/stage_2/m3t_tracker/vision_info"),
        ("/stage_3/color/image_raw", "/camera/color/image_raw"),
        ("/stage_3/color/camera_info", "/camera/color/camera_info"),
        ("/stage_3/depth/image_raw", "/camera/depth/image_rect_raw"),
        ("/stage_3/depth/camera_info", "/camera/depth/camera_info"),
    ]

    detection_filter_remappings = [
        ("/stage_4/unfiltered/detections", "/stage_3/m3t_tracker/detections"),
        ("/stage_4/unfiltered/vision_info", "/stage_3/m3t_tracker/vision_info"),
        ("/stage_4/filtered/detections", "/m3t_tracker/filtered/detections"),
        ("/stage_4/filtered/vision_info", "/m3t_tracker/filtered/vision_info"),
    ]

    composed_pipeline = Node(
        package="olt_ros2_pipeline",
        executable="composed_tracker",
        parameters=[
            {
                # m3t_time_catchup_node
                "dataset_path": m3t_data_path.as_posix(),
                "class_id_regex": class_id_regex,
                "filename_format": "${class_id}.${file_fmt}",
                # # detection_pose_filter
                "filtering_frame_id": "world",
                "tracking_frame_id": "camera_color_optical_frame",
                "alpha_t": 0.05,
                "alpha_o": radians(15.0),
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
            *happypose_labeler_remappings,
            *time_catchup_node_remappings,
            *real_time_tracker_node_remappings,
            *detection_filter_remappings,
        ],
    )

    marker_publishers = [
        Node(
            package="detection2d_marker_publisher",
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
        launch_arguments={
            "rgb_camera.profile": "640x480x30",
            "depth_module.profile": "640x480x30",
        }.items(),
    )

    return [
        composed_pipeline,
        *marker_publishers,
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
