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


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    use_sim = LaunchConfiguration("use_sim")

    m3t_data_dir = LaunchConfiguration("m3t_data_dir")
    m3t_data_dir = pathlib.Path(m3t_data_dir.perform(context)).absolute()

    dataset_name = LaunchConfiguration("dataset_name")
    dataset_name = dataset_name.perform(context)
    class_id_regex = f"^{dataset_name}-(.*?)$"

    happypose_labeler = Node(
        package="olt_ros2_pipeline",
        executable="happypose_labeler",
        name="happypose_labeler_node",
        parameters=[
            {
                "use_sim_time": use_sim,
                "track_max_dist": 0.1,
                "distance_cost_weight": 1.0,
                "angle_cost_weight": 0.2,
                "buffer_timeout": 6.0,
                "use_symmetry_minimization": True,
            }
        ],
        remappings=[
            ("/unlabeled/detections", "/happypose/detections"),
            ("/unlabeled/vision_info", "/happypose/vision_info"),
            ("/upsampled/detections", "/m3t_tracker/intermediate/detections"),
            ("/upsampled/vision_info", "/m3t_tracker/intermediate/vision_info"),
        ],
    )

    detection_pose_filter = Node(
        package="olt_ros2_pipeline",
        executable="detection_pose_filter",
        name="detection_pose_filter_node",
        parameters=[
            {
                "use_sim_time": use_sim,
                "filtering_frame_id": "world",
                "tracking_frame_id": "camera_color_optical_frame",
                "max_delta_distance": 0.05,
                "max_delta_angle": radians(25.0),
                "alpha_t": 0.95,
                "alpha_o": 0.95,
                "publish_desired_pose": True,
                "desired_pose": [0.2, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            }
        ],
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
                    "use_sim_time": use_sim,
                    "dataset_path": m3t_data_dir.as_posix(),
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
            package="detection2d_marker_publisher",
            executable="marker_publisher",
            namespace=namespace,
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim,
                    "class_id_regex": class_id_regex,
                    "filename_format": "${class_id}.obj",
                    "marker_lifetime": 3.0,
                    "mesh.use_vision_info_uri": False,
                    "mesh.uri": "file://" + m3t_data_dir.as_posix(),
                    "mesh.scale": 1.0,
                    "mesh.color_overwrite": color,
                }
            ],
        )
        for namespace, color in [
            ("happypose", [1.0, 1.0, 1.0, 0.6]),
            # ("labeled", [1.0, 1.0, 0.0, 0.6]),
            # ("catchup", [0.3, 0.3, 1.0, 0.6]),
            # ("m3t_tracker", [1.0, 0.3, 0.3, 0.6]),
            # ("m3t_tracker/filtered", [0.3, 1.0, 0.3, 0.6]),
        ]
    ]

    # Start RViz2 ROS node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{"use_sim_time": use_sim}],
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
        condition=UnlessCondition(use_sim),
        launch_arguments={
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
            "camera_namespace": "",
            "pointcloud.enable": "true",
        }.items(),
    )

    detection_visualizer = Node(
        package="olt_ros2_pipeline",
        executable="track_visualizer",
        name="track_visualizer_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim,
                "text_color": [0.0, 1.0, 0.0, 1.0],
                "text_size": 0.025,
                "marker_lifetime": 3.0,
            }
        ],
        remappings=[
            ("detections", "/m3t_tracker/filtered/detections"),
            ("vision_info", "/m3t_tracker/filtered/vision_info"),
            ("markers", "/labeled/markers_id"),
            ("poses", "/labeled/poses"),
        ],
    )

    return [
        detection_pose_filter,
        happypose_labeler,
        *marker_publishers,
        *trackers,
        rviz_node,
        # realsense2_camera,
        detection_visualizer,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "dataset_name",
            default_value="tless",
            description="Name of the dataset to be used in the pipeline.",
        ),
        DeclareLaunchArgument(
            "m3t_data_dir",
            description="Name of the dataset to be used in the pipeline.",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="False",
            description="Enables `use_sim` flag in all nodes. Disables Realsense node.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
