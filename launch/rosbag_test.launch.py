from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    happypose_labeler = Node(
        package="m3t_tracker_examples",
        executable="happypose_labeler",
        name="happypose_labeler_node",
        parameters=[{"skip_msg": 1}],
    )

    # Start ROS node for M3T tracker
    trackers = [
        Node(
            package="m3t_tracker_ros",
            executable=node_name,
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "dataset_path": "/tmp/m3t_tracker_ros_data",
                    "class_id_regex": "^tless-(.*?)$",
                    "filename_format": "${class_id}.${file_fmt}",
                    "tracked_objects": [
                        "tless-obj_000008",
                        "tless-obj_000016",
                        "tless-obj_000024",
                        "tless-obj_000001",
                        "tless-obj_000009",
                        "tless-obj_000017",
                        "tless-obj_000025",
                        "tless-obj_000002",
                        "tless-obj_000010",
                        "tless-obj_000018",
                        "tless-obj_000026",
                        "tless-obj_000003",
                        "tless-obj_000011",
                        "tless-obj_000019",
                        "tless-obj_000027",
                        "tless-obj_000004",
                        "tless-obj_000012",
                        "tless-obj_000020",
                        "tless-obj_000028",
                        "tless-obj_000005",
                        "tless-obj_000013",
                        "tless-obj_000021",
                        "tless-obj_000029",
                        "tless-obj_000006",
                        "tless-obj_000014",
                        "tless-obj_000022",
                        "tless-obj_000030",
                        "tless-obj_000007",
                        "tless-obj_000015",
                        "tless-obj_000023",
                    ],
                    "tless-obj_000026.max_instances": 2,
                    "tless-obj_000001.max_instances": 5,
                    "tless-obj_000021.max_instances": 5,
                    "tless-obj_000023.max_instances": 5,
                    # "compensate_camera_motion": True,
                    # "camera_motion_stationary_frame_id": "panda2_link0",
                    "detection_to_image_time_slop": 10.0,
                    # "region_modality.model_occlusion": True,
                    # "region_modality.learning_rate_f": 0.5,
                    # "region_modality.learning_rate_b": 0.5,
                    # "optimizer.tikhonov_parameter_rotation": 500.0,
                    # "optimizer.tikhonov_parameter_translation": 20000.0,
                    # "use_texture_modality": True,
                    # "use_depth": True,
                }
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
            ("time_catchup_node", "labeled", "ketchup"),
            ("real_time_tracker_node", "ketchup", "m3t_tracker"),
        ]
    ]

    # Start mesh publisher
    happypose_marker_publisher = Node(
        package="happypose_marker_publisher",
        executable="marker_publisher",
        output="screen",
        parameters=[
            {
                "class_id_regex": "^tless-(.*?)$",
                "filename_format": "${class_id}.obj",
                "marker_lifetime": 1.0 / 3.0 + 0.005,
                "mesh.use_vision_info_uri": False,
                "mesh.uri": "file:///tmp/m3t_tracker_ros_data",
                "mesh.scale": 1.0,
                "mesh.color_overwrite": [0.5, 1.0, 0.5, 1.0],
            }
        ],
        remappings=[
            ("/reference/detections", "/m3t_tracker/detections"),
            ("/reference/vision_info", "/m3t_tracker/vision_info"),
            ("/marker_publisher_node/markers", "/m3t_tracker/markers"),
        ],
    )

    happypose_happypose_marker_publisher = Node(
        package="happypose_marker_publisher",
        executable="marker_publisher",
        output="screen",
        parameters=[
            {
                "class_id_regex": "^tless-(.*?)$",
                "filename_format": "${class_id}.obj",
                "marker_lifetime": 4.0,
                "mesh.use_vision_info_uri": False,
                "mesh.uri": "file:///tmp/m3t_tracker_ros_data",
                "mesh.scale": 1.0,
                "mesh.color_overwrite": [1.0, 0.3, 0.3, 0.2],
            }
        ],
        remappings=[
            # ("/reference/detections", "/happypose/detections"),
            # ("/reference/vision_info", "/happypose/vision_info"),
            ("/marker_publisher_node/markers", "/happypose/markers"),
        ],
    )

    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("m3t_tracker_examples"),
            "rviz",
            "rosbag_example.rviz",
        ]
    )

    # Start RViz2 ROS node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            rviz_config_path,
        ],
    )

    return [
        rviz_node,
        *trackers,
        happypose_marker_publisher,
        happypose_happypose_marker_publisher,
        happypose_labeler,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
