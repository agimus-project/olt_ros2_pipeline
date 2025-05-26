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

# realsense_node_params = [{'name': 'serial_no',              'default': "''", 'description': 'choose device by serial number'},
#                          {'name': 'usb_port_id',            'default': "''", 'description': 'choose device by usb port id'},
#                          {'name': 'device_type',            'default': "''", 'description': 'choose device by type'},
#                          {'name': 'log_level',              'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
#                          {'name': 'rgb_camera.profile',     'default': '0,0,0', 'description': 'color image width'},
#                          {'name': 'enable_color',           'default': 'true', 'description': 'enable color stream'},
#                          {'name': 'enable_depth',           'default': 'true', 'description': 'enable depth stream'},
#                          {'name': 'enable_infra',           'default': 'false', 'description': 'enable infra stream'},
#                          {'name': 'enable_infra1',          'default': 'true', 'description': 'enable infra1 stream'},
#                          {'name': 'enable_infra2',          'default': 'true', 'description': 'enable infra2 stream'},
#                          {'name': 'enable_gyro',            'default': 'true', 'description': "enable gyro stream"},
#                          {'name': 'enable_accel',           'default': 'true', 'description': "enable accel stream"},
#                          {'name': 'unite_imu_method',       'default': "1", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
#                          {'name': 'intra_process_comms',    'default': 'true', 'description': "enable intra-process communication"},
#                          {'name': 'enable_sync',            'default': 'true', 'description': "'enable sync mode'"},
#                          {'name': 'pointcloud.enable',      'default': 'true', 'description': ''},
#                          {'name': 'enable_rgbd',            'default': 'true', 'description': "'enable rgbd topic'"},
#                          {'name': 'align_depth.enable',     'default': 'true', 'description': "'enable align depth filter'"},
#                          {'name': 'publish_tf',             'default': 'true', 'description': '[bool] enable/disable publishing static & dynamic TF'},
#                          {'name': 'tf_publish_rate',        'default': '1.0', 'description': '[double] rate in HZ for publishing dynamic TF'},
#                         ]


# def set_configurable_parameters(parameters):
#     return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    use_sim = LaunchConfiguration("use_sim")
    # Include launch file for RealSense camera
    # realsense2_camera = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             PathJoinSubstitution(
    #                 [
    #                     FindPackageShare("realsense2_camera"),
    #                     "launch",
    #                     "rs_launch.py",
    #                 ]
    #             )
    #         ]
    #     ),
    #     launch_arguments={
    #         # "rgb_camera.color_profile": "640x480x30",
    #         # "depth_module.depth_profile": "640x480x30",
    #         "camera_namespace": "",
    #         # "pointcloud.enable": "true",
    #     }.items(),
    # )

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

    return [container]


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
