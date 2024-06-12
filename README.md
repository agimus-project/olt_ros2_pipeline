# olt_ros2_pipeline

ROS 2 reimplementation of Object Localization and Tracking vision pipeline.

## ROS API

### happypose_labeler

ROS node labeling detections by creating simple labels based on combining class_id with its number of instances in the received message.

### Publishers

- **labeled/detections** [vision_msgs/msg/Detection2DArray]

    Detections after applied labeling.

- **labeled/vision_info** [vision_msgs/msg/VisionInfo]

    Vision info message associated with labeled detections.

### Subscribers

- **unlabeled/detections** [vision_msgs/msg/Detection2DArray]

    Unlabeled detections.

- **unlabeled/vision_info** [vision_msgs/msg/VisionInfo]

    Vision info message associated with unlabeled detections.

### detection_pose_filter

ROS node applying basic pose filtering and safety management on top of 6D pose detection pipeline.
Ensures translation difference between consecutive poses is not too big. If detections are
within error threshold, applies first order smoothing.

### Publishers

- **filtered/detections** [vision_msgs/msg/Detection2DArray]

    Detections after safety check and filtering.

- **filtered/vision_info** [vision_msgs/msg/VisionInfo]

    Vision info message associated with filtered detections.

### Subscribers

- **unfiltered/detections** [vision_msgs/msg/Detection2DArray]

    Detections to filter.

- **unfiltered/vision_info** [vision_msgs/msg/VisionInfo]

    Vision info message associated with unfiltered detections.

### Service Servers

- **~/set_paramters** [rcl_interfaces/srv/SetParameters]

    Allows to dynamically change ROS parameters. For more information. For more information, refer to the [documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html).

### Parameters

Parameters are generated with [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library). Currently, no automatic documentation generation is set up. Refer to [detection_pose_filter_parameters.yaml](./olt_ros2_pipeline/detection_pose_filter_parameters.yaml) for more information.

Note that some of the parameters are possible to tune in the runtime. Refer to the code generation YAML file to see which of them are available.

### composed_tracker

ROS nodes combined within single multi-threaded executor for better performance. This setup
encapsulated pipeline with following ROS nodes:

- [happypose_labeler](./olt_ros2_pipeline/happypose_labeler.py): for detection labeling.

- [time_catchup_node](https://gitlab.laas.fr/kwojciecho/m3t_tracker_ros/-/blob/devel/m3t_tracker_ros/m3t_tracker_ros/time_catchup_node.py?ref_type=heads): for time compensation between happypose detections and most recent image.

- [real_time_tracker_node](https://gitlab.laas.fr/kwojciecho/m3t_tracker_ros/-/blob/devel/m3t_tracker_ros/m3t_tracker_ros/real_time_tracker_node.py?ref_type=heads): for real time pose tracking.

- [detection_pose_filter](./olt_ros2_pipeline/detection_pose_filter.py): for final pose filtering.
