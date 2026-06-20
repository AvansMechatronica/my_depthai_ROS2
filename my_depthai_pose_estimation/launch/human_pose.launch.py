from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    device_arg = DeclareLaunchArgument(
        "device",
        default_value="",
        description="Optional DepthAI device id, name, or IP.",
    )
    media_path_arg = DeclareLaunchArgument(
        "media_path",
        default_value="",
        description="Optional path to a media file. Leave empty to use camera input.",
    )
    model_arg = DeclareLaunchArgument(
        "model",
        default_value="luxonis/lite-hrnet:18-coco-192x256",
        description="Human pose model from model zoo.",
    )
    fps_limit_arg = DeclareLaunchArgument(
        "fps_limit",
        default_value="0",
        description="FPS limit. Use 0 for platform default.",
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/human_pose/image",
        description="ROS topic for annotated human pose image frames.",
    )
    landmarks_topic_arg = DeclareLaunchArgument(
        "landmarks_topic",
        default_value="/human_pose/landmarks",
        description="ROS topic for HumanLandmarkArray detections.",
    )
    markers_topic_arg = DeclareLaunchArgument(
        "markers_topic",
        default_value="/human_pose/markers",
        description="ROS topic for MarkerArray human skeleton output.",
    )
    start_markers_arg = DeclareLaunchArgument(
        "start_markers",
        default_value="true",
        description="Start human pose marker publisher node.",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="oak_rgb_camera_optical_frame",
        description="frame_id to publish in sensor_msgs/Image header.",
    )
    reconnect_interval_arg = DeclareLaunchArgument(
        "reconnect_interval_sec",
        default_value="2.0",
        description="Retry interval in seconds when no DepthAI device is available.",
    )
    keypoint_conf_threshold_arg = DeclareLaunchArgument(
        "keypoint_conf_threshold",
        default_value="0.5",
        description="Minimum keypoint confidence used for overlay drawing.",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Start RViz2 alongside the human pose node.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("my_depthai_pose_estimation"), "rviz", "human_pose.rviz"]
        ),
        description="Absolute path to the RViz2 config file.",
    )

    start_urdf_arg = DeclareLaunchArgument(
        "start_urdf",
        default_value="true",
        description="Start camera URDF/TF publisher launch.",
    )
    camera_model_arg = DeclareLaunchArgument(
        "camera_model",
        default_value="OAK-D",
        description="DepthAI camera model used by URDF.",
    )
    tf_prefix_arg = DeclareLaunchArgument(
        "tf_prefix",
        default_value="oak",
        description="TF prefix / camera name for URDF frames.",
    )
    base_frame_arg = DeclareLaunchArgument(
        "base_frame",
        default_value="oak-d_frame",
        description="Base frame name for camera URDF.",
    )
    parent_frame_arg = DeclareLaunchArgument(
        "parent_frame",
        default_value="world",
        description="Parent frame to attach camera URDF.",
    )
    cam_pos_x_arg = DeclareLaunchArgument(
        "cam_pos_x",
        default_value="0.0",
        description="Camera X position relative to parent frame.",
    )
    cam_pos_y_arg = DeclareLaunchArgument(
        "cam_pos_y",
        default_value="0.0",
        description="Camera Y position relative to parent frame.",
    )
    cam_pos_z_arg = DeclareLaunchArgument(
        "cam_pos_z",
        default_value="0.0",
        description="Camera Z position relative to parent frame.",
    )
    cam_roll_arg = DeclareLaunchArgument(
        "cam_roll",
        default_value="0.0",
        description="Camera roll relative to parent frame.",
    )
    cam_pitch_arg = DeclareLaunchArgument(
        "cam_pitch",
        default_value="0.0",
        description="Camera pitch relative to parent frame.",
    )
    cam_yaw_arg = DeclareLaunchArgument(
        "cam_yaw",
        default_value="0.0",
        description="Camera yaw relative to parent frame.",
    )

    human_pose_node = Node(
        package="my_depthai_pose_estimation",
        executable="human_pose_node",
        name="human_pose_node",
        output="screen",
        parameters=[
            {
                "device": LaunchConfiguration("device"),
                "media_path": LaunchConfiguration("media_path"),
                "model": LaunchConfiguration("model"),
                "fps_limit": LaunchConfiguration("fps_limit"),
                "image_topic": LaunchConfiguration("image_topic"),
                "landmarks_topic": LaunchConfiguration("landmarks_topic"),
                "frame_id": LaunchConfiguration("frame_id"),
                "reconnect_interval_sec": LaunchConfiguration("reconnect_interval_sec"),
                "keypoint_conf_threshold": LaunchConfiguration("keypoint_conf_threshold"),
            }
        ],
    )

    human_pose_markers_node = Node(
        package="my_depthai_pose_estimation",
        executable="human_pose_markers_node",
        name="human_pose_markers_node",
        output="screen",
        parameters=[
            {
                "landmarks_topic": LaunchConfiguration("landmarks_topic"),
                "markers_topic": LaunchConfiguration("markers_topic"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("start_markers")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("my_depthai_python"), "launch", "urdf_launch.py"]
            )
        ),
        launch_arguments={
            "camera_model": LaunchConfiguration("camera_model"),
            "tf_prefix": LaunchConfiguration("tf_prefix"),
            "base_frame": LaunchConfiguration("base_frame"),
            "parent_frame": LaunchConfiguration("parent_frame"),
            "cam_pos_x": LaunchConfiguration("cam_pos_x"),
            "cam_pos_y": LaunchConfiguration("cam_pos_y"),
            "cam_pos_z": LaunchConfiguration("cam_pos_z"),
            "cam_roll": LaunchConfiguration("cam_roll"),
            "cam_pitch": LaunchConfiguration("cam_pitch"),
            "cam_yaw": LaunchConfiguration("cam_yaw"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_urdf")),
    )

    return LaunchDescription(
        [
            device_arg,
            media_path_arg,
            model_arg,
            fps_limit_arg,
            image_topic_arg,
            landmarks_topic_arg,
            markers_topic_arg,
            start_markers_arg,
            frame_id_arg,
            reconnect_interval_arg,
            keypoint_conf_threshold_arg,
            start_rviz_arg,
            rviz_config_arg,
            start_urdf_arg,
            camera_model_arg,
            tf_prefix_arg,
            base_frame_arg,
            parent_frame_arg,
            cam_pos_x_arg,
            cam_pos_y_arg,
            cam_pos_z_arg,
            cam_roll_arg,
            cam_pitch_arg,
            cam_yaw_arg,
            urdf_launch,
            human_pose_node,
            human_pose_markers_node,
            rviz_node,
        ]
    )
