from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
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
        default_value="0.25",
        description="Camera X position relative to parent frame.",
    )
    cam_pos_y_arg = DeclareLaunchArgument(
        "cam_pos_y",
        default_value="0.0",
        description="Camera Y position relative to parent frame.",
    )
    cam_pos_z_arg = DeclareLaunchArgument(
        "cam_pos_z",
        default_value="0.5",
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

    device_arg = DeclareLaunchArgument(
        "device",
        default_value="",
        description="Optional DepthAI device ID, name, or IP.",
    )
    fps_limit_arg = DeclareLaunchArgument(
        "fps_limit",
        default_value="0",
        description="FPS limit. Set 0 to use platform default.",
    )
    model_name_arg = DeclareLaunchArgument(
        "model_name",
        default_value="SimpleFruitsYoloV8.rvc2.tar.xz",
        description=(
            "Model file name inside depthai_models/ (.yaml or .tar.xz). "
            "Use model_path for an explicit custom file path; model_path overrides model_name when set."
        ),
    )
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Optional custom model file path (.yaml or .tar.xz). Overrides model_name when set.",
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/kalman_tracking/image",
        description="ROS topic for sensor_msgs/Image frames.",
    )
    tracklets_topic_arg = DeclareLaunchArgument(
        "tracklets_topic",
        default_value="/kalman_tracking/tracklets",
        description="ROS topic for my_depthai_interfaces/TrackDetection2DArray.",
    )
    markers_topic_arg = DeclareLaunchArgument(
        "markers_topic",
        default_value="/kalman_tracking/markers",
        description="ROS topic for visualization_msgs/MarkerArray markers.",
    )
    start_markers_arg = DeclareLaunchArgument(
        "start_markers",
        default_value="true",
        description="Start markers node for tracklet visualization.",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="oak_rgb_camera_optical_frame",
        description="frame_id used in published image headers.",
    )
    reconnect_interval_arg = DeclareLaunchArgument(
        "reconnect_interval_sec",
        default_value="2.0",
        description="Seconds between reconnect attempts when no device is available.",
    )
    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Start RViz2 alongside the kalman tracking node.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("my_depthai_object_tracking"),
                "rviz",
                "kalman_tracking.rviz",
            ]
        ),
        description="Absolute path to the RViz2 config file.",
    )

    kalman_tracking_node = Node(
        package="my_depthai_object_tracking",
        executable="kalman_tracking_node",
        name="kalman_tracking_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {
                "device": LaunchConfiguration("device"),
                "fps_limit": LaunchConfiguration("fps_limit"),
                "model_name": LaunchConfiguration("model_name"),
                "model_path": LaunchConfiguration("model_path"),
                "image_topic": LaunchConfiguration("image_topic"),
                "tracklets_topic": LaunchConfiguration("tracklets_topic"),
                "frame_id": LaunchConfiguration("frame_id"),
                "reconnect_interval_sec": LaunchConfiguration("reconnect_interval_sec"),
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    kalman_tracking_markers_node = Node(
        package="my_depthai_object_tracking",
        executable="kalman_tracking_markers_node",
        name="kalman_tracking_markers_node",
        output="screen",
        parameters=[
            {
                "tracklets_topic": LaunchConfiguration("tracklets_topic"),
                "markers_topic": LaunchConfiguration("markers_topic"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("start_markers")),
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
            device_arg,
            fps_limit_arg,
            model_name_arg,
            model_path_arg,
            image_topic_arg,
            tracklets_topic_arg,
            markers_topic_arg,
            frame_id_arg,
            reconnect_interval_arg,
            start_markers_arg,
            start_rviz_arg,
            rviz_config_arg,
            urdf_launch,
            kalman_tracking_node,
            kalman_tracking_markers_node,
            rviz_node,
        ]
    )
