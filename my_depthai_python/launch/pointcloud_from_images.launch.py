from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('my_depthai_python'), 'config', 'pointcloud_from_images.yaml']
        ),
        description='Path to YAML file with pointcloud_from_images ROS parameters',
    )
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='stereo/depth_raw',
        description='Input depth image topic (raw metric depth, e.g., mono16)',
    )
    rgb_topic_arg = DeclareLaunchArgument(
        'rgb_topic',
        default_value='camera/rgb',
        description='Input RGB image topic',
    )
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='camera/camera_info',
        description='Input camera info topic for intrinsics',
    )
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='stereo/pointcloud',
        description='Published PointCloud2 topic',
    )

    node = Node(
        package='my_depthai_python',
        executable='pointcloud_from_images',
        name='pointcloud_from_images',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'depth_topic': LaunchConfiguration('depth_topic'),
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            },
        ],
    )

    return LaunchDescription(
        [
            params_file_arg,
            depth_topic_arg,
            rgb_topic_arg,
            camera_info_topic_arg,
            pointcloud_topic_arg,
            node,
        ]
    )