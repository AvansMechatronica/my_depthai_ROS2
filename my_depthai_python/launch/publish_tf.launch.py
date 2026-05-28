import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    depthai_examples_path = get_package_share_directory('my_depthai_python')

    default_resources_path = os.path.join(depthai_examples_path,
                                'resources')
    print('Default resources path..............')
    print(default_resources_path)

    config_path = os.path.join(depthai_examples_path, 'config', 'publish_tf.yaml')

    publisch_tf_node = launch_ros.actions.Node(
            package='my_depthai_python', executable='publisch_tf',
            output='screen',
            parameters=[config_path])

    ld = LaunchDescription()
    ld.add_action(publisch_tf_node)
    return ld

