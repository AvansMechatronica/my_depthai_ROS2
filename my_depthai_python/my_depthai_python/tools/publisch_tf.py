#!/usr/bin/env python3
import tarfile
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from pathlib import Path

from std_msgs.msg import String
from depthai_ros_msgs.msg import SpatialDetectionArray

from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import json
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError


def _get_resource_dir() -> Path:
    try:
        return Path(get_package_share_directory('my_depthai_python')) / 'resources'
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[2] / 'resources'


def _read_labels_from_archive(archive_path: Path) -> list:
    """Read class labels from config.json inside a .rvc2.tar.xz NNArchive."""
    with tarfile.open(archive_path, 'r:xz') as tar:
        config_member = tar.getmember('config.json')
        with tar.extractfile(config_member) as f:
            config = json.load(f)
    heads = config.get('model', {}).get('heads', [])
    if heads:
        return heads[0].get('metadata', {}).get('classes', [])
    return []

class Publisch_TF(Node):

    def __init__(self):
        super().__init__('publisch_tf')

        self.declare_parameter("resourceBaseFolder", "")
        self.declare_parameter("nn_archive", "")
        self.declare_parameter("detections_topic", "color/yolov4_spatial_detections")
        self.declare_parameter("marker_topic", "color/ObjectText")
        self.declare_parameter("frame_id", "oak_rgb_camera_optical_frame")
        self.declare_parameter("marker_lifetime_sec", 10.0)

        nn_archive_name = self.get_parameter("nn_archive").get_parameter_value().string_value

        self.detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value
        self.marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.marker_lifetime_sec = self.get_parameter("marker_lifetime_sec").get_parameter_value().double_value

        resource_dir = _get_resource_dir()
        if not nn_archive_name:
            raise ValueError('Parameter nn_archive must not be empty')
        nn_archive_path = resource_dir / nn_archive_name
        if not nn_archive_path.is_file():
            raise FileNotFoundError(f'NN archive not found: {nn_archive_path}')

        # Read class labels from config.json inside the .rvc2.tar.xz NNArchive
        self.labels = _read_labels_from_archive(nn_archive_path)
        if not self.labels:
            raise RuntimeError(f'No labels found in archive: {nn_archive_path}')

        self.labels_dict = {label: 0 for label in self.labels}
        self.get_logger().info(f'Loaded labels: {self.labels}')


        self.subSpatialDetection = self.create_subscription(
            SpatialDetectionArray,
            self.detections_topic,
            self.spatial_dections_callback,
            10)
        self.subSpatialDetection  # prevent unused variable warning

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pubTextMarker = self.create_publisher(Marker, self.marker_topic, 10)

    def spatial_dections_callback(self, spatial_detection_array_msg):
        for label in self.labels:
            self.labels_dict[label] = 0
        for detection in spatial_detection_array_msg.detections:
            label_name = None
            score = -1.0
            for result in detection.results:
                if result.score > score:
                    label_name = result.class_id
                    score = result.score

            if label_name not in self.labels_dict:
                continue

            position = detection.position

            child_frame_id = label_name + "_" + str(self.labels_dict[label_name])
            self.labels_dict[label_name] += 1

            # Read message content and assign it to
            # corresponding tf variables
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.frame_id
            t.child_frame_id = child_frame_id

            t.transform.translation.x = position.x
            t.transform.translation.y = -position.y
            t.transform.translation.z = position.z

            # For the same reason, turtle can only rotate around one axis
            # and this why we set rotation in x and y to 0 and obtain
            # rotation in z axis from the message
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            # Send the transformation
            self.tf_broadcaster.sendTransform(t)

            text_marker = Marker()  # Text
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.header.frame_id = child_frame_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.pose.position.y = 0.00
            text_marker.pose.position.x = 0.00
            text_marker.pose.position.z = -0.03

            text_marker.scale.x = text_marker.scale.y = text_marker.scale.z = 0.1#0.06
            text_marker.color.r = text_marker.color.g = text_marker.color.b = text_marker.color.a = 1.0
            text_marker.text = child_frame_id
            text_marker.lifetime = Duration(seconds=self.marker_lifetime_sec).to_msg()
            self.pubTextMarker.publish(text_marker)   

def main(args=None):
    rclpy.init(args=args)

    publisch_tf = Publisch_TF()

    rclpy.spin(publisch_tf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisch_tf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
