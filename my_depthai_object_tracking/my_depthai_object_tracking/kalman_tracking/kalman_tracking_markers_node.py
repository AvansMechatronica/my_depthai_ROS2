from typing import Tuple

import rclpy
from geometry_msgs.msg import Point
from my_depthai_interfaces.msg import TrackDetection2DArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class KalmanTrackingMarkersNode(Node):
    def __init__(self) -> None:
        super().__init__("kalman_tracking_markers_node")

        self.declare_parameter("tracklets_topic", "/kalman_tracking/tracklets")
        self.declare_parameter("markers_topic", "/kalman_tracking/markers")
        self.declare_parameter("point_scale", 0.06)
        self.declare_parameter("text_scale", 0.08)
        self.declare_parameter("marker_lifetime_sec", 0.25)

        self._tracklets_topic = str(self.get_parameter("tracklets_topic").value)
        self._markers_topic = str(self.get_parameter("markers_topic").value)
        self._point_scale = float(self.get_parameter("point_scale").value)
        self._text_scale = float(self.get_parameter("text_scale").value)
        self._marker_lifetime_sec = float(self.get_parameter("marker_lifetime_sec").value)

        self._sub = self.create_subscription(
            TrackDetection2DArray,
            self._tracklets_topic,
            self._on_tracklets,
            10,
        )
        self._pub = self.create_publisher(MarkerArray, self._markers_topic, 10)

        self.get_logger().info(
            f"Subscribing to {self._tracklets_topic}, publishing markers on {self._markers_topic}"
        )

    def _status_color(self, status: int) -> Tuple[float, float, float]:
        if status == 0:  # NEW
            return 1.0, 0.78, 0.0
        if status == 1:  # TRACKED
            return 0.0, 0.86, 0.0
        if status == 2:  # LOST
            return 0.0, 0.47, 1.0
        if status == 3:  # REMOVED
            return 1.0, 0.0, 0.0
        return 0.8, 0.8, 0.8

    def _on_tracklets(self, msg: TrackDetection2DArray) -> None:
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        lifetime_sec, lifetime_nanosec = self._lifetime_parts()

        for idx, det in enumerate(msg.detections):
            if len(det.results) > 0:
                xyz = det.results[0].pose.pose.position
                x = float(xyz.x)
                y = float(xyz.y)
                z = float(xyz.z)
            else:
                continue

            color_r, color_g, color_b = self._status_color(int(det.tracking_status))
            dot_marker = Marker()
            dot_marker.header = msg.header
            dot_marker.ns = "kalman_track_dot"
            dot_marker.id = idx
            dot_marker.type = Marker.SPHERE
            dot_marker.action = Marker.ADD
            dot_marker.scale.x = self._point_scale
            dot_marker.scale.y = self._point_scale
            dot_marker.scale.z = self._point_scale
            dot_marker.color.r = color_r
            dot_marker.color.g = color_g
            dot_marker.color.b = color_b
            dot_marker.color.a = 1.0
            dot_marker.lifetime.sec = lifetime_sec
            dot_marker.lifetime.nanosec = lifetime_nanosec
            dot_marker.pose.position = Point(x=x, y=y, z=z)
            dot_marker.pose.orientation.w = 1.0
            marker_array.markers.append(dot_marker)

            text_marker = Marker()
            text_marker.header = msg.header
            text_marker.ns = "kalman_track_text"
            text_marker.id = idx + 10000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = self._text_scale
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime.sec = lifetime_sec
            text_marker.lifetime.nanosec = lifetime_nanosec
            text_marker.pose.position = Point(x=x, y=y, z=z + 0.08)
            text_marker.pose.orientation.w = 1.0
            text_marker.text = (
                f"id:{det.tracking_id}\n"
                f"x:{x:.2f}\n"
                f"y:{y:.2f}\n"
                f"z:{z:.2f}"
            )
            marker_array.markers.append(text_marker)

        self._pub.publish(marker_array)

    def _lifetime_parts(self) -> Tuple[int, int]:
        total_nsec = max(0, int(self._marker_lifetime_sec * 1e9))
        sec = total_nsec // 1_000_000_000
        nanosec = total_nsec % 1_000_000_000
        return sec, nanosec


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KalmanTrackingMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
