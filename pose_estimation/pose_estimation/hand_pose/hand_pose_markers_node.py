from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Point
from my_depthai_interfaces.msg import HandLandmarkArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


SKELETON_EDGES: List[Tuple[int, int]] = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 4),
    (0, 5),
    (5, 6),
    (6, 7),
    (7, 8),
    (5, 9),
    (9, 10),
    (10, 11),
    (11, 12),
    (9, 13),
    (13, 14),
    (14, 15),
    (15, 16),
    (13, 17),
    (0, 17),
    (17, 18),
    (18, 19),
    (19, 20),
]


class HandPoseMarkersNode(Node):
    def __init__(self) -> None:
        super().__init__("hand_pose_markers_node")

        self.declare_parameter("landmarks_topic", "/hand_pose/landmarks")
        self.declare_parameter("markers_topic", "/hand_pose/markers")
        self.declare_parameter("point_scale", 0.01)
        self.declare_parameter("line_width", 0.005)
        self.declare_parameter("text_scale", 0.05)
        self.declare_parameter("marker_lifetime_sec", 0.2)

        self._landmarks_topic = str(self.get_parameter("landmarks_topic").value)
        self._markers_topic = str(self.get_parameter("markers_topic").value)
        self._point_scale = float(self.get_parameter("point_scale").value)
        self._line_width = float(self.get_parameter("line_width").value)
        self._text_scale = float(self.get_parameter("text_scale").value)
        self._marker_lifetime_sec = float(self.get_parameter("marker_lifetime_sec").value)

        self._sub = self.create_subscription(
            HandLandmarkArray,
            self._landmarks_topic,
            self._on_landmarks,
            10,
        )
        self._pub = self.create_publisher(MarkerArray, self._markers_topic, 10)

        self.get_logger().info(
            f"Subscribing to {self._landmarks_topic}, publishing markers on {self._markers_topic}"
        )

    def _on_landmarks(self, msg: HandLandmarkArray) -> None:
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        for hand_idx, hand in enumerate(msg.landmarks):
            base_id = hand_idx * 10
            color = self._color_for_label(hand.label)

            points_marker = Marker()
            points_marker.header = msg.header
            points_marker.ns = "hand_points"
            points_marker.id = base_id
            points_marker.type = Marker.SPHERE_LIST
            points_marker.action = Marker.ADD
            points_marker.scale.x = self._point_scale
            points_marker.scale.y = self._point_scale
            points_marker.scale.z = self._point_scale
            points_marker.color.r = color[0]
            points_marker.color.g = color[1]
            points_marker.color.b = color[2]
            points_marker.color.a = 1.0
            lifetime_sec, lifetime_nanosec = self._lifetime_parts()
            points_marker.lifetime.sec = lifetime_sec
            points_marker.lifetime.nanosec = lifetime_nanosec
            points_marker.points = [
                Point(x=float(kp.x), y=float(kp.y), z=0.0) for kp in hand.landmark
            ]
            marker_array.markers.append(points_marker)

            lines_marker = Marker()
            lines_marker.header = msg.header
            lines_marker.ns = "hand_lines"
            lines_marker.id = base_id + 1
            lines_marker.type = Marker.LINE_LIST
            lines_marker.action = Marker.ADD
            lines_marker.scale.x = self._line_width
            lines_marker.color.r = color[0]
            lines_marker.color.g = color[1]
            lines_marker.color.b = color[2]
            lines_marker.color.a = 1.0
            lines_marker.lifetime.sec = lifetime_sec
            lines_marker.lifetime.nanosec = lifetime_nanosec
            for a, b in SKELETON_EDGES:
                if a < len(hand.landmark) and b < len(hand.landmark):
                    lines_marker.points.append(
                        Point(
                            x=float(hand.landmark[a].x),
                            y=float(hand.landmark[a].y),
                            z=0.0,
                        )
                    )
                    lines_marker.points.append(
                        Point(
                            x=float(hand.landmark[b].x),
                            y=float(hand.landmark[b].y),
                            z=0.0,
                        )
                    )
            marker_array.markers.append(lines_marker)

            text_marker = Marker()
            text_marker.header = msg.header
            text_marker.ns = "hand_text"
            text_marker.id = base_id + 2
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = self._text_scale
            text_marker.color.r = color[0]
            text_marker.color.g = color[1]
            text_marker.color.b = color[2]
            text_marker.color.a = 1.0
            text_marker.lifetime.sec = lifetime_sec
            text_marker.lifetime.nanosec = lifetime_nanosec
            text_marker.text = f"{hand.label} ({hand.lm_score:.2f})"
            text_marker.pose.position.x = float(hand.position.x)
            text_marker.pose.position.y = float(hand.position.y)
            text_marker.pose.position.z = 0.03
            text_marker.pose.orientation.w = 1.0
            marker_array.markers.append(text_marker)

        self._pub.publish(marker_array)

    @staticmethod
    def _color_for_label(label: str) -> Tuple[float, float, float]:
        if label.lower() == "left":
            return (0.2, 0.8, 1.0)
        return (1.0, 0.5, 0.2)

    def _lifetime_parts(self) -> Tuple[int, int]:
        total_nsec = max(0, int(self._marker_lifetime_sec * 1e9))
        sec = total_nsec // 1_000_000_000
        nanosec = total_nsec % 1_000_000_000
        return sec, nanosec


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandPoseMarkersNode()
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
