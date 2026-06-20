from typing import List, Tuple

import rclpy
from geometry_msgs.msg import Point
from my_depthai_interfaces.msg import AnimalLandmarkArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


SKELETON_EDGES: List[Tuple[int, int]] = [
    (0, 1),
    (0, 2),
    (1, 3),
    (2, 4),
    (0, 5),
    (0, 6),
    (5, 6),
    (5, 7),
    (7, 9),
    (6, 8),
    (8, 10),
    (5, 11),
    (6, 12),
    (11, 12),
    (11, 13),
    (13, 15),
    (12, 14),
    (14, 16),
]


class AnimalPoseMarkersNode(Node):
    def __init__(self) -> None:
        super().__init__("animal_pose_markers_node")

        self.declare_parameter("landmarks_topic", "/animal_pose/landmarks")
        self.declare_parameter("markers_topic", "/animal_pose/markers")
        self.declare_parameter("point_scale", 0.02)
        self.declare_parameter("line_width", 0.008)
        self.declare_parameter("text_scale", 0.05)
        self.declare_parameter("marker_lifetime_sec", 0.25)

        self._landmarks_topic = str(self.get_parameter("landmarks_topic").value)
        self._markers_topic = str(self.get_parameter("markers_topic").value)
        self._point_scale = float(self.get_parameter("point_scale").value)
        self._line_width = float(self.get_parameter("line_width").value)
        self._text_scale = float(self.get_parameter("text_scale").value)
        self._marker_lifetime_sec = float(self.get_parameter("marker_lifetime_sec").value)

        self._sub = self.create_subscription(
            AnimalLandmarkArray,
            self._landmarks_topic,
            self._on_landmarks,
            10,
        )
        self._pub = self.create_publisher(MarkerArray, self._markers_topic, 10)

        self.get_logger().info(
            f"Subscribing to {self._landmarks_topic}, publishing markers on {self._markers_topic}"
        )

    def _on_landmarks(self, msg: AnimalLandmarkArray) -> None:
        marker_array = MarkerArray()

        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        lifetime_sec, lifetime_nanosec = self._lifetime_parts()

        for animal_idx, animal in enumerate(msg.landmarks):
            base_id = animal_idx * 10

            points_marker = Marker()
            points_marker.header = msg.header
            points_marker.ns = "animal_points"
            points_marker.id = base_id
            points_marker.type = Marker.SPHERE_LIST
            points_marker.action = Marker.ADD
            points_marker.scale.x = self._point_scale
            points_marker.scale.y = self._point_scale
            points_marker.scale.z = self._point_scale
            points_marker.color.r = 0.2
            points_marker.color.g = 1.0
            points_marker.color.b = 0.4
            points_marker.color.a = 1.0
            points_marker.lifetime.sec = lifetime_sec
            points_marker.lifetime.nanosec = lifetime_nanosec
            points_marker.points = [
                Point(x=float(kp.x), y=float(kp.y), z=0.0) for kp in animal.landmark
            ]
            marker_array.markers.append(points_marker)

            lines_marker = Marker()
            lines_marker.header = msg.header
            lines_marker.ns = "animal_lines"
            lines_marker.id = base_id + 1
            lines_marker.type = Marker.LINE_LIST
            lines_marker.action = Marker.ADD
            lines_marker.scale.x = self._line_width
            lines_marker.color.r = 1.0
            lines_marker.color.g = 0.7
            lines_marker.color.b = 0.1
            lines_marker.color.a = 1.0
            lines_marker.lifetime.sec = lifetime_sec
            lines_marker.lifetime.nanosec = lifetime_nanosec
            for a, b in SKELETON_EDGES:
                if a < len(animal.landmark) and b < len(animal.landmark):
                    lines_marker.points.append(
                        Point(
                            x=float(animal.landmark[a].x),
                            y=float(animal.landmark[a].y),
                            z=0.0,
                        )
                    )
                    lines_marker.points.append(
                        Point(
                            x=float(animal.landmark[b].x),
                            y=float(animal.landmark[b].y),
                            z=0.0,
                        )
                    )
            marker_array.markers.append(lines_marker)

            text_marker = Marker()
            text_marker.header = msg.header
            text_marker.ns = "animal_text"
            text_marker.id = base_id + 2
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = self._text_scale
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.lifetime.sec = lifetime_sec
            text_marker.lifetime.nanosec = lifetime_nanosec
            text_marker.text = f"{animal.label} det:{animal.det_score:.2f} lm:{animal.lm_score:.2f}"
            text_marker.pose.position.x = float(animal.position.x)
            text_marker.pose.position.y = float(animal.position.y)
            text_marker.pose.position.z = 0.03
            text_marker.pose.orientation.w = 1.0
            marker_array.markers.append(text_marker)

        self._pub.publish(marker_array)

    def _lifetime_parts(self) -> Tuple[int, int]:
        total_nsec = max(0, int(self._marker_lifetime_sec * 1e9))
        sec = total_nsec // 1_000_000_000
        nanosec = total_nsec % 1_000_000_000
        return sec, nanosec


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnimalPoseMarkersNode()
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
