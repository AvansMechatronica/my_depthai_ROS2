from pathlib import Path
from typing import Optional

import cv2
import depthai as dai
import rclpy
from my_depthai_interfaces.msg import TrackDetection2D, TrackDetection2DArray
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, ObjectHypothesis, ObjectHypothesisWithPose

from .utils.kalman_filter_node import KalmanFilterNode


class KalmanTrackingNode(Node):
    def __init__(self) -> None:
        super().__init__("kalman_tracking_node")

        self.declare_parameter("device", "")
        self.declare_parameter("fps_limit", 0)
        self.declare_parameter("image_topic", "/kalman_tracking/image")
        self.declare_parameter("tracklets_topic", "/kalman_tracking/tracklets")
        self.declare_parameter("frame_id", "oak_rgb_camera_optical_frame")
        self.declare_parameter("reconnect_interval_sec", 2.0)

        image_topic = str(self.get_parameter("image_topic").value)
        tracklets_topic = str(self.get_parameter("tracklets_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._image_pub = self.create_publisher(Image, image_topic, 10)
        self._tracklets_pub = self.create_publisher(
            TrackDetection2DArray, tracklets_topic, 10
        )

        self._reconnect_interval_sec = float(
            self.get_parameter("reconnect_interval_sec").value
        )
        if self._reconnect_interval_sec <= 0.0:
            self._reconnect_interval_sec = 2.0

        self._device: Optional[dai.Device] = None
        self._pipeline: Optional[dai.Pipeline] = None
        self._rgb_queue: Optional[dai.DataOutputQueue] = None
        self._tracklets_queue: Optional[dai.DataOutputQueue] = None
        self._latest_tracklets: Optional[dai.Tracklets] = None
        self._labels = []
        self._pipeline_start_logged = False

        self._retry_timer = self.create_timer(
            self._reconnect_interval_sec, self._ensure_pipeline_running
        )
        self._ensure_pipeline_running()
        self._timer = self.create_timer(0.01, self._on_timer)

    def _status_color(self, status_name: str):
        status_colors = {
            "NEW": (255, 200, 0),
            "TRACKED": (0, 220, 0),
            "LOST": (0, 120, 255),
            "REMOVED": (0, 0, 255),
        }
        return status_colors.get(status_name, (200, 200, 200))

    def _status_to_int(self, status_name: str) -> int:
        status_map = {
            "NEW": 0,
            "TRACKED": 1,
            "LOST": 2,
            "REMOVED": 3,
        }
        return status_map.get(status_name, -1)

    def _model_path_for_platform(self, platform: str) -> Path:
        model_name = f"yolov6_nano_r2_coco.{platform}.yaml"

        source_path = Path(__file__).resolve().parent / "depthai_models" / model_name
        if source_path.exists():
            return source_path

        from ament_index_python.packages import get_package_share_directory

        share_path = (
            Path(get_package_share_directory("my_depthai_object_tracking"))
            / "kalman_tracking"
            / "depthai_models"
            / model_name
        )
        if share_path.exists():
            return share_path

        raise FileNotFoundError(
            f"Could not find model YAML for platform {platform}: {model_name}"
        )

    def _ensure_pipeline_running(self) -> None:
        if self._pipeline is not None:
            return
        try:
            self._start_pipeline()
            if not self._pipeline_start_logged:
                self.get_logger().info("Kalman tracking pipeline started.")
                self._pipeline_start_logged = True
        except RuntimeError as exc:
            self.get_logger().warn(
                "DepthAI device not available yet "
                f"({exc}). Retrying in {self._reconnect_interval_sec:.1f}s."
            )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to start kalman tracking pipeline: {exc}. "
                f"Retrying in {self._reconnect_interval_sec:.1f}s."
            )

    def _start_pipeline(self) -> None:
        device_name = str(self.get_parameter("device").value).strip()
        fps_limit = int(self.get_parameter("fps_limit").value)
        fps_value = fps_limit if fps_limit > 0 else None

        self._device = (
            dai.Device(dai.DeviceInfo(device_name)) if device_name else dai.Device()
        )
        platform = self._device.getPlatform().name
        self.get_logger().info(f"Platform: {platform}")

        if fps_value is None:
            fps_value = 20 if platform == "RVC2" else 30
            self.get_logger().info(
                f"FPS limit set to {fps_value} for {platform} platform."
            )

        if len(self._device.getConnectedCameras()) < 3:
            raise RuntimeError(
                "Device must have 3 cameras (color, left and right) in order to "
                "run this node."
            )

        model_path = self._model_path_for_platform(platform)
        self._pipeline = dai.Pipeline(self._device)
        pipeline = self._pipeline

        model_description = dai.NNModelDescription.fromYamlFile(str(model_path))
        nn_archive = dai.NNArchive(dai.getModelFromZoo(model_description))
        labels = nn_archive.getConfig().model.heads[0].metadata.classes
        self._labels = labels
        person_label = labels.index("person")

        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        left_cam = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B, sensorFps=fps_value
        )
        right_cam = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C, sensorFps=fps_value
        )
        stereo = pipeline.create(dai.node.StereoDepth).build(
            left=left_cam.requestOutput((640, 400)),
            right=right_cam.requestOutput((640, 400)),
            presetMode=dai.node.StereoDepth.PresetMode.HIGH_DETAIL,
        )
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        if platform == "RVC2":
            stereo.setOutputSize(*nn_archive.getInputSize())
        stereo.setLeftRightCheck(True)
        stereo.setRectification(True)

        nn = pipeline.create(dai.node.SpatialDetectionNetwork).build(
            input=cam, stereo=stereo, nnArchive=nn_archive, fps=fps_value
        )
        nn.setBoundingBoxScaleFactor(0.7)
        nn.setDepthLowerThreshold(100)
        nn.setDepthUpperThreshold(5000)
        if platform == "RVC2":
            nn.setNNArchive(nn_archive, numShaves=6)

        object_tracker = pipeline.create(dai.node.ObjectTracker)
        object_tracker.setDetectionLabelsToTrack([person_label])
        if platform == "RVC2":
            object_tracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        else:
            object_tracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
        object_tracker.setTrackerIdAssignmentPolicy(
            dai.TrackerIdAssignmentPolicy.UNIQUE_ID
        )

        nn.passthrough.link(object_tracker.inputTrackerFrame)
        nn.passthrough.link(object_tracker.inputDetectionFrame)
        nn.out.link(object_tracker.inputDetections)

        calibration_handler = self._device.readCalibration()
        baseline = calibration_handler.getBaselineDistance() * 10
        focal_length = calibration_handler.getCameraIntrinsics(
            dai.CameraBoardSocket.CAM_C, 640, 400
        )[0][0]

        pipeline.create(KalmanFilterNode).build(
            rgb=nn.passthrough,
            tracker_out=object_tracker.out,
            baseline=baseline,
            focal_length=focal_length,
            label_map=labels,
        )

        self._rgb_queue = nn.passthrough.createOutputQueue(maxSize=4, blocking=False)
        self._tracklets_queue = object_tracker.out.createOutputQueue(
            maxSize=4, blocking=False
        )
        pipeline.start()

    def _stop_pipeline(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None
        self._rgb_queue = None
        self._tracklets_queue = None
        self._latest_tracklets = None
        self._labels = []
        self._device = None

    def _draw_tracklets(self, frame, tracklets: dai.Tracklets) -> None:
        height, width = frame.shape[:2]
        for tracklet in tracklets.tracklets:
            roi = tracklet.roi.denormalize(width, height)
            x1 = int(roi.topLeft().x)
            y1 = int(roi.topLeft().y)
            x2 = int(roi.bottomRight().x)
            y2 = int(roi.bottomRight().y)

            status_name = tracklet.status.name
            status_color = self._status_color(status_name)

            cv2.rectangle(frame, (x1, y1), (x2, y2), status_color, 2)

            try:
                label = self._labels[tracklet.label]
            except Exception:
                label = str(tracklet.label)

            xyz_text = (
                f"X:{int(tracklet.spatialCoordinates.x)} "
                f"Y:{int(tracklet.spatialCoordinates.y)} "
                f"Z:{int(tracklet.spatialCoordinates.z)} mm"
            )
            id_text = f"ID:{tracklet.id} {label} {status_name}"

            text_y = y1 - 8 if y1 > 28 else y1 + 20
            cv2.putText(
                frame,
                id_text,
                (x1, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                status_color,
                1,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                xyz_text,
                (x1, text_y + 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

    def _publish_tracklets_msg(
        self, tracklets: dai.Tracklets, frame_width: int, frame_height: int
    ) -> None:
        track_msg = TrackDetection2DArray()
        track_msg.header.stamp = self.get_clock().now().to_msg()
        track_msg.header.frame_id = self._frame_id

        for tracklet in tracklets.tracklets:
            roi = tracklet.roi.denormalize(frame_width, frame_height)
            x1 = float(roi.topLeft().x)
            y1 = float(roi.topLeft().y)
            x2 = float(roi.bottomRight().x)
            y2 = float(roi.bottomRight().y)

            det = TrackDetection2D()
            bbox = BoundingBox2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.center.theta = 0.0
            bbox.size_x = x2 - x1
            bbox.size_y = y2 - y1
            det.bbox = bbox

            hyp_pose = ObjectHypothesisWithPose()
            hyp = ObjectHypothesis()
            try:
                hyp.class_id = self._labels[tracklet.label]
            except Exception:
                hyp.class_id = str(tracklet.label)
            hyp.score = 1.0
            hyp_pose.hypothesis = hyp
            hyp_pose.pose.pose.position.x = float(tracklet.spatialCoordinates.x) / 1000.0
            hyp_pose.pose.pose.position.y = float(tracklet.spatialCoordinates.y) / 1000.0
            hyp_pose.pose.pose.position.z = float(tracklet.spatialCoordinates.z) / 1000.0
            hyp_pose.pose.pose.orientation.w = 1.0
            det.results.append(hyp_pose)

            det.is_tracking = True
            det.tracking_id = str(tracklet.id)
            det.tracking_age = int(tracklet.age) if hasattr(tracklet, "age") else 0
            det.tracking_status = self._status_to_int(tracklet.status.name)
            track_msg.detections.append(det)

        self._tracklets_pub.publish(track_msg)

    def _on_timer(self) -> None:
        if self._pipeline is None or self._rgb_queue is None:
            return
        if not self._pipeline.isRunning():
            self.get_logger().warn("Pipeline stopped; waiting for reconnect timer.")
            self._stop_pipeline()
            return

        try:
            if self._tracklets_queue is not None:
                while True:
                    tracklets = self._tracklets_queue.tryGet()
                    if tracklets is None:
                        break
                    assert isinstance(tracklets, dai.Tracklets)
                    self._latest_tracklets = tracklets

            frame = self._rgb_queue.tryGet()
            if frame is None:
                return

            assert isinstance(frame, dai.ImgFrame)
            cv_frame = frame.getCvFrame()
            if self._latest_tracklets is not None:
                self._draw_tracklets(cv_frame, self._latest_tracklets)
                self._publish_tracklets_msg(
                    self._latest_tracklets, cv_frame.shape[1], cv_frame.shape[0]
                )

            image_msg = Image()
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.header.frame_id = self._frame_id
            image_msg.height = cv_frame.shape[0]
            image_msg.width = cv_frame.shape[1]
            image_msg.encoding = "bgr8"
            image_msg.is_bigendian = False
            image_msg.step = cv_frame.shape[1] * cv_frame.shape[2]
            image_msg.data = cv_frame.tobytes()
            self._image_pub.publish(image_msg)
        except Exception as exc:
            self.get_logger().error(f"Failed to publish image frame: {exc}")

    def destroy_node(self) -> bool:
        if hasattr(self, "_retry_timer") and self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None
        if hasattr(self, "_timer") and self._timer is not None:
            self._timer.cancel()
            self._timer = None
        self._stop_pipeline()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KalmanTrackingNode()
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
