from pathlib import Path
import importlib.util
import queue
import sys
from typing import Callable, List

import cv2
import depthai as dai
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from depthai_nodes import GatheredData, Predictions
from depthai_nodes.message import Keypoints
from depthai_nodes.node import FrameCropper, GatherData, ParsingNeuralNetwork
from geometry_msgs.msg import Point, Pose2D
from my_depthai_interfaces.msg import HandLandmark, HandLandmarkArray
from rclpy.node import Node
from sensor_msgs.msg import Image

PADDING = 0.1
CONFIDENCE_THRESHOLD = 0.5


class OpenCVDisplayNode(dai.node.HostNode):
    def __init__(self) -> None:
        super().__init__()
        self.confidence_threshold = 0.5
        self.padding_factor = 0.1
        self.connection_pairs: List[List[int]] = [[]]
        self.output_queue = None
        self.gesture_recognizer: Callable[[List[List[float]]], str] = lambda _: ""

    def build(
        self,
        gathered_data: dai.Node.Output,
        video: dai.Node.Output,
        output_queue: queue.Queue,
        gesture_recognizer: Callable[[List[List[float]]], str],
        padding_factor: float,
        confidence_threshold: float,
        connections_pairs: List[List[int]],
    ) -> "OpenCVDisplayNode":
        self.output_queue = output_queue
        self.gesture_recognizer = gesture_recognizer
        self.confidence_threshold = confidence_threshold
        self.padding_factor = padding_factor
        self.connection_pairs = connections_pairs
        self.link_args(gathered_data, video)
        return self

    def process(self, gathered_data_msg: dai.Buffer, video_message: dai.ImgFrame) -> None:
        assert isinstance(gathered_data_msg, GatheredData)
        frame = video_message.getCvFrame()
        h, w = frame.shape[:2]

        detections_message = gathered_data_msg.reference_data
        detections_list = detections_message.detections
        hand_landmarks: List[HandLandmark] = []

        for ix, detection in enumerate(detections_list):
            keypoints_msg: Keypoints = gathered_data_msg.items[ix]["0"]
            confidence_msg: Predictions = gathered_data_msg.items[ix]["1"]
            handness_msg: Predictions = gathered_data_msg.items[ix]["2"]

            if confidence_msg.prediction < self.confidence_threshold:
                continue

            bbox = detection.getBoundingBox()
            bw = bbox.size.width
            bh = bbox.size.height
            xmin = bbox.center.x - bw / 2
            xmax = bbox.center.x + bw / 2
            ymin = bbox.center.y - bh / 2
            ymax = bbox.center.y + bh / 2
            padding = self.padding_factor

            x1 = max(int((xmin - padding) * w), 0)
            y1 = max(int((ymin - padding) * h), 0)
            x2 = min(int((xmax + padding) * w), w - 1)
            y2 = min(int((ymax + padding) * h), h - 1)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            slope_x = (xmax + padding) - (xmin - padding)
            slope_y = (ymax + padding) - (ymin - padding)
            xs, ys = [], []
            for kp in keypoints_msg.getKeypoints():
                x = min(max(xmin - padding + slope_x * kp.imageCoordinates.x, 0.0), 1.0)
                y = min(max(ymin - padding + slope_y * kp.imageCoordinates.y, 0.0), 1.0)
                xs.append(x)
                ys.append(y)

            for conn in self.connection_pairs:
                pt1_ix, pt2_ix = conn
                cv2.line(
                    frame,
                    (int(xs[pt1_ix] * w), int(ys[pt1_ix] * h)),
                    (int(xs[pt2_ix] * w), int(ys[pt2_ix] * h)),
                    (0, 200, 200),
                    2,
                )

            for x, y in zip(xs, ys):
                cv2.circle(frame, (int(x * w), int(y * h)), 4, (255, 100, 0), -1)

            gesture = self.gesture_recognizer([[kx, ky] for kx, ky in zip(xs, ys)])
            hand_label = "Left" if handness_msg.prediction < 0.5 else "Right"
            cv2.putText(
                frame,
                f"{hand_label} {gesture}",
                (x1, max(y1 - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
            )

            landmark_msg = HandLandmark()
            landmark_msg.label = hand_label
            landmark_msg.lm_score = float(confidence_msg.prediction)
            landmark_msg.landmark = [
                Pose2D(x=float(x), y=float(y), theta=0.0) for x, y in zip(xs, ys)
            ]
            landmark_msg.position = Point(
                x=float((xmin + xmax) / 2.0), y=float((ymin + ymax) / 2.0), z=0.0
            )
            landmark_msg.is_spatial = False
            hand_landmarks.append(landmark_msg)

        try:
            if self.output_queue is not None:
                self.output_queue.put_nowait((frame, hand_landmarks))
        except queue.Full:
            pass


class HandPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("hand_pose_node")

        self.declare_parameter("device", "")
        self.declare_parameter("media_path", "")
        self.declare_parameter("fps_limit", 0)
        self.declare_parameter("image_topic", "/hand_pose/image")
        self.declare_parameter("landmarks_topic", "/hand_pose/landmarks")
        self.declare_parameter("reconnect_interval_sec", 2.0)

        image_topic = str(self.get_parameter("image_topic").value)
        landmarks_topic = str(self.get_parameter("landmarks_topic").value)
        self._image_pub = self.create_publisher(Image, image_topic, 10)
        self._landmarks_pub = self.create_publisher(
            HandLandmarkArray, landmarks_topic, 10
        )
        self._output_queue: queue.Queue = queue.Queue(maxsize=4)
        self._reconnect_interval_sec = float(
            self.get_parameter("reconnect_interval_sec").value
        )
        if self._reconnect_interval_sec <= 0.0:
            self._reconnect_interval_sec = 2.0

        self._pipeline = None
        self._hand_pose_root = self._resolve_hand_pose_root()
        self._configure_imports(self._hand_pose_root)

        self._pipeline_start_logged = False
        self._retry_timer = self.create_timer(
            self._reconnect_interval_sec, self._ensure_pipeline_running
        )
        self._ensure_pipeline_running()
        self._timer = self.create_timer(0.01, self._on_timer)

    def _ensure_pipeline_running(self) -> None:
        if self._pipeline is not None:
            return
        try:
            self._start_pipeline()
            if not self._pipeline_start_logged:
                self.get_logger().info("Hand pose pipeline started.")
                self._pipeline_start_logged = True
        except RuntimeError as exc:
            self.get_logger().warn(
                f"DepthAI device not available yet ({exc}). Retrying in {self._reconnect_interval_sec:.1f}s."
            )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to start hand pose pipeline: {exc}. Retrying in {self._reconnect_interval_sec:.1f}s."
            )

    def _resolve_hand_pose_root(self) -> Path:
        current_file = Path(__file__).resolve()

        # Prefer installed ROS share path when available.
        try:
            share_dir = Path(get_package_share_directory("my_depthai_pose_estimation"))
            share_candidate = share_dir / "hand_pose"
            if share_candidate.exists():
                return share_candidate
        except PackageNotFoundError:
            pass

        # Fall back to common source/build workspace layouts.
        candidates = [
            current_file.parent,
            current_file.parents[1] / "hand_pose",
            current_file.parents[3]
            / "src"
            / "my_depthai_ROS2"
            / "pose_estimation"
            / "pose_estimation"
            / "hand_pose",
            current_file.parents[3]
            / "install"
            / "pose_estimation"
            / "share"
            / "pose_estimation"
            / "hand_pose",
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate

        raise FileNotFoundError("Could not locate hand_pose assets directory.")

    def _configure_imports(self, hand_pose_root: Path) -> None:
        global ProcessDetections, recognize_gesture
        gesture_module = self._load_module_from_path(
            "hand_pose_gesture_recognition",
            hand_pose_root / "utils" / "gesture_recognition.py",
        )
        process_module = self._load_module_from_path(
            "hand_pose_process",
            hand_pose_root / "utils" / "process.py",
        )
        recognize_gesture = gesture_module.recognize_gesture
        ProcessDetections = process_module.ProcessDetections

    def _load_module_from_path(self, module_name: str, module_path: Path):
        if not module_path.exists():
            raise FileNotFoundError(f"Required module file not found: {module_path}")

        spec = importlib.util.spec_from_file_location(module_name, module_path)
        if spec is None or spec.loader is None:
            raise ImportError(f"Could not load module spec for {module_path}")

        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def _start_pipeline(self) -> None:
        device_name = str(self.get_parameter("device").value).strip()
        media_path = str(self.get_parameter("media_path").value).strip()
        fps_limit = int(self.get_parameter("fps_limit").value)
        fps_value = fps_limit if fps_limit > 0 else None

        device = dai.Device(dai.DeviceInfo(device_name)) if device_name else dai.Device()
        platform = device.getPlatform().name
        self.get_logger().info(f"Platform: {platform}")

        frame_type = (
            dai.ImgFrame.Type.BGR888p if platform == "RVC2" else dai.ImgFrame.Type.BGR888i
        )

        if fps_value is None:
            fps_value = 8 if platform == "RVC2" else 30
            self.get_logger().info(f"FPS limit set to {fps_value} for {platform} platform.")

        det_model_path = self._hand_pose_root / "depthai_models" / f"mediapipe_palm_detection.{platform}.yaml"
        pose_model_path = self._hand_pose_root / "depthai_models" / f"mediapipe_hand_landmarker.{platform}.yaml"

        if not det_model_path.exists() or not pose_model_path.exists():
            raise FileNotFoundError(
                f"Missing model YAML files for platform {platform} in {self._hand_pose_root / 'depthai_models'}"
            )

        self._pipeline = dai.Pipeline(device)
        pipeline = self._pipeline

        det_model_description = dai.NNModelDescription.fromYamlFile(str(det_model_path))
        det_nn_archive = dai.NNArchive(dai.getModelFromZoo(det_model_description))

        pose_model_description = dai.NNModelDescription.fromYamlFile(str(pose_model_path))
        pose_nn_archive = dai.NNArchive(dai.getModelFromZoo(pose_model_description))

        if media_path:
            replay = pipeline.create(dai.node.ReplayVideo)
            replay.setReplayVideoFile(Path(media_path))
            replay.setOutFrameType(frame_type)
            replay.setLoop(True)
            replay.setFps(fps_value)
            input_node = replay.out
        else:
            cam = pipeline.create(dai.node.Camera).build()
            input_node = cam.requestOutput((768, 768), frame_type, fps=fps_value)

        resize_node = pipeline.create(dai.node.ImageManip)
        resize_node.setMaxOutputFrameSize(
            det_nn_archive.getInputWidth() * det_nn_archive.getInputHeight() * 3
        )
        resize_node.initialConfig.setOutputSize(
            det_nn_archive.getInputWidth(),
            det_nn_archive.getInputHeight(),
            mode=dai.ImageManipConfig.ResizeMode.STRETCH,
        )
        resize_node.initialConfig.setFrameType(frame_type)
        input_node.link(resize_node.inputImage)

        detection_nn: ParsingNeuralNetwork = pipeline.create(ParsingNeuralNetwork).build(
            resize_node.out, det_nn_archive
        )

        detections_processor = pipeline.create(ProcessDetections).build(
            detections_input=detection_nn.out,
            padding=PADDING,
            target_size=(pose_nn_archive.getInputWidth(), pose_nn_archive.getInputHeight()),
        )

        crop_output_size = (pose_nn_archive.getInputWidth(), pose_nn_archive.getInputHeight())
        hand_crop_node = (
            pipeline.create(FrameCropper)
            .fromManipConfigs(
                inputManipConfigs=detections_processor.config_output,
                maxOutputFrameSize=crop_output_size[0] * crop_output_size[1] * 3,
                waitForConfig=True,
            )
            .build(detection_nn.passthrough)
        )

        pose_nn: ParsingNeuralNetwork = pipeline.create(ParsingNeuralNetwork).build(
            hand_crop_node.out, pose_nn_archive
        )

        gather_data = pipeline.create(GatherData).build(
            cameraFps=fps_value,
            inputData=pose_nn.outputs,
            inputReference=detection_nn.out,
        )

        connection_pairs = (
            pose_nn_archive.getConfig().model.heads[0].metadata.extraParams["skeleton_edges"]
        )
        pipeline.create(OpenCVDisplayNode).build(
            gathered_data=gather_data.out,
            video=input_node,
            output_queue=self._output_queue,
            gesture_recognizer=recognize_gesture,
            padding_factor=PADDING,
            confidence_threshold=CONFIDENCE_THRESHOLD,
            connections_pairs=connection_pairs,
        )

        pipeline.start()

    def _on_timer(self) -> None:
        try:
            frame, hand_landmarks = self._output_queue.get_nowait()
        except queue.Empty:
            return

        image_msg = Image()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = "oak_rgb_camera_optical_frame"
        image_msg.height = frame.shape[0]
        image_msg.width = frame.shape[1]
        image_msg.encoding = "bgr8"
        image_msg.is_bigendian = False
        image_msg.step = frame.shape[1] * frame.shape[2]
        image_msg.data = frame.tobytes()
        self._image_pub.publish(image_msg)

        landmarks_msg = HandLandmarkArray()
        landmarks_msg.header.stamp = image_msg.header.stamp
        landmarks_msg.header.frame_id = image_msg.header.frame_id
        landmarks_msg.landmarks = hand_landmarks
        self._landmarks_pub.publish(landmarks_msg)

    def destroy_node(self) -> bool:
        if hasattr(self, "_retry_timer") and self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None
        if hasattr(self, "_timer") and self._timer is not None:
            self._timer.cancel()
            self._timer = None
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
