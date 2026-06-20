from pathlib import Path
import queue
from typing import List, Tuple

import cv2
import depthai as dai
import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from depthai_nodes.message import Keypoints
from depthai_nodes.node import FrameCropper, GatherData, ImgDetectionsFilter, ParsingNeuralNetwork
from geometry_msgs.msg import Point, Pose2D
from my_depthai_interfaces.msg import AnimalLandmark, AnimalLandmarkArray
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D

PADDING = 0.1
VALID_LABELS = [0]


class AnimalPoseOverlayNode(dai.node.HostNode):
    def __init__(self) -> None:
        super().__init__()
        self._output_queue: queue.Queue = queue.Queue(maxsize=1)
        self._connection_pairs: List[Tuple[int, int]] = []
        self._padding = PADDING
        self._keypoint_conf_threshold = 0.5

    def build(
        self,
        gathered_data: dai.Node.Output,
        video: dai.Node.Output,
        output_queue: queue.Queue,
        connection_pairs: List[List[int]],
        padding: float,
        keypoint_conf_threshold: float,
    ) -> "AnimalPoseOverlayNode":
        self._output_queue = output_queue
        self._connection_pairs = [tuple(edge) for edge in connection_pairs]
        self._padding = padding
        self._keypoint_conf_threshold = keypoint_conf_threshold
        self.link_args(gathered_data, video)
        return self

    def process(self, gathered_data_msg: dai.Buffer, video_message: dai.ImgFrame) -> None:
        frame = video_message.getCvFrame()
        h, w = frame.shape[:2]
        animal_landmarks = []

        detections = gathered_data_msg.reference_data.detections
        keypoints_msg_list: List[Keypoints] = gathered_data_msg.items

        for detection, keypoints_msg in zip(detections, keypoints_msg_list):
            xmin, ymin, xmax, ymax = (
                detection.xmin,
                detection.ymin,
                detection.xmax,
                detection.ymax,
            )
            x1 = max(int((xmin - self._padding) * w), 0)
            y1 = max(int((ymin - self._padding) * h), 0)
            x2 = min(int((xmax + self._padding) * w), w - 1)
            y2 = min(int((ymax + self._padding) * h), h - 1)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 0), 2)

            slope_x = (xmax + self._padding) - (xmin - self._padding)
            slope_y = (ymax + self._padding) - (ymin - self._padding)
            xs: List[float] = []
            ys: List[float] = []
            confidences: List[float] = []
            for keypoint in keypoints_msg.getKeypoints():
                x = min(
                    max(xmin - self._padding + slope_x * keypoint.imageCoordinates.x, 0.0),
                    1.0,
                )
                y = min(
                    max(ymin - self._padding + slope_y * keypoint.imageCoordinates.y, 0.0),
                    1.0,
                )
                xs.append(x)
                ys.append(y)
                confidences.append(float(keypoint.confidence))

            for pt1_ix, pt2_ix in self._connection_pairs:
                if pt1_ix >= len(xs) or pt2_ix >= len(xs):
                    continue
                if (
                    confidences[pt1_ix] < self._keypoint_conf_threshold
                    or confidences[pt2_ix] < self._keypoint_conf_threshold
                ):
                    continue
                cv2.line(
                    frame,
                    (int(xs[pt1_ix] * w), int(ys[pt1_ix] * h)),
                    (int(xs[pt2_ix] * w), int(ys[pt2_ix] * h)),
                    (0, 220, 220),
                    2,
                )

            for x, y, confidence in zip(xs, ys, confidences):
                if confidence < self._keypoint_conf_threshold:
                    continue
                cv2.circle(frame, (int(x * w), int(y * h)), 3, (255, 80, 0), -1)

            mean_conf = sum(confidences) / len(confidences) if confidences else 0.0
            animal_landmarks.append(
                {
                    "label": "animal",
                    "det_score": float(detection.confidence),
                    "lm_score": float(mean_conf),
                    "bbox": [float(xmin), float(ymin), float(xmax), float(ymax)],
                    "landmarks": [
                        [float(x), float(y), float(conf)]
                        for x, y, conf in zip(xs, ys, confidences)
                    ],
                    "position": [
                        float((xmin + xmax) / 2.0),
                        float((ymin + ymax) / 2.0),
                    ],
                }
            )

        try:
            self._output_queue.put_nowait((frame, animal_landmarks))
        except queue.Full:
            pass


class ProcessDetections(dai.node.HostNode):
    def __init__(self) -> None:
        super().__init__()
        self.detections_input = self.createInput()
        self.config_output = self.createOutput()
        self.padding = PADDING
        self._target_h = 0
        self._target_w = 0

    def build(
        self,
        detections_input: dai.Node.Output,
        padding: float,
        target_size: Tuple[int, int],
    ) -> "ProcessDetections":
        self.padding = padding
        self._target_w = target_size[0]
        self._target_h = target_size[1]
        self.link_args(detections_input)
        return self

    def process(self, img_detections: dai.Buffer) -> None:
        assert isinstance(img_detections, dai.ImgDetections)
        configs_group = dai.MessageGroup()

        for i, detection in enumerate(img_detections.detections):
            cfg = dai.ImageManipConfig()
            rect = detection.getBoundingBox()

            new_rect = dai.RotatedRect()
            new_rect.center.x = rect.center.x
            new_rect.center.y = rect.center.y
            new_rect.size.width = rect.size.width + self.padding * 2
            new_rect.size.height = rect.size.height + self.padding * 2
            new_rect.angle = 0

            cfg.addCropRotatedRect(new_rect, normalizedCoords=True)
            cfg.setOutputSize(
                self._target_w,
                self._target_h,
                dai.ImageManipConfig.ResizeMode.STRETCH,
            )
            cfg.setReusePreviousImage(False)
            cfg.setTimestamp(img_detections.getTimestamp())
            cfg.setSequenceNum(img_detections.getSequenceNum())
            configs_group[f"cfg_{i}"] = cfg

        configs_group.setTimestamp(img_detections.getTimestamp())
        configs_group.setSequenceNum(img_detections.getSequenceNum())
        self.config_output.send(configs_group)


class AnimalPoseNode(Node):
    def __init__(self) -> None:
        super().__init__("animal_pose_node")

        self.declare_parameter("device", "")
        self.declare_parameter("media_path", "")
        self.declare_parameter("fps_limit", 0)
        self.declare_parameter("image_topic", "/animal_pose/image")
        self.declare_parameter("landmarks_topic", "/animal_pose/landmarks")
        self.declare_parameter("frame_id", "oak_rgb_camera_optical_frame")
        self.declare_parameter("reconnect_interval_sec", 2.0)
        self.declare_parameter("keypoint_conf_threshold", 0.5)

        self._image_topic = str(self.get_parameter("image_topic").value)
        self._landmarks_topic = str(self.get_parameter("landmarks_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._reconnect_interval_sec = float(
            self.get_parameter("reconnect_interval_sec").value
        )
        if self._reconnect_interval_sec <= 0.0:
            self._reconnect_interval_sec = 2.0

        self._image_pub = self.create_publisher(Image, self._image_topic, 10)
        self._landmarks_pub = self.create_publisher(
            AnimalLandmarkArray, self._landmarks_topic, 10
        )
        self._output_queue: queue.Queue = queue.Queue(maxsize=4)
        self._animal_pose_root = self._resolve_animal_pose_root()
        self._pipeline = None
        self._pipeline_start_logged = False

        self._retry_timer = self.create_timer(
            self._reconnect_interval_sec, self._ensure_pipeline_running
        )
        self._timer = self.create_timer(0.01, self._on_timer)
        self._ensure_pipeline_running()

    def _ensure_pipeline_running(self) -> None:
        if self._pipeline is not None:
            return
        try:
            self._start_pipeline()
            if not self._pipeline_start_logged:
                self.get_logger().info("Animal pose pipeline started.")
                self._pipeline_start_logged = True
        except RuntimeError as exc:
            self.get_logger().warn(
                f"DepthAI device not available yet ({exc}). Retrying in {self._reconnect_interval_sec:.1f}s."
            )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to start animal pose pipeline: {exc}. Retrying in {self._reconnect_interval_sec:.1f}s."
            )

    def _start_pipeline(self) -> None:
        device_name = str(self.get_parameter("device").value).strip()
        media_path = str(self.get_parameter("media_path").value).strip()
        fps_limit = int(self.get_parameter("fps_limit").value)
        keypoint_conf_threshold = float(
            self.get_parameter("keypoint_conf_threshold").value
        )
        fps_value = fps_limit if fps_limit > 0 else None

        device = dai.Device(dai.DeviceInfo(device_name)) if device_name else dai.Device()
        platform = device.getPlatform().name
        self.get_logger().info(f"Platform: {platform}")

        frame_type = (
            dai.ImgFrame.Type.BGR888p if platform == "RVC2" else dai.ImgFrame.Type.BGR888i
        )
        if fps_value is None:
            fps_value = 5 if platform == "RVC2" else 20
            self.get_logger().info(f"FPS limit set to {fps_value} for {platform} platform.")

        self._pipeline = dai.Pipeline(device)
        pipeline = self._pipeline

        models_dir = self._animal_pose_root / "depthai_models"
        det_model_path = models_dir / f"wildlife_megadetector.{platform}.yaml"
        pose_model_path = models_dir / f"superanimal_landmarker.{platform}.yaml"
        if not det_model_path.exists() or not pose_model_path.exists():
            raise FileNotFoundError(
                f"Missing model YAML files for platform {platform} in {models_dir}"
            )

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
            nn_input = replay
            input_frame_output = replay.out
        else:
            cam = pipeline.create(dai.node.Camera).build()
            nn_input = cam
            input_frame_output = cam.requestOutput((768, 768), frame_type, fps=fps_value)

        detection_nn: ParsingNeuralNetwork = pipeline.create(ParsingNeuralNetwork).build(
            nn_input, det_nn_archive, fps=fps_value
        )
        detection_nn.input.setBlocking(False)
        detection_nn.input.setMaxSize(1)

        detections_filter = pipeline.create(ImgDetectionsFilter).build(detection_nn.out)
        detections_filter.keepLabels(VALID_LABELS)

        pose_output_size = (
            pose_nn_archive.getInputWidth(),
            pose_nn_archive.getInputHeight(),
        )
        detections_processor = pipeline.create(ProcessDetections).build(
            detections_input=detections_filter.out,
            padding=PADDING,
            target_size=pose_output_size,
        )

        pose_manip = (
            pipeline.create(FrameCropper)
            .fromManipConfigs(
                inputManipConfigs=detections_processor.config_output,
                maxOutputFrameSize=pose_output_size[0] * pose_output_size[1] * 3,
                waitForConfig=True,
            )
            .build(detection_nn.passthrough)
        )

        pose_nn: ParsingNeuralNetwork = pipeline.create(ParsingNeuralNetwork).build(
            pose_manip.out, pose_nn_archive
        )
        pose_nn.input.setBlocking(False)
        pose_nn.input.setMaxSize(1)

        gather_data = pipeline.create(GatherData).build(
            cameraFps=fps_value,
            inputData=pose_nn.out,
            inputReference=detections_filter.out,
        )

        connection_pairs = (
            pose_nn_archive.getConfig().model.heads[0].metadata.extraParams["skeleton_edges"]
        )
        pipeline.create(AnimalPoseOverlayNode).build(
            gathered_data=gather_data.out,
            video=input_frame_output,
            output_queue=self._output_queue,
            connection_pairs=connection_pairs,
            padding=PADDING,
            keypoint_conf_threshold=keypoint_conf_threshold,
        )

        pipeline.start()

    def _resolve_animal_pose_root(self) -> Path:
        current_file = Path(__file__).resolve()

        try:
            share_dir = Path(get_package_share_directory("my_depthai_pose_estimation"))
            share_candidate = share_dir / "animal_pose"
            if share_candidate.exists():
                return share_candidate
        except PackageNotFoundError:
            pass

        candidates = [
            current_file.parent,
            current_file.parents[1] / "animal_pose",
            current_file.parents[3]
            / "src"
            / "my_depthai_ROS2"
            / "pose_estimation"
            / "pose_estimation"
            / "animal_pose",
            current_file.parents[3]
            / "install"
            / "pose_estimation"
            / "share"
            / "pose_estimation"
            / "animal_pose",
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate

        raise FileNotFoundError("Could not locate animal_pose assets directory.")

    def _on_timer(self) -> None:
        try:
            frame, animal_landmarks = self._output_queue.get_nowait()
        except queue.Empty:
            return

        image_msg = Image()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = self._frame_id
        image_msg.height = frame.shape[0]
        image_msg.width = frame.shape[1]
        image_msg.encoding = "bgr8"
        image_msg.is_bigendian = False
        image_msg.step = frame.shape[1] * frame.shape[2]
        image_msg.data = frame.tobytes()
        self._image_pub.publish(image_msg)

        landmarks_msg = AnimalLandmarkArray()
        landmarks_msg.header.stamp = image_msg.header.stamp
        landmarks_msg.header.frame_id = image_msg.header.frame_id
        for animal in animal_landmarks:
            animal_msg = AnimalLandmark()
            animal_msg.label = animal["label"]
            animal_msg.det_score = animal["det_score"]
            animal_msg.lm_score = animal["lm_score"]

            xmin, ymin, xmax, ymax = animal["bbox"]
            bbox_msg = BoundingBox2D()
            bbox_msg.center.position.x = float((xmin + xmax) / 2.0)
            bbox_msg.center.position.y = float((ymin + ymax) / 2.0)
            bbox_msg.center.theta = 0.0
            bbox_msg.size_x = float(max(0.0, xmax - xmin))
            bbox_msg.size_y = float(max(0.0, ymax - ymin))
            animal_msg.bbox = bbox_msg

            animal_msg.landmark = [
                Pose2D(x=kp[0], y=kp[1], theta=kp[2]) for kp in animal["landmarks"]
            ]
            animal_msg.position = Point(
                x=animal["position"][0],
                y=animal["position"][1],
                z=0.0,
            )
            animal_msg.is_spatial = False
            landmarks_msg.landmarks.append(animal_msg)

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
    node = AnimalPoseNode()
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
