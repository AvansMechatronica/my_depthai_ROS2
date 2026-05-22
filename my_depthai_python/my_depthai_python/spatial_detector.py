#!/usr/bin/env python3

import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

try:
    import depthai as dai
except ImportError as exc:
    raise RuntimeError(
        'depthai is not installed for this Python interpreter: '
        f'{sys.executable}. Install it in the same environment used for colcon build.'
    ) from exc

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from vision_msgs.msg import BoundingBox2D, ObjectHypothesis
from depthai_ros_msgs.msg import SpatialDetection, SpatialDetectionArray


def _get_resource_dir() -> Path:
    try:
        return Path(get_package_share_directory('my_depthai_python')) / 'resources'
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[1] / 'resources'


def _load_network_config(config_path: Path) -> dict:
    with config_path.open('r', encoding='utf-8') as handle:
        return json.load(handle)


def _get_mono_resolution(width: int, height: int):
    resolution_map = {
        (640, 400): dai.MonoCameraProperties.SensorResolution.THE_400_P,
        (640, 480): dai.MonoCameraProperties.SensorResolution.THE_480_P,
        (1280, 720): dai.MonoCameraProperties.SensorResolution.THE_720_P,
        (1280, 800): dai.MonoCameraProperties.SensorResolution.THE_800_P,
    }
    return resolution_map.get(
        (width, height),
        dai.MonoCameraProperties.SensorResolution.THE_400_P,
    )


def _colorize_depth_frame(depth_frame: np.ndarray) -> np.ndarray:
    depth_downscaled = depth_frame[::4]
    if np.all(depth_downscaled == 0):
        min_depth = 0
    else:
        min_depth = np.percentile(depth_downscaled[depth_downscaled != 0], 1)
    max_depth = np.percentile(depth_downscaled, 99)
    depth_frame_color = np.interp(
        depth_frame,
        (min_depth, max_depth),
        (0, 255),
    ).astype(np.uint8)
    return cv2.applyColorMap(depth_frame_color, cv2.COLORMAP_HOT)


class SpatialDetectorNode(Node):
    def __init__(self):
        super().__init__('spatial_detector')

        self.declare_parameter('image_topic', 'camera/rgb')
        self.declare_parameter('depth_topic', 'stereo/depth')
        self.declare_parameter('depth_preview_topic', 'stereo/depth_color')
        self.declare_parameter('detections_topic', 'spatial_detections')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 400)
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('queue_size', 4)
        self.declare_parameter('reconnect_cooldown_sec', 2.0)
        self.declare_parameter('max_reconnect_attempts', 20)
        self.declare_parameter('pipeline_mode', 'v3')
        self.declare_parameter('depth_source', 'stereo')
        self.declare_parameter('stereo_extended_disparity', False)
        self.declare_parameter('blob_name', 'SimpleFruitsYoloV5.blob')
        self.declare_parameter('config_name', 'SimpleFruitsYoloV5.json')

        depth_width = int(self.get_parameter('width').value)
        depth_height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        self.queue_size = int(self.get_parameter('queue_size').value)
        self.reconnect_cooldown_sec = float(
            self.get_parameter('reconnect_cooldown_sec').value
        )
        self.max_reconnect_attempts = int(
            self.get_parameter('max_reconnect_attempts').value
        )
        pipeline_mode = self.get_parameter('pipeline_mode').value
        depth_source = self.get_parameter('depth_source').value
        stereo_extended_disparity = bool(
            self.get_parameter('stereo_extended_disparity').value
        )
        blob_name = self.get_parameter('blob_name').value
        config_name = self.get_parameter('config_name').value
        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        depth_preview_topic = self.get_parameter('depth_preview_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        self.frame_id = 'oak_rgb_camera_optical_frame'
        timer_period = 1.0 / fps if fps > 0.0 else 1.0 / 20.0

        resource_dir = _get_resource_dir()
        blob_path = resource_dir / blob_name
        config_path = resource_dir / config_name
        if not blob_path.is_file():
            raise FileNotFoundError(f'Network blob not found: {blob_path}')
        if not config_path.is_file():
            raise FileNotFoundError(f'Network config not found: {config_path}')

        network_config = _load_network_config(config_path)
        metadata = network_config['nn_config']['NN_specific_metadata']
        labels = network_config.get('mappings', {}).get('labels', [])
        input_size = network_config['nn_config']['input_size']
        rgb_width, rgb_height = [int(value) for value in input_size.split('x', maxsplit=1)]
        self.width = rgb_width
        self.height = rgb_height

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, image_topic, 10)
        self.depth_pub = self.create_publisher(Image, depth_topic, 10)
        self.depth_preview_pub = self.create_publisher(Image, depth_preview_topic, 10)
        self.detection_pub = self.create_publisher(
            SpatialDetectionArray, detections_topic, 10
        )

        self.pipeline = None
        self.rgb_queue = None
        self.depth_queue = None
        self.detection_queue = None
        self.timer = None
        self._last_rgb_image = None
        self._last_depth_image = None
        self._last_depth_preview_image = None
        self._reconnect_attempts = 0
        self._last_reconnect_attempt = 0.0
        self._reconnect_exhausted_logged = False

        self.pipeline_config = {
            'fps': fps,
            'pipeline_mode': pipeline_mode,
            'rgb_width': rgb_width,
            'rgb_height': rgb_height,
            'depth_width': depth_width,
            'depth_height': depth_height,
            'depth_source': depth_source,
            'stereo_extended_disparity': stereo_extended_disparity,
            'blob_path': blob_path,
            'metadata': metadata,
            'labels': labels,
            'blob_name': blob_name,
            'config_name': config_name,
            'image_topic': image_topic,
            'depth_topic': depth_topic,
            'depth_preview_topic': depth_preview_topic,
            'detections_topic': detections_topic,
        }

        self._build_and_start_pipeline()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def _build_and_start_pipeline(self):
        cfg = self.pipeline_config

        self.pipeline = dai.Pipeline()

        if cfg['pipeline_mode'] == 'legacy':
            self.get_logger().info("Using legacy pipeline construction (DepthAI v2 style).")
            self._build_pipeline_legacy(cfg)
        elif cfg['pipeline_mode'] == 'v3':
            self.get_logger().info("Using v3 pipeline construction (DepthAI v3 style).")
            self._build_pipeline_v3(cfg)
        else:
            raise ValueError(
                f"Invalid pipeline_mode: {cfg['pipeline_mode']!r}. Use 'legacy' or 'v3'."
            )

        self.pipeline.start()
        self._reconnect_attempts = 0
        self._reconnect_exhausted_logged = False
        self.get_logger().info(
            f"SpatialDetector started - mode={cfg['pipeline_mode']!r} blob={cfg['blob_name']!r} "
            f"config={cfg['config_name']!r} depth={cfg['depth_source']!r} "
            f"extended_disparity={cfg['stereo_extended_disparity']!r} "
            f"rgb_size={self.width}x{self.height} fps={cfg['fps']} "
            f"image->{cfg['image_topic']!r} depth->{cfg['depth_topic']!r} "
            f"depth_preview->{cfg['depth_preview_topic']!r} "
            f"detections->{cfg['detections_topic']!r}"
        )

    def _configure_spatial_network(self, cfg: dict):
        spatial_det_net = self.pipeline.create(dai.node.SpatialDetectionNetwork)
        spatial_det_net.setBlobPath(cfg['blob_path'])
        spatial_det_net.input.setBlocking(False)
        spatial_det_net.setConfidenceThreshold(
            float(cfg['metadata']['confidence_threshold'])
        )
        spatial_det_net.setBoundingBoxScaleFactor(0.5)
        spatial_det_net.setDepthLowerThreshold(100)
        spatial_det_net.setDepthUpperThreshold(5000)
        spatial_det_net.detectionParser.setNumClasses(int(cfg['metadata']['classes']))
        spatial_det_net.detectionParser.setCoordinateSize(
            int(cfg['metadata']['coordinates'])
        )
        spatial_det_net.detectionParser.setAnchors(
            [float(anchor) for anchor in cfg['metadata'].get('anchors', [])]
        )
        spatial_det_net.detectionParser.setAnchorMasks(
            {
                name: [int(index) for index in indices]
                for name, indices in cfg['metadata'].get('anchor_masks', {}).items()
            }
        )
        spatial_det_net.detectionParser.setIouThreshold(
            float(cfg['metadata']['iou_threshold'])
        )
        if cfg['labels']:
            spatial_det_net.detectionParser.setClasses(cfg['labels'])
        spatial_det_net.spatialLocationCalculator.initialConfig.setSegmentationPassthrough(
            False
        )
        return spatial_det_net

    def _build_pipeline_v3(self, cfg: dict):
        nn_width = 512
        nn_height = 384

        cam_rgb = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A,
            sensorFps=cfg['fps'],
        )
        mono_left = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B,
            sensorFps=cfg['fps'],
        )
        mono_right = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C,
            sensorFps=cfg['fps'],
        )

        rgb_out = cam_rgb.requestOutput(
            size=(nn_width, nn_height),
            fps=cfg['fps'],
        )

        if cfg['depth_source'] == 'stereo':
            depth_node = self.pipeline.create(dai.node.StereoDepth)
            depth_node.setExtendedDisparity(cfg['stereo_extended_disparity'])
            depth_node.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            mono_left.requestOutput((cfg['depth_width'], cfg['depth_height'])).link(
                depth_node.left
            )
            mono_right.requestOutput((cfg['depth_width'], cfg['depth_height'])).link(
                depth_node.right
            )
        elif cfg['depth_source'] == 'neural':
            depth_node = self.pipeline.create(dai.node.NeuralDepth).build(
                mono_left.requestFullResolutionOutput(),
                mono_right.requestFullResolutionOutput(),
                dai.DeviceModelZoo.NEURAL_DEPTH_LARGE,
            )
        else:
            self.get_logger().fatal(f"Unknown depth_source: {cfg['depth_source']!r}")
            raise ValueError(f"Invalid depth_source: {cfg['depth_source']}")

        test_w_yolo_v6_nano = True
        if test_w_yolo_v6_nano:
            modelDescription = dai.NNModelDescription("yolov6-nano")
            spatial_det_net = self.pipeline.create(dai.node.SpatialDetectionNetwork).build(
            cam_rgb, depth_node, modelDescription)
            spatial_det_net.spatialLocationCalculator.initialConfig.setSegmentationPassthrough(False)
            spatial_det_net.input.setBlocking(False)
            spatial_det_net.setDepthLowerThreshold(100)
            spatial_det_net.setDepthUpperThreshold(5000)
        else:
            spatial_det_net = self._configure_spatial_network(cfg)   

        manip = self.pipeline.create(dai.node.ImageManip)
        manip.initialConfig.setOutputSize(nn_width, nn_height)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        manip.setMaxOutputFrameSize(nn_width * nn_height * 3)

        # Keep published image size and bbox scaling aligned with the actual NN input.
        self.width = nn_width
        self.height = nn_height

        rgb_out.link(manip.inputImage)
        manip.out.link(spatial_det_net.input)

        self.rgb_queue = manip.out.createOutputQueue(
            maxSize=self.queue_size,
            blocking=False,
        )
        self.depth_queue = depth_node.depth.createOutputQueue(
            maxSize=self.queue_size,
            blocking=False,
        )
        self.detection_queue = spatial_det_net.out.createOutputQueue(
            maxSize=self.queue_size,
            blocking=False,
        )

    def _build_pipeline_legacy(self, cfg: dict):
        color_cam = self.pipeline.create(dai.node.ColorCamera)
        color_cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        color_cam.setPreviewSize(cfg['rgb_width'], cfg['rgb_height'])
        color_cam.setVideoSize(cfg['rgb_width'], cfg['rgb_height'])
        color_cam.setInterleaved(False)
        color_cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        color_cam.setPreviewKeepAspectRatio(False)
        color_cam.setFps(cfg['fps'])

        mono_resolution = _get_mono_resolution(cfg['depth_width'], cfg['depth_height'])
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_left.setResolution(mono_resolution)
        mono_left.setFps(cfg['fps'])

        mono_right = self.pipeline.create(dai.node.MonoCamera)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        mono_right.setResolution(mono_resolution)
        mono_right.setFps(cfg['fps'])

        if cfg['depth_source'] == 'stereo':
            depth_node = self.pipeline.create(dai.node.StereoDepth)
            depth_node.setExtendedDisparity(cfg['stereo_extended_disparity'])
            depth_node.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            mono_left.out.link(depth_node.left)
            mono_right.out.link(depth_node.right)
        elif cfg['depth_source'] == 'neural':
            depth_node = self.pipeline.create(dai.node.NeuralDepth).build(
                mono_left.out,
                mono_right.out,
                dai.DeviceModelZoo.NEURAL_DEPTH_LARGE,
            )
        else:
            self.get_logger().fatal(f"Unknown depth_source: {cfg['depth_source']!r}")
            raise ValueError(f"Invalid depth_source: {cfg['depth_source']}")

        spatial_det_net = self._configure_spatial_network(cfg)

        manip = self.pipeline.create(dai.node.ImageManip)
        manip.initialConfig.setOutputSize(cfg['rgb_width'], cfg['rgb_height'])
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        manip.setMaxOutputFrameSize(cfg['rgb_width'] * cfg['rgb_height'] * 3)

        color_cam.preview.link(manip.inputImage)
        manip.out.link(spatial_det_net.input)
        depth_node.depth.link(spatial_det_net.inputDepth)

        self.rgb_queue = manip.out.createOutputQueue(
            maxSize=self.queue_size,
            blocking=False,
        )
        self.depth_queue = depth_node.depth.createOutputQueue(
            maxSize=self.queue_size,
            blocking=False,
        )
        self.detection_queue = spatial_det_net.out.createOutputQueue(
            maxSize=self.queue_size,
            blocking=False,
        )

    def _is_link_disconnect_error(self, exc: Exception) -> bool:
        message = str(exc)
        return any(
            token in message
            for token in (
                'X_LINK_ERROR',
                'Closed connection',
                'Communication exception',
                'MessageQueue was closed',
                'Node threw exception',
            )
        )

    def _restart_pipeline(self):
        now = time.monotonic()
        if now - self._last_reconnect_attempt < self.reconnect_cooldown_sec:
            return

        if (
            self.max_reconnect_attempts >= 0
            and self._reconnect_attempts >= self.max_reconnect_attempts
        ):
            if not self._reconnect_exhausted_logged:
                self._reconnect_exhausted_logged = True
                self.get_logger().error(
                    'Pipeline reconnect budget exhausted. '
                    f'Increase max_reconnect_attempts (current={self.max_reconnect_attempts}) '
                    'or inspect USB/power stability.'
                )
            return

        self._last_reconnect_attempt = now
        self._reconnect_attempts += 1
        self.get_logger().warn(
            f'Restarting DepthAI pipeline after link loss '
            f'(attempt {self._reconnect_attempts}/{self.max_reconnect_attempts}).'
        )

        try:
            if self.pipeline is not None and self.pipeline.isRunning():
                self.pipeline.stop()
        except Exception:
            pass

        self.rgb_queue = None
        self.depth_queue = None
        self.detection_queue = None
        self._last_rgb_image = None
        self._last_depth_image = None
        self._last_depth_preview_image = None

        try:
            self._build_and_start_pipeline()
        except Exception as restart_exc:
            self.get_logger().error(f'Pipeline restart failed: {restart_exc}')

    @staticmethod
    def _get_latest_message(queue):
        latest_msg = None
        while True:
            msg = queue.tryGet()
            if msg is None:
                break
            latest_msg = msg
        return latest_msg

    def timer_callback(self):
        try:
            if (
                self.rgb_queue is None
                or self.depth_queue is None
                or self.detection_queue is None
            ):
                return
            stamp = self.get_clock().now().to_msg()

            rgb_msg = self._get_latest_message(self.rgb_queue)
            if rgb_msg is not None:
                self._last_rgb_image = self.bridge.cv2_to_imgmsg(
                    rgb_msg.getCvFrame(), encoding='bgr8'
                )
            if self._last_rgb_image is not None:
                self._last_rgb_image.header.stamp = stamp
                self._last_rgb_image.header.frame_id = self.frame_id
                self.image_pub.publish(self._last_rgb_image)

            depth_msg = self._get_latest_message(self.depth_queue)
            if depth_msg is not None:
                depth_frame = depth_msg.getCvFrame()
                if depth_frame.ndim == 3 and depth_frame.shape[2] == 1:
                    depth_frame = depth_frame[:, :, 0]
                depth_frame = np.ascontiguousarray(depth_frame, dtype=np.uint16)
                depth_frame_color = _colorize_depth_frame(depth_frame)
                self._last_depth_image = self.bridge.cv2_to_imgmsg(
                    depth_frame, encoding='16UC1'
                )
                self._last_depth_preview_image = self.bridge.cv2_to_imgmsg(
                    depth_frame_color, encoding='bgr8'
                )
            if self._last_depth_image is not None:
                self._last_depth_image.header.stamp = stamp
                self._last_depth_image.header.frame_id = self.frame_id
                self.depth_pub.publish(self._last_depth_image)
            if self._last_depth_preview_image is not None:
                self._last_depth_preview_image.header.stamp = stamp
                self._last_depth_preview_image.header.frame_id = self.frame_id
                self.depth_preview_pub.publish(self._last_depth_preview_image)

            detections_msg = self._get_latest_message(self.detection_queue)
            if detections_msg is not None:
                self.detection_pub.publish(self._to_detection_array(detections_msg))
        except RuntimeError as exc:
            if self._is_link_disconnect_error(exc):
                self._restart_pipeline()
            else:
                self.get_logger().error(f'Pipeline runtime error: {exc}')
        except Exception as exc:
            if self._is_link_disconnect_error(exc):
                self._restart_pipeline()
            else:
                self.get_logger().error(f'Pipeline publish error: {exc}')

    def _to_detection_array(self, detections_msg):
        detection_array = SpatialDetectionArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = self.frame_id

        width = float(self.width)
        height = float(self.height)

        for det in detections_msg.detections:
            spatial_det = SpatialDetection()

            hyp = ObjectHypothesis()
            hyp.class_id = det.labelName
            hyp.score = float(det.confidence)
            spatial_det.results.append(hyp)

            bbox = BoundingBox2D()
            bbox.center.position.x = ((det.xmin + det.xmax) / 2.0) * width
            bbox.center.position.y = ((det.ymin + det.ymax) / 2.0) * height
            bbox.size_x = (det.xmax - det.xmin) * width
            bbox.size_y = (det.ymax - det.ymin) * height
            spatial_det.bbox = bbox

            pos = Point()
            pos.x = det.spatialCoordinates.x / 1000.0
            pos.y = det.spatialCoordinates.y / 1000.0
            pos.z = det.spatialCoordinates.z / 1000.0
            spatial_det.position = pos

            detection_array.detections.append(spatial_det)

        return detection_array

    def destroy_node(self):
        if self.pipeline is not None and self.pipeline.isRunning():
            self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpatialDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
