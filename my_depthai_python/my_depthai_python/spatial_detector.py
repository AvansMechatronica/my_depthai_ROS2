#!/usr/bin/env python3

import json
import sys
from pathlib import Path

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


class SpatialDetectorNode(Node):
    def __init__(self):
        super().__init__('spatial_detector')

        self.declare_parameter('image_topic', 'camera/rgb')
        self.declare_parameter('detections_topic', 'spatial_detections')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 400)
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('queue_size', 4)
        self.declare_parameter('depth_source', 'stereo')
        self.declare_parameter('blob_name', 'SimpleFruitsYoloV5.blob')
        self.declare_parameter('config_name', 'SimpleFruitsYoloV5.json')

        depth_width = int(self.get_parameter('width').value)
        depth_height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        queue_size = int(self.get_parameter('queue_size').value)
        depth_source = self.get_parameter('depth_source').value
        blob_name = self.get_parameter('blob_name').value
        config_name = self.get_parameter('config_name').value
        image_topic = self.get_parameter('image_topic').value
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
        self.detection_pub = self.create_publisher(
            SpatialDetectionArray, detections_topic, 10
        )

        self.pipeline = dai.Pipeline()

        color_cam = self.pipeline.create(dai.node.ColorCamera)
        color_cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        color_cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        color_cam.setPreviewSize(rgb_width, rgb_height)
        color_cam.setVideoSize(rgb_width, rgb_height)
        color_cam.setInterleaved(False)
        color_cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        color_cam.setPreviewKeepAspectRatio(False)
        color_cam.setFps(fps)

        mono_resolution = _get_mono_resolution(depth_width, depth_height)
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_left.setResolution(mono_resolution)
        mono_left.setFps(fps)

        mono_right = self.pipeline.create(dai.node.MonoCamera)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        mono_right.setResolution(mono_resolution)
        mono_right.setFps(fps)

        if depth_source == 'stereo':
            depth_node = self.pipeline.create(dai.node.StereoDepth)
            depth_node.setExtendedDisparity(True)
            depth_node.setDepthAlign(dai.CameraBoardSocket.CAM_A)
            mono_left.out.link(depth_node.left)
            mono_right.out.link(depth_node.right)
        elif depth_source == 'neural':
            depth_node = self.pipeline.create(dai.node.NeuralDepth).build(
                mono_left.out,
                mono_right.out,
                dai.DeviceModelZoo.NEURAL_DEPTH_LARGE,
            )
        else:
            self.get_logger().fatal(f'Unknown depth_source: {depth_source!r}')
            raise ValueError(f'Invalid depth_source: {depth_source}')

        spatial_det_net = self.pipeline.create(dai.node.SpatialDetectionNetwork)
        spatial_det_net.setBlobPath(blob_path)
        spatial_det_net.input.setBlocking(False)
        spatial_det_net.setConfidenceThreshold(float(metadata['confidence_threshold']))
        spatial_det_net.setBoundingBoxScaleFactor(0.5)
        spatial_det_net.setDepthLowerThreshold(100)
        spatial_det_net.setDepthUpperThreshold(5000)
        spatial_det_net.detectionParser.setNumClasses(int(metadata['classes']))
        spatial_det_net.detectionParser.setCoordinateSize(int(metadata['coordinates']))
        spatial_det_net.detectionParser.setAnchors(
            [float(anchor) for anchor in metadata.get('anchors', [])]
        )
        spatial_det_net.detectionParser.setAnchorMasks(
            {
                name: [int(index) for index in indices]
                for name, indices in metadata.get('anchor_masks', {}).items()
            }
        )
        spatial_det_net.detectionParser.setIouThreshold(float(metadata['iou_threshold']))
        if labels:
            spatial_det_net.detectionParser.setClasses(labels)
        spatial_det_net.spatialLocationCalculator.initialConfig.setSegmentationPassthrough(False)

        # Use ImageManip to ensure proper format conversion
        manip = self.pipeline.create(dai.node.ImageManip)
        manip.initialConfig.setOutputSize(rgb_width, rgb_height)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)
        manip.setMaxOutputFrameSize(rgb_width * rgb_height * 3)

        color_cam.preview.link(manip.inputImage)
        manip.out.link(spatial_det_net.input)
        depth_node.depth.link(spatial_det_net.inputDepth)

        # Create output queues - from manip (resized RGB) and network detections
        self.rgb_queue = manip.out.createOutputQueue(
            maxSize=queue_size,
            blocking=False,
        )
        self.detection_queue = spatial_det_net.out.createOutputQueue(
            maxSize=queue_size,
            blocking=False,
        )

        # Start the pipeline
        self.pipeline.start()
        self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f'SpatialDetector started — blob={blob_name!r} config={config_name!r} '
            f'depth={depth_source!r} rgb_size={self.width}x{self.height} fps={fps} '
            f'image→{image_topic!r} detections→{detections_topic!r}'
        )

    def timer_callback(self):
        try:
            rgb_msg = self.rgb_queue.tryGet()
            if rgb_msg is not None:
                ros_image = self.bridge.cv2_to_imgmsg(
                    rgb_msg.getCvFrame(), encoding='bgr8'
                )
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = self.frame_id
                self.image_pub.publish(ros_image)

            detections_msg = self.detection_queue.tryGet()
            if detections_msg is not None:
                self.detection_pub.publish(self._to_detection_array(detections_msg))
        except Exception as exc:
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
        if self.pipeline.isRunning():
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
