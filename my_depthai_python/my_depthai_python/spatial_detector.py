#!/usr/bin/env python3

import json
import sys
import time
import threading
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
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from vision_msgs.msg import BoundingBox2D, ObjectHypothesis
from depthai_ros_msgs.msg import SpatialDetection, SpatialDetectionArray

debug = False

def _get_resource_dir() -> Path:
    """Bepaal de map met modelbestanden en configuratiebestanden.

    Eerst wordt de ROS package-share locatie gebruikt. Als de package
    niet via de ROS index gevonden wordt (bijvoorbeeld tijdens lokale
    ontwikkeling), wordt teruggevallen op de resources-map relatief
    aan dit Python-bestand.
    """
    try:
        return Path(get_package_share_directory('my_depthai_python')) / 'resources'
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[1] / 'resources'


def _load_network_config(config_path: Path) -> dict:
    """Lees de netwerkconfiguratie (JSON) in vanaf schijf.

    De inhoud wordt als dictionary teruggegeven zodat modelparameters
    zoals input_size, labels en detectie-metadata later in de pipeline
    kunnen worden toegepast.
    """
    with config_path.open('r', encoding='utf-8') as handle:
        return json.load(handle)


def _camera_info_from_calibration(calibration_handler, socket, width, height, frame_id):
    intrinsics = calibration_handler.getCameraIntrinsics(socket, width, height)
    distortion = calibration_handler.getDistortionCoefficients(socket)

    fx = float(intrinsics[0][0])
    fy = float(intrinsics[1][1])
    cx = float(intrinsics[0][2])
    cy = float(intrinsics[1][2])

    camera_info = CameraInfo()
    camera_info.header.frame_id = frame_id
    camera_info.width = int(width)
    camera_info.height = int(height)
    camera_info.distortion_model = 'plumb_bob'
    camera_info.d = [float(value) for value in distortion]
    camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return camera_info


def _copy_camera_info(template, stamp, frame_id):
    camera_info = CameraInfo()
    camera_info.header.stamp = stamp
    camera_info.header.frame_id = frame_id
    camera_info.width = template.width
    camera_info.height = template.height
    camera_info.distortion_model = template.distortion_model
    camera_info.d = list(template.d)
    camera_info.k = list(template.k)
    camera_info.r = list(template.r)
    camera_info.p = list(template.p)
    return camera_info


class Publisher(dai.node.HostNode):
    def __init__(self):
        """Initialiseer de HostNode die pipeline-uitvoer naar ROS publiceert."""
        dai.node.HostNode.__init__(self)
        self.sendProcessingToPipeline(False)
        self.bridge = CvBridge()


    def build(
        self,
        depth: dai.Node.Output,
        detections: dai.Node.Output,
        rgb: dai.Node.Output,
        image_pub,
        depth_pub,
        detection_pub,
        camera_info_pub,
        camera_info_template,
        conversion_fn,
        show_bounding_boxes=True,
        publish_images=True,
        depth_raw_pub=None,
        on_activity=None,
        make_stamp=None,
        frame_id='',
    ):
        """Configureer de hostnode met ingangen, ROS publishers en callbacklogica.

        De volgorde in link_args moet exact overeenkomen met de argumenten
        van process(...), zodat DepthAI frames correct doorgeeft.
        """
        self.image_pub = image_pub
        self.depth_pub = depth_pub
        self.detection_pub = detection_pub
        self.camera_info_pub = camera_info_pub
        self.camera_info_template = camera_info_template
        self.conversion_fn = conversion_fn
        self.show_bounding_boxes = show_bounding_boxes
        self.publish_images = publish_images
        self.depth_raw_pub = depth_raw_pub
        self.on_activity = on_activity
        self.make_stamp = make_stamp
        self.frame_id = frame_id

        self.link_args(depth, detections, rgb) # Must match the inputs to the process method

    def process(self, depthPreview, detections, rgbPreview):
        """Verwerk een inkomende batch uit de pipeline en publiceer resultaten.

        Depth en RGB frames worden naar OpenCV-conforme afbeeldingen
        omgezet, waarna visualisatie en ROS-publicatie in een centrale
        routine plaatsvinden.
        """
        #print("Publisher.process called with new data batch", flush=True)
        try:
            depthPreview = depthPreview.getCvFrame()
            rgbPreview = rgbPreview.getCvFrame()
            depthFrameColor = self.processDepthFrame(depthPreview)
            # Pass both colorized (depthFrameColor) and raw (depthPreview) depth frames
            self.publishResults(rgbPreview, depthFrameColor, depthPreview, detections.detections)
        except Exception as e:
            print(f"Error in Publisher.process: {e}", flush=True)

    def processDepthFrame(self, depthFrame):
        """Normaliseer een diepteframe en kleur het voor visualisatie.

        Door percentielen te gebruiken in plaats van absolute min/max
        wordt het contrast stabieler bij ruis en uitschieters.
        """
        depthDownscaled = depthFrame[::4]
        if np.all(depthDownscaled == 0):
            minDepth = 0
        else:
            minDepth = np.percentile(depthDownscaled[depthDownscaled != 0], 1)
        maxDepth = np.percentile(depthDownscaled, 99)
        depthFrameColor = np.interp(depthFrame, (minDepth, maxDepth), (0, 255)).astype(np.uint8)
        return cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

    def publishResults(self, rgbFrame, depthFrameColor, depthFrame, detections):
        """Teken detecties, publiceer ROS-berichten en werk activiteitsstatus bij.

        In degraded mode kunnen beeldtopics worden overgeslagen, terwijl
        detecties wel gepubliceerd blijven voor downstream nodes.
        
        depthFrame: raw depth data (uint16 millimeters) for metric depth
        depthFrameColor: colorized depth visualization (BGR8)
        """
        height, width, _ = rgbFrame.shape
        if self.publish_images and self.show_bounding_boxes:
            for detection in detections:
                self.drawBoundingBoxes(depthFrameColor, detection)
                self.drawDetections(rgbFrame, detection, width, height)

        try:
            if self.on_activity is not None:
                self.on_activity()
            stamp = self.make_stamp() if self.make_stamp is not None else None
            # In degraded mode we keep detections alive and skip image topics.
            if self.publish_images:
                image_msg = self.bridge.cv2_to_imgmsg(rgbFrame, "bgr8")
                depth_msg = self.bridge.cv2_to_imgmsg(depthFrameColor, "bgr8")
                if stamp is not None:
                    image_msg.header.stamp = stamp
                    depth_msg.header.stamp = stamp
                if self.frame_id:
                    image_msg.header.frame_id = self.frame_id
                    depth_msg.header.frame_id = self.frame_id
                self.image_pub.publish(image_msg)
                self.depth_pub.publish(depth_msg)
                # Publish raw metric depth if available
                if self.depth_raw_pub is not None and depthFrame is not None:
                    try:
                        # Ensure depthFrame is uint16 (millimeters)
                        if depthFrame.dtype != np.uint16:
                            depth_raw = depthFrame.astype(np.uint16)
                        else:
                            depth_raw = depthFrame
                        depth_raw_msg = self.bridge.cv2_to_imgmsg(depth_raw, "mono16")
                        if stamp is not None:
                            depth_raw_msg.header.stamp = stamp
                        if self.frame_id:
                            depth_raw_msg.header.frame_id = self.frame_id
                        self.depth_raw_pub.publish(depth_raw_msg)
                    except Exception as e:
                        print(f"Error publishing raw depth: {e}", flush=True)
                if (
                    self.camera_info_pub is not None
                    and self.camera_info_template is not None
                    and stamp is not None
                ):
                    self.camera_info_pub.publish(
                        _copy_camera_info(self.camera_info_template, stamp, self.frame_id)
                    )
            self.detection_pub.publish(self.conversion_fn(detections))
            if debug and self.publish_images:
                cv2.imshow("Depth frame", depthFrameColor)
                cv2.imshow("Color frame", rgbFrame)
                if cv2.waitKey(1) == ord('q'):
                    self.stopPipeline()
        except Exception as e:
            print(f"Error publishing results: {e}", flush=True)

    def drawBoundingBoxes(self, depthFrameColor, detection):
        """Teken de ROI van de depth-mapping voor een detectie op het dieptebeeld."""
        roiData = detection.boundingBoxMapping
        roi = roiData.roi
        roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
        topLeft = roi.topLeft()
        bottomRight = roi.bottomRight()
        cv2.rectangle(depthFrameColor, (int(topLeft.x), int(topLeft.y)), (int(bottomRight.x), int(bottomRight.y)), (255, 255, 255), 1)

    def drawDetections(self, frame, detection, frameWidth, frameHeight):
        """Teken label, confidence en XYZ-coordinaten op het RGB-frame."""
        x1 = int(detection.xmin * frameWidth)
        x2 = int(detection.xmax * frameWidth)
        y1 = int(detection.ymin * frameHeight)
        y2 = int(detection.ymax * frameHeight)
        label = detection.labelName
        color = (255, 255, 255)
        cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(frame, "{:.2f}".format(detection.confidence * 100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)


class SpatialDetectorNode(Node):
    def __init__(self):
        """Initialiseer ROS-parameters, laad modelconfig en start de pipeline.

        Deze constructor verzamelt runtime-instellingen, bouwt de
        pipeline_config op en activeert direct de eerste pipeline-run.
        """
        super().__init__('spatial_detector')

        self.declare_parameter('image_topic', 'camera/rgb')
        self.declare_parameter('depth_topic', 'stereo/depth')
        self.declare_parameter('depth_raw_topic', 'stereo/depth_raw')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
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
        self.declare_parameter('show_bounding_boxes', True)
        self.declare_parameter('publish_images', True)
        self.declare_parameter('auto_degrade_on_reconnect', True)
        self.declare_parameter('reconnect_degrade_threshold', 3)
        self.declare_parameter('degraded_fps', 5.0)
        self.declare_parameter('data_stall_timeout_sec', 5.0)

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
        show_bounding_boxes = bool(self.get_parameter('show_bounding_boxes').value)
        publish_images = bool(self.get_parameter('publish_images').value)
        auto_degrade_on_reconnect = bool(
            self.get_parameter('auto_degrade_on_reconnect').value
        )
        reconnect_degrade_threshold = int(
            self.get_parameter('reconnect_degrade_threshold').value
        )
        degraded_fps = float(self.get_parameter('degraded_fps').value)
        self.data_stall_timeout_sec = float(
            self.get_parameter('data_stall_timeout_sec').value
        )
        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        depth_raw_topic = self.get_parameter('depth_raw_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        self.frame_id = 'oak_rgb_camera_optical_frame'

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
        nn_width, nn_height = [int(value) for value in input_size.split('x', maxsplit=1)]
        self.width = nn_width
        self.height = nn_height

        self.image_pub = self.create_publisher(Image, image_topic, 10)
        self.depth_pub = self.create_publisher(Image, depth_topic, 10)
        self.depth_raw_pub = self.create_publisher(Image, depth_raw_topic, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)
        self.detection_pub = self.create_publisher(
            SpatialDetectionArray, detections_topic, 10
        )
        self.camera_info_template = None

        self.pipeline = None
        self._reconnect_attempts = 0
        self._last_reconnect_attempt = 0.0
        self._reconnect_exhausted_logged = False
        self._pipeline_thread = None
        self._pipeline_running = False
        self._degraded_mode = False
        self._last_data_monotonic = time.monotonic()
        self._restart_lock = threading.Lock()
        self._watchdog_timer = self.create_timer(1.0, self._watchdog_cb)

        self.pipeline_config = {
            'fps': fps,
            'pipeline_mode': pipeline_mode,
            'nn_width': nn_width,
            'nn_height': nn_height,
            'depth_width': depth_width,
            'depth_height': depth_height,
            'depth_source': depth_source,
            'stereo_extended_disparity': stereo_extended_disparity,
            'blob_path': blob_path,
            'metadata': metadata,
            'labels': labels,
            'blob_name': blob_name,
            'config_name': config_name,
            'show_bounding_boxes': show_bounding_boxes,
            'publish_images': publish_images,
            'auto_degrade_on_reconnect': auto_degrade_on_reconnect,
            'reconnect_degrade_threshold': reconnect_degrade_threshold,
            'degraded_fps': degraded_fps,
            'image_topic': image_topic,
            'depth_topic': depth_topic,
            'depth_raw_topic': depth_raw_topic,
            'camera_info_topic': camera_info_topic,
            'detections_topic': detections_topic,
        }

        self._build_and_start_pipeline()
 
    def _build_and_start_pipeline(self):
        """Bouw de volledige DepthAI pipeline en start de achtergrondthread.

        Deze methode koppelt camera's, dieptebron en detectienetwerk,
        configureert parserinstellingen en verbindt de Publisher-hostnode.
        """
        cfg = self.pipeline_config

        self.pipeline = dai.Pipeline()

        # Define sources and outputs
        default_device = self.pipeline.getDefaultDevice()
        self.platform = default_device.getPlatform()
        try:
            calibration_handler = default_device.readCalibration()
            self.camera_info_template = _camera_info_from_calibration(
                calibration_handler,
                dai.CameraBoardSocket.CAM_A,
                self.width,
                self.height,
                self.frame_id,
            )
        except Exception as exc:
            self.camera_info_template = None
            self.get_logger().warn(f'Unable to load RGB calibration for CameraInfo: {exc}')

        # Define sources and outputs


        size = (416, 416)

        camRgb = self.pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A,
            sensorFps=cfg['fps'],
        )

        monoLeft = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=cfg['fps'])
        monoRight = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=cfg['fps'])
        if cfg['depth_source'] == 'stereo':
            depthSource = self.pipeline.create(dai.node.StereoDepth)
            depthSource.setExtendedDisparity(cfg['stereo_extended_disparity'])
            monoLeft.requestOutput(size).link(depthSource.left)
            monoRight.requestOutput(size).link(depthSource.right)
        elif cfg['depth_source'] == "neural":
            depthSource = self.pipeline.create(dai.node.NeuralDepth).build(
                monoLeft.requestFullResolutionOutput(),
                monoRight.requestFullResolutionOutput(),
                dai.DeviceModelZoo.NEURAL_DEPTH_LARGE,
            )
        else:
            self.get_logger().fatal(f"Unknown depth_source: {cfg['depth_source']!r}")
            raise ValueError(f"Invalid depth_source: {cfg['depth_source']}")

        test_w_yolo_v6_nano = False # Set to True to test with yolov6-nano, which has fixed input size in the blob and different output format (no detectionParser)
        if test_w_yolo_v6_nano:
            modelDescription = dai.NNModelDescription("yolov6-nano")
            spatial_det_net = self.pipeline.create(dai.node.SpatialDetectionNetwork).build(
                camRgb, depthSource, modelDescription)
            spatial_det_net.spatialLocationCalculator.initialConfig.setSegmentationPassthrough(False)
            spatial_det_net.input.setBlocking(False)
            spatial_det_net.setDepthLowerThreshold(100)
            spatial_det_net.setDepthUpperThreshold(5000)

        else:

            spatial_det_net = self.pipeline.create(dai.node.SpatialDetectionNetwork)

            # Uitleg van deze koppeling:
            # 1) nn_size moet exact overeenkomen met het model-inputformaat uit de JSON
            #    (bijv. 416x416). Als de camera minder of andere bytes levert dan verwacht,
            #    krijg je runtime-fouten zoals:
            #    "Input tensor ... exceeds available data range" en dan wordt inferentie
            #    overgeslagen.
            # 2) camRgb.requestOutput(nn_size) dwingt de RGB-uitvoer naar het formaat dat
            #    de Neural Network input daadwerkelijk verwacht.
            # 3) depthSource.depth -> spatial_det_net.inputDepth is noodzakelijk voor de
            #    3D-berekening (X, Y, Z). Zonder deze link heb je alleen 2D-detecties.
            # 4) De depth-resolutie en alignment-instellingen bepalen of spatial mapping
            #    stabiel blijft. Mismatch tussen RGB/diepte transformaties kan leiden tot
            #    waarschuwingen over niet-uitgelijnde transformationData.

            nn_size = (cfg['nn_width'], cfg['nn_height'])
            frame_type = (
                dai.ImgFrame.Type.BGR888i
                if self.platform == dai.Platform.RVC4
                else dai.ImgFrame.Type.BGR888p
            )

            camRgb.requestOutput(nn_size, type=frame_type).link(spatial_det_net.input)
            #camRgb.requestOutput(nn_size).link(spatial_det_net.input)          
            depthSource.depth.link(spatial_det_net.inputDepth)

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
            spatial_det_net.setSpatialCalculationAlgorithm(dai.SpatialLocationCalculatorAlgorithm.MIN)

        self.publisher = self.pipeline.create(Publisher)
            
        self.publisher.build(
            spatial_det_net.passthroughDepth,
            spatial_det_net.out,
            spatial_det_net.passthrough,
            self.image_pub,
            self.depth_pub,
            self.detection_pub,
            self.camera_info_pub,
            self.camera_info_template,
            self._to_detection_array,
            show_bounding_boxes=cfg['show_bounding_boxes'],
            publish_images=cfg['publish_images'],
            depth_raw_pub=self.depth_raw_pub,
            on_activity=self._mark_pipeline_activity,
            make_stamp=self.get_clock().now().to_msg,
            frame_id=self.frame_id,
        )

        print("Starting pipeline with depth source: ", cfg['depth_source'])
        self._last_data_monotonic = time.monotonic()
        self._pipeline_running = True
        
        # Run pipeline in background thread
        self._pipeline_thread = threading.Thread(target=self._run_pipeline_loop, daemon=True)
        self._pipeline_thread.start()

        self._reconnect_attempts = 0
        self._reconnect_exhausted_logged = False
        self.get_logger().info(
            f"SpatialDetector started - mode={cfg['pipeline_mode']!r} blob={cfg['blob_name']!r} "
            f"config={cfg['config_name']!r} depth={cfg['depth_source']!r} "
            f"extended_disparity={cfg['stereo_extended_disparity']!r} "
            f"rgb_size={self.width}x{self.height} fps={cfg['fps']} "
            f"publish_images={cfg['publish_images']!r} degraded_mode={self._degraded_mode!r} "
            f"stall_timeout={self.data_stall_timeout_sec}s "
            f"image->{cfg['image_topic']!r} depth->{cfg['depth_topic']!r} depth_raw->{cfg['depth_raw_topic']!r} "
            f"camera_info->{cfg['camera_info_topic']!r} "
            f"detections->{cfg['detections_topic']!r}"
        )

    def _mark_pipeline_activity(self):
        """Markeer het tijdstip van laatst ontvangen pipeline-data.

        De watchdog gebruikt deze timestamp om vastlopers of datastilstand
        te detecteren en automatisch een restart te triggeren.
        """
        self._last_data_monotonic = time.monotonic()

    def _watchdog_cb(self):
        """Controleer periodiek of de pipeline nog data produceert.

        Als langer dan data_stall_timeout_sec geen activiteit is gemeten,
        wordt een gecontroleerde restart uitgevoerd.
        """
        if not self._pipeline_running:
            return
        elapsed = time.monotonic() - self._last_data_monotonic
        if elapsed <= self.data_stall_timeout_sec:
            return
        self.get_logger().warn(
            f'No pipeline data for {elapsed:.1f}s; restarting pipeline.'
        )
        self._restart_pipeline()

    def _run_pipeline_loop(self):
        """Voer pipeline.run() uit in een aparte thread met foutafhandeling.

        Bij onverwachte stop of bekende linkfouten wordt een herstart
        geprobeerd, zonder direct de volledige ROS node te stoppen.
        """
        current_thread = threading.current_thread()
        try:
            self.get_logger().info("Pipeline thread started, calling pipeline.run()")
            self.pipeline.run()
            self.get_logger().info("Pipeline thread finished normally")

            # If run() exits while the node is still active, attempt a controlled restart.
            if self._pipeline_running:
                self.get_logger().warn(
                    "Pipeline stopped while node is active; attempting restart."
                )
                self._restart_pipeline()
        except Exception as e:
            import traceback
            self.get_logger().error(f"Pipeline loop error: {e}\n{traceback.format_exc()}")

            if self._is_link_disconnect_error(e):
                self.get_logger().warn(
                    "Detected X_LINK disconnect; attempting pipeline restart."
                )
                self._restart_pipeline()
        finally:
            # Don't clobber running state if a newer pipeline thread has already started.
            if self._pipeline_thread is current_thread:
                self._pipeline_running = False
            self.get_logger().info("Pipeline thread stopped")



    def _is_link_disconnect_error(self, exc: Exception) -> bool:
        """Herken fouten die wijzen op USB/X_LINK communicatieproblemen."""
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
        """Voer een veilige pipeline-restart uit met cooldown en limieten.

        De methode voorkomt parallelle restarts, respecteert een maximaal
        aantal pogingen en kan in degraded mode schakelen bij herhaalde
        reconnects.
        """
        if not self._restart_lock.acquire(blocking=False):
            return

        current_thread = threading.current_thread()
        try:
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

            if (
                self.pipeline_config['auto_degrade_on_reconnect']
                and not self._degraded_mode
                and self._reconnect_attempts
                >= self.pipeline_config['reconnect_degrade_threshold']
            ):
                self._degraded_mode = True
                self.pipeline_config['publish_images'] = False
                self.pipeline_config['show_bounding_boxes'] = False
                self.pipeline_config['fps'] = self.pipeline_config['degraded_fps']
                self.pipeline_config['depth_width'] = min(
                    self.pipeline_config['depth_width'],
                    400,
                )
                self.pipeline_config['depth_height'] = min(
                    self.pipeline_config['depth_height'],
                    300,
                )
                self.get_logger().warn(
                    'Enabling degraded mode after repeated reconnects: '
                    f"fps={self.pipeline_config['fps']} depth={self.pipeline_config['depth_width']}x{self.pipeline_config['depth_height']} publish_images=False."
                )

            self.get_logger().warn(
                f'Restarting DepthAI pipeline after link loss '
                f'(attempt {self._reconnect_attempts}/{self.max_reconnect_attempts}).'
            )

            try:
                self._pipeline_running = False
                if self.pipeline is not None and self.pipeline.isRunning():
                    self.pipeline.stop()
                if (
                    self._pipeline_thread is not None
                    and self._pipeline_thread is not current_thread
                ):
                    self._pipeline_thread.join(timeout=2.0)
            except Exception:
                pass

            try:
                self._build_and_start_pipeline()
            except Exception as restart_exc:
                self.get_logger().error(f'Pipeline restart failed: {restart_exc}')
        finally:
            self._restart_lock.release()


    def _to_detection_array(self, detections_msg):
        """Converteer DepthAI-detecties naar depthai_ros_msgs berichttype.

        Bounding boxes worden van genormaliseerde naar pixelcoordinaten
        omgerekend en XYZ-posities worden van millimeter naar meter gezet.
        """
        detection_array = SpatialDetectionArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = self.frame_id

        width = float(self.width)
        height = float(self.height)

        for det in detections_msg:
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
        """Stop pipeline/thread netjes voordat de ROS node wordt afgebroken."""
        self._pipeline_running = False
        if self.pipeline is not None and self.pipeline.isRunning():
            self.pipeline.stop()
        if self._pipeline_thread is not None:
            self._pipeline_thread.join(timeout=2.0)
        super().destroy_node()


def main():
    """Entry point: initialiseer ROS, start node en verzorg nette shutdown."""
    rclpy.init()
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
