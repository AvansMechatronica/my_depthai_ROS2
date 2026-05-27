#!/usr/bin/env python3

import sys

try:
    import depthai as dai
except ImportError as exc:
    raise RuntimeError(
        'depthai is not installed for this Python interpreter: '
        f'{sys.executable}. Install it in the same environment used for colcon build.'
    ) from exc

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge


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


class DepthAIPublisher(Node):
    def __init__(self):
        """Initialiseer de ROS2-node en start een minimale DepthAI RGB-pipeline.

        Deze constructor doet alle opstartstappen:
        1) leest ROS-parameters in (topic, resolutie, fps, queue-grootte),
        2) maakt een ROS Image publisher,
        3) bouwt en start een DepthAI v3 pipeline,
        4) zet een timer op om frames periodiek op te halen en te publiceren.
        """
        super().__init__('depthai_publisher')

        # Parameters zijn bewust als launch/YAML-override beschikbaar,
        # zodat dezelfde code op meerdere camera-opstellingen bruikbaar blijft.
        self.declare_parameter('topic_name', 'camera/rgb')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 400)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('queue_size', 4)

        # Lees parameterwaarden en cast expliciet naar het gewenste type.
        # Dit voorkomt verrassingen als waarden als string uit launch/YAML komen.
        topic_name = self.get_parameter('topic_name').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        queue_size = int(self.get_parameter('queue_size').value)
        self.frame_id = 'oak_rgb_camera_optical_frame'

        # Timerfrequentie volgt de ingestelde fps.
        # Bij ongeldige fps <= 0 vallen we terug op 30 Hz.
        timer_period = 1.0 / fps if fps > 0.0 else 1.0 / 30.0
        
        # ROS publishers voor RGB-beelden en bijbehorende camera-intrinsics.
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, camera_info_topic, 10)
        
        # CvBridge converteert OpenCV-matrices naar sensor_msgs/Image.
        self.bridge = CvBridge()
        
        # Maak een nieuwe DepthAI pipeline-instantie.
        self.pipeline = dai.Pipeline()
        self.camera_info_template = None
        
        # DepthAI v3: maak een camera-node op CAM_A en vraag direct host-output.
        # De gevraagde size/fps bepalen wat de host ontvangt, los van eventuele
        # interne sensor-native resolutie.
        default_device = self.pipeline.getDefaultDevice()
        self.cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        self.rgb_out = self.cam.requestOutput(size=(width, height), fps=fps)
        try:
            calibration_handler = default_device.readCalibration()
            self.camera_info_template = _camera_info_from_calibration(
                calibration_handler,
                dai.CameraBoardSocket.CAM_A,
                width,
                height,
                self.frame_id,
            )
        except Exception as exc:
            self.get_logger().warn(f'Unable to load RGB calibration for CameraInfo: {exc}')
        
        # OutputQueue is non-blocking zodat de timerloop niet vastloopt.
        # Bij frame-achterstand worden oudere frames gedropt i.p.v. vertraging opbouwen.
        self.videoQueue = self.rgb_out.createOutputQueue(maxSize=queue_size, blocking=False)

        # Start de pipeline zodra alle nodes en queues zijn geconfigureerd.
        self.pipeline.start()
        
        # Timer triggert het ophalen/publiceren van frames op vaste frequentie.
        self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f'DepthAI node started: topic={topic_name}, camera_info={camera_info_topic}, '
            f'size={width}x{height}, fps={fps}'
        )
    
    def timer_callback(self):
        """Haal een frame op uit de DepthAI-queue en publiceer het als ROS Image.

        tryGet() is non-blocking: als er geen frame beschikbaar is, doet de
        callback niets. Zo blijft de node responsief, ook bij tijdelijke dips
        in cameradoorvoer.
        """
        try:
            videoIn = self.videoQueue.tryGet()
            if videoIn is not None:
                assert isinstance(videoIn, dai.ImgFrame)
                cv_frame = videoIn.getCvFrame()
                stamp = self.get_clock().now().to_msg()
                
                # Converteer OpenCV BGR-frame naar ROS Image en publiceer direct.
                ros_image = self.bridge.cv2_to_imgmsg(cv_frame, encoding="bgr8")
                ros_image.header.stamp = stamp
                ros_image.header.frame_id = self.frame_id
                self.publisher_.publish(ros_image)
                if self.camera_info_template is not None:
                    self.camera_info_pub.publish(
                        _copy_camera_info(self.camera_info_template, stamp, self.frame_id)
                    )
        except Exception as e:
            # Fouten in de callback loggen we, maar we laten de node doorlopen.
            # Dit voorkomt dat een incidentele conversie- of queuefout de hele
            # publisher stopt.
            self.get_logger().error(f'Frame publish error: {e}')
    
    def destroy_node(self):
        """Stop de DepthAI pipeline netjes voordat de ROS node wordt afgesloten."""
        # Controle op isRunning() voorkomt onnodige exceptions tijdens shutdown.
        if self.pipeline.isRunning():
            self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    """Entry point voor de executable.

    Initialiseer rclpy, start de node en zorg voor gecontroleerde shutdown
    bij Ctrl+C of andere stopcondities.
    """
    rclpy.init(args=args)
    depthai_publisher = DepthAIPublisher()
    try:
        rclpy.spin(depthai_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        depthai_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
