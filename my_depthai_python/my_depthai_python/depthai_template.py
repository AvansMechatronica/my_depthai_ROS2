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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthAIPublisher(Node):
    def __init__(self):
        super().__init__('depthai_publisher')

        self.declare_parameter('topic_name', 'camera/rgb')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 400)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('queue_size', 4)

        topic_name = self.get_parameter('topic_name').value
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        queue_size = int(self.get_parameter('queue_size').value)
        timer_period = 1.0 / fps if fps > 0.0 else 1.0 / 30.0
        
        # Create publisher for camera frames
        self.publisher_ = self.create_publisher(Image, topic_name, 10)
        
        # Initialize cv_bridge
        self.bridge = CvBridge()
        
        # Create DepthAI pipeline
        self.pipeline = dai.Pipeline()
        
        # DepthAI v3 pipeline: request host output directly from camera node
        self.cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        self.rgb_out = self.cam.requestOutput(size=(width, height), fps=fps)
        
        self.videoQueue = self.rgb_out.createOutputQueue(maxSize=queue_size, blocking=False)
        self.pipeline.start()
        
        # Create timer for frame publishing
        self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(
            f'DepthAI node started: topic={topic_name}, size={width}x{height}, fps={fps}'
        )
    
    def timer_callback(self):
        try:
            videoIn = self.videoQueue.tryGet()
            if videoIn is not None:
                assert isinstance(videoIn, dai.ImgFrame)
                cv_frame = videoIn.getCvFrame()
                
                # Convert and publish
                ros_image = self.bridge.cv2_to_imgmsg(cv_frame, encoding="bgr8")
                self.publisher_.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Frame publish error: {e}')
    
    def destroy_node(self):
        if self.pipeline.isRunning():
            self.pipeline.stop()
        super().destroy_node()


def main(args=None):
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
