#!/usr/bin/env python3

import time

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class PointCloudFromImagesNode(Node):
    def __init__(self):
        super().__init__('pointcloud_from_images')

        self.declare_parameter('depth_topic', 'stereo/depth')
        self.declare_parameter('rgb_topic', 'camera/rgb')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('pointcloud_topic', 'stereo/pointcloud')
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop_sec', 0.05)
        self.declare_parameter('publisher_queue_size', 10)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_depth_m', 0.2)
        self.declare_parameter('max_depth_m', 10.0)
        self.declare_parameter('pixel_step', 2)
        self.declare_parameter('resize_rgb_to_depth', True)
        self.declare_parameter('use_color', True)
        self.declare_parameter('frame_id', '')
        self.declare_parameter('fx', 0.0)
        self.declare_parameter('fy', 0.0)
        self.declare_parameter('cx', 0.0)
        self.declare_parameter('cy', 0.0)

        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.rgb_topic = str(self.get_parameter('rgb_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        pointcloud_topic = str(self.get_parameter('pointcloud_topic').value)
        sync_queue_size = int(self.get_parameter('sync_queue_size').value)
        sync_slop_sec = float(self.get_parameter('sync_slop_sec').value)
        publisher_queue_size = int(self.get_parameter('publisher_queue_size').value)

        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.pixel_step = max(1, int(self.get_parameter('pixel_step').value))
        self.resize_rgb_to_depth = bool(self.get_parameter('resize_rgb_to_depth').value)
        self.use_color = bool(self.get_parameter('use_color').value)
        self.frame_id_override = str(self.get_parameter('frame_id').value)

        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)

        self.bridge = CvBridge()
        self._last_intrinsics_frame_id = ''
        self._last_intrinsics_stamp = None
        self._last_intrinsics_warning = 0.0

        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            pointcloud_topic,
            publisher_queue_size,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_cb,
            10,
        )

        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_topic)
        self.rgb_sub = message_filters.Subscriber(self, Image, self.rgb_topic)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.rgb_sub],
            queue_size=sync_queue_size,
            slop=sync_slop_sec,
        )
        self.sync.registerCallback(self._synced_images_cb)

        self.get_logger().info(
            'Point cloud node started: '
            f'depth={self.depth_topic!r} rgb={self.rgb_topic!r} '
            f'camera_info={self.camera_info_topic!r} pointcloud={pointcloud_topic!r} '
            f'pixel_step={self.pixel_step} use_color={self.use_color!r}'
        )

    def _camera_info_cb(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self._last_intrinsics_frame_id = msg.header.frame_id
        self._last_intrinsics_stamp = msg.header.stamp

    def _synced_images_cb(self, depth_msg: Image, rgb_msg: Image):
        if not self._has_intrinsics():
            self._warn_missing_intrinsics()
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return

        depth_m = self._depth_to_meters(depth_image)
        if depth_m is None:
            self.get_logger().error(
                f'Unsupported depth encoding {depth_msg.encoding!r} with dtype {depth_image.dtype}'
            )
            return

        rgb = self._to_rgb8(rgb_image, rgb_msg.encoding)
        if rgb is None:
            self.get_logger().error(f'Unsupported RGB encoding: {rgb_msg.encoding!r}')
            return

        if rgb.shape[:2] != depth_m.shape[:2]:
            if not self.resize_rgb_to_depth:
                self.get_logger().error(
                    'RGB and depth image sizes differ and resize_rgb_to_depth is false: '
                    f'rgb={rgb.shape[:2]} depth={depth_m.shape[:2]}'
                )
                return
            rgb = cv2.resize(
                rgb,
                (depth_m.shape[1], depth_m.shape[0]),
                interpolation=cv2.INTER_LINEAR,
            )

        pointcloud_msg = self._build_pointcloud(depth_msg, rgb_msg, depth_m, rgb)
        if pointcloud_msg is not None:
            self.pointcloud_pub.publish(pointcloud_msg)

    def _has_intrinsics(self) -> bool:
        return self.fx > 0.0 and self.fy > 0.0

    def _warn_missing_intrinsics(self):
        now = time.monotonic()
        if now - self._last_intrinsics_warning < 5.0:
            return
        self._last_intrinsics_warning = now
        self.get_logger().warn(
            'No valid camera intrinsics available yet. Publish CameraInfo or set fx/fy/cx/cy parameters.'
        )

    def _depth_to_meters(self, depth_image: np.ndarray):
        if depth_image.dtype == np.uint16:
            return depth_image.astype(np.float32) * self.depth_scale
        if depth_image.dtype == np.float32:
            return depth_image.copy()
        if depth_image.dtype == np.float64:
            return depth_image.astype(np.float32)
        return None

    def _to_rgb8(self, image: np.ndarray, encoding: str):
        normalized = encoding.lower()
        if normalized == 'rgb8':
            return image
        if normalized == 'bgr8':
            return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if normalized in ('bgra8', 'rgba8'):
            code = cv2.COLOR_BGRA2RGB if normalized == 'bgra8' else cv2.COLOR_RGBA2RGB
            return cv2.cvtColor(image, code)
        if len(image.shape) == 2 or normalized in ('mono8', '8uc1'):
            return cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
        return None

    def _build_pointcloud(
        self,
        depth_msg: Image,
        rgb_msg: Image,
        depth_m: np.ndarray,
        rgb: np.ndarray,
    ):
        sampled_depth = depth_m[::self.pixel_step, ::self.pixel_step]
        sampled_rgb = rgb[::self.pixel_step, ::self.pixel_step]

        row_coords, col_coords = np.indices(sampled_depth.shape)
        row_coords = row_coords.astype(np.float32) * self.pixel_step
        col_coords = col_coords.astype(np.float32) * self.pixel_step

        valid = np.isfinite(sampled_depth)
        valid &= sampled_depth >= self.min_depth_m
        valid &= sampled_depth <= self.max_depth_m

        if not np.any(valid):
            return None

        z = sampled_depth[valid].astype(np.float32)
        u = col_coords[valid]
        v = row_coords[valid]

        x = ((u - self.cx) * z / self.fx).astype(np.float32)
        y = ((v - self.cy) * z / self.fy).astype(np.float32)

        if self.use_color:
            colors = sampled_rgb[valid].astype(np.uint32)
            packed_rgb = (
                (colors[:, 0] << 16)
                | (colors[:, 1] << 8)
                | colors[:, 2]
            )
            cloud_array = np.empty(
                z.shape[0],
                dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('rgb', np.uint32)],
            )
            cloud_array['x'] = x
            cloud_array['y'] = y
            cloud_array['z'] = z
            cloud_array['rgb'] = packed_rgb
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
            ]
        else:
            cloud_array = np.empty(
                z.shape[0],
                dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)],
            )
            cloud_array['x'] = x
            cloud_array['y'] = y
            cloud_array['z'] = z
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]

        pointcloud = PointCloud2()
        pointcloud.header.stamp = depth_msg.header.stamp
        pointcloud.header.frame_id = self._resolve_frame_id(depth_msg, rgb_msg)
        pointcloud.height = 1
        pointcloud.width = int(cloud_array.shape[0])
        pointcloud.fields = fields
        pointcloud.is_bigendian = False
        pointcloud.point_step = cloud_array.dtype.itemsize
        pointcloud.row_step = pointcloud.point_step * pointcloud.width
        pointcloud.is_dense = False
        pointcloud.data = cloud_array.tobytes()
        return pointcloud

    def _resolve_frame_id(self, depth_msg: Image, rgb_msg: Image) -> str:
        if self.frame_id_override:
            return self.frame_id_override
        if self._last_intrinsics_frame_id:
            return self._last_intrinsics_frame_id
        if depth_msg.header.frame_id:
            return depth_msg.header.frame_id
        return rgb_msg.header.frame_id


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFromImagesNode()
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