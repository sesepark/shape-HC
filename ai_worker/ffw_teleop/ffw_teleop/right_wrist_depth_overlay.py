import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class RightWristDepthOverlay(Node):

    def __init__(self):
        super().__init__('right_wrist_depth_overlay')

        self.declare_parameter('depth_topic', '/camera_right/camera_right/depth/image_rect_raw')
        self.declare_parameter('overlay_topic', '/teleop/wrist_right/depth_overlay')
        self.declare_parameter('compressed_topic', '/teleop/wrist_right/depth_overlay/compressed')
        self.declare_parameter('center_distance_topic', '/teleop/wrist_right/center_distance_m')
        self.declare_parameter('publish_fps', 10.0)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_depth_m', 0.10)
        self.declare_parameter('max_depth_m', 1.50)
        self.declare_parameter('roi_size_px', 32)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('colormap', 'TURBO')

        self.depth_topic = self.get_parameter('depth_topic').value
        self.overlay_topic = self.get_parameter('overlay_topic').value
        self.compressed_topic = self.get_parameter('compressed_topic').value
        self.center_distance_topic = self.get_parameter('center_distance_topic').value
        self.publish_fps = max(float(self.get_parameter('publish_fps').value), 0.1)
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.roi_size_px = max(int(self.get_parameter('roi_size_px').value), 4)
        self.jpeg_quality = int(np.clip(int(self.get_parameter('jpeg_quality').value), 1, 100))
        self.colormap = self._resolve_colormap(str(self.get_parameter('colormap').value))
        self.publish_period_ns = int(1_000_000_000 / self.publish_fps)
        self.last_publish_time = None
        self.last_warn_time = None

        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._depth_callback, qos_profile_sensor_data)
        self.overlay_pub = self.create_publisher(
            Image, self.overlay_topic, qos_profile_sensor_data)
        self.compressed_pub = self.create_publisher(
            CompressedImage, self.compressed_topic, qos_profile_sensor_data)
        self.center_pub = self.create_publisher(
            Float32, self.center_distance_topic, qos_profile_sensor_data)

        self.get_logger().info(
            f'right wrist depth overlay: {self.depth_topic} -> {self.overlay_topic}, '
            f'{self.compressed_topic}')

    def _resolve_colormap(self, name):
        attr = f'COLORMAP_{name.strip().upper()}'
        return getattr(cv2, attr, cv2.COLORMAP_JET)

    def _warn_throttled(self, message):
        now = self.get_clock().now()
        if self.last_warn_time is None or (now - self.last_warn_time).nanoseconds > 5_000_000_000:
            self.get_logger().warn(message)
            self.last_warn_time = now

    def _depth_callback(self, msg):
        now = self.get_clock().now()
        if (self.last_publish_time is not None and
                (now - self.last_publish_time).nanoseconds < self.publish_period_ns):
            return
        self.last_publish_time = now

        depth_m = self._image_to_depth_meters(msg)
        if depth_m is None:
            return

        center_distance = self._center_distance(depth_m)
        overlay = self._make_overlay(depth_m, center_distance)

        self._publish_center_distance(center_distance)
        self._publish_overlay_image(msg, overlay)
        self._publish_compressed_image(msg, overlay)

    def _image_to_depth_meters(self, msg):
        encoding = msg.encoding.upper()
        if encoding in ('16UC1', 'MONO16'):
            dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
            scale = self.depth_scale
        elif encoding == '32FC1':
            dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
            scale = 1.0
        else:
            self._warn_throttled(f'unsupported depth encoding: {msg.encoding}')
            return None

        if msg.step <= 0 or msg.height <= 0 or msg.width <= 0:
            self._warn_throttled('received malformed depth image')
            return None

        stride = msg.step // dtype.itemsize
        expected = stride * msg.height
        raw = np.frombuffer(msg.data, dtype=dtype)
        if raw.size < expected:
            self._warn_throttled('depth image data is shorter than expected')
            return None

        image = raw[:expected].reshape((msg.height, stride))[:, :msg.width]
        return image.astype(np.float32, copy=False) * scale

    def _valid_mask(self, depth_m):
        return (
            np.isfinite(depth_m) &
            (depth_m >= self.min_depth_m) &
            (depth_m <= self.max_depth_m)
        )

    def _center_distance(self, depth_m):
        height, width = depth_m.shape[:2]
        half = self.roi_size_px // 2
        cx = width // 2
        cy = height // 2
        x0 = max(cx - half, 0)
        x1 = min(cx + half, width)
        y0 = max(cy - half, 0)
        y1 = min(cy + half, height)
        roi = depth_m[y0:y1, x0:x1]
        valid = self._valid_mask(roi)
        if not np.any(valid):
            return float('nan')
        return float(np.median(roi[valid]))

    def _make_overlay(self, depth_m, center_distance):
        valid = self._valid_mask(depth_m)
        clipped = np.clip(depth_m, self.min_depth_m, self.max_depth_m)
        normalized = np.zeros(depth_m.shape, dtype=np.uint8)
        scale = 255.0 / max(self.max_depth_m - self.min_depth_m, 1e-6)
        normalized[valid] = ((self.max_depth_m - clipped[valid]) * scale).astype(np.uint8)

        overlay = cv2.applyColorMap(normalized, self.colormap)
        overlay[~valid] = (20, 20, 20)

        height, width = depth_m.shape[:2]
        half = self.roi_size_px // 2
        cx = width // 2
        cy = height // 2
        cv2.rectangle(
            overlay,
            (max(cx - half, 0), max(cy - half, 0)),
            (min(cx + half, width - 1), min(cy + half, height - 1)),
            (255, 255, 255),
            1,
        )
        cv2.drawMarker(
            overlay, (cx, cy), (255, 255, 255), cv2.MARKER_CROSS, 18, 1)

        if math.isfinite(center_distance):
            center_text = f'CENTER {center_distance:.2f} m'
        else:
            center_text = 'CENTER -- m'
        range_text = f'{self.min_depth_m:.2f}-{self.max_depth_m:.2f} m'
        self._put_text(overlay, center_text, (10, 26), 0.70)
        self._put_text(overlay, range_text, (10, 50), 0.55)
        return np.ascontiguousarray(overlay)

    def _put_text(self, image, text, origin, scale):
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (255, 255, 255), 1, cv2.LINE_AA)

    def _publish_center_distance(self, center_distance):
        msg = Float32()
        msg.data = float(center_distance)
        self.center_pub.publish(msg)

    def _publish_overlay_image(self, source_msg, overlay):
        msg = Image()
        msg.header = source_msg.header
        msg.height = overlay.shape[0]
        msg.width = overlay.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = overlay.shape[1] * 3
        msg.data = overlay.tobytes()
        self.overlay_pub.publish(msg)

    def _publish_compressed_image(self, source_msg, overlay):
        ok, encoded = cv2.imencode(
            '.jpg', overlay, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self._warn_throttled('failed to JPEG-encode depth overlay')
            return
        msg = CompressedImage()
        msg.header = source_msg.header
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        self.compressed_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RightWristDepthOverlay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
