import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class WristDepthOverlay(Node):

    def __init__(self):
        super().__init__('wrist_depth_overlay')

        self.declare_parameter('depth_topic', '/camera_right/camera_right/depth/image_rect_raw')
        self.declare_parameter('base_image_topic', '/camera_right/camera_right/color/image_raw')
        self.declare_parameter('overlay_topic', '/teleop/wrist_right/depth_overlay')
        self.declare_parameter('compressed_topic', '/teleop/wrist_right/depth_overlay/compressed')
        self.declare_parameter('base_compressed_topic', '/teleop/wrist_right/color/compressed')
        self.declare_parameter('center_distance_topic', '/teleop/wrist_right/center_distance_m')
        self.declare_parameter('publish_raw_overlay', False)
        self.declare_parameter('publish_base_compressed', True)
        self.declare_parameter('publish_fps', 10.0)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_depth_m', 0.10)
        self.declare_parameter('max_depth_m', 1.50)
        self.declare_parameter('roi_size_px', 32)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('colormap', 'TURBO')
        self.declare_parameter('depth_colormap', '')
        self.declare_parameter('base_alpha', 0.70)
        self.declare_parameter('depth_alpha', 0.30)
        self.declare_parameter('base_image_timeout_s', 0.5)
        self.declare_parameter('show_depth_contours', True)
        self.declare_parameter('contour_near_depth_m', 0.55)
        self.declare_parameter('contour_min_area_px', 30.0)
        self.declare_parameter('invalid_depth_mode', 'base_only')

        self.depth_topic = self.get_parameter('depth_topic').value
        self.base_image_topic = self.get_parameter('base_image_topic').value
        self.overlay_topic = self.get_parameter('overlay_topic').value
        self.compressed_topic = self.get_parameter('compressed_topic').value
        self.base_compressed_topic = self.get_parameter('base_compressed_topic').value
        self.center_distance_topic = self.get_parameter('center_distance_topic').value
        self.publish_raw_overlay = self._as_bool(
            self.get_parameter('publish_raw_overlay').value)
        self.publish_base_compressed = self._as_bool(
            self.get_parameter('publish_base_compressed').value)
        self.publish_fps = max(float(self.get_parameter('publish_fps').value), 0.1)
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.roi_size_px = max(int(self.get_parameter('roi_size_px').value), 4)
        self.jpeg_quality = int(np.clip(int(self.get_parameter('jpeg_quality').value), 1, 100))
        depth_colormap = str(self.get_parameter('depth_colormap').value).strip()
        if not depth_colormap:
            depth_colormap = str(self.get_parameter('colormap').value)
        self.colormap = self._resolve_colormap(depth_colormap)
        self.base_alpha = max(float(self.get_parameter('base_alpha').value), 0.0)
        self.depth_alpha = max(float(self.get_parameter('depth_alpha').value), 0.0)
        self.base_image_timeout_s = max(
            float(self.get_parameter('base_image_timeout_s').value), 0.0)
        self.show_depth_contours = self._as_bool(
            self.get_parameter('show_depth_contours').value)
        self.contour_near_depth_m = float(self.get_parameter('contour_near_depth_m').value)
        self.contour_min_area_px = max(
            float(self.get_parameter('contour_min_area_px').value), 0.0)
        self.invalid_depth_mode = str(self.get_parameter('invalid_depth_mode').value)
        self.publish_period_ns = int(1_000_000_000 / self.publish_fps)
        self.last_publish_time = None
        self.last_warn_time = None
        self.latest_base_image = None
        self.latest_base_image_time = None
        self.latest_base_image_header = None

        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._depth_callback, qos_profile_sensor_data)
        self.base_image_sub = None
        if self.base_image_topic:
            self.base_image_sub = self.create_subscription(
                Image, self.base_image_topic, self._base_image_callback, qos_profile_sensor_data)
        self.overlay_pub = None
        if self.publish_raw_overlay:
            self.overlay_pub = self.create_publisher(
                Image, self.overlay_topic, qos_profile_sensor_data)
        self.compressed_pub = self.create_publisher(
            CompressedImage, self.compressed_topic, qos_profile_sensor_data)
        self.base_compressed_pub = None
        if self.publish_base_compressed and self.base_compressed_topic:
            self.base_compressed_pub = self.create_publisher(
                CompressedImage, self.base_compressed_topic, qos_profile_sensor_data)
        self.center_pub = self.create_publisher(
            Float32, self.center_distance_topic, qos_profile_sensor_data)

        self.get_logger().info(
            f'wrist depth overlay: depth={self.depth_topic}, base={self.base_image_topic or "none"} '
            f'-> raw={self.overlay_topic if self.publish_raw_overlay else "disabled"}, '
            f'compressed={self.compressed_topic}, '
            f'base_compressed={self.base_compressed_topic if self.base_compressed_pub else "disabled"}')

    def _resolve_colormap(self, name):
        attr = f'COLORMAP_{name.strip().upper()}'
        return getattr(cv2, attr, cv2.COLORMAP_JET)

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _warn_throttled(self, message):
        now = self.get_clock().now()
        if self.last_warn_time is None or (now - self.last_warn_time).nanoseconds > 5_000_000_000:
            self.get_logger().warn(message)
            self.last_warn_time = now

    def _has_subscribers(self, topic):
        return bool(topic) and self.count_subscribers(topic) > 0

    def _base_image_callback(self, msg):
        base_image = self._image_to_bgr(msg)
        if base_image is None:
            return
        self.latest_base_image = base_image
        self.latest_base_image_time = self.get_clock().now()
        self.latest_base_image_header = msg.header

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
        self._publish_center_distance(center_distance)

        needs_raw_overlay = self.publish_raw_overlay and self._has_subscribers(self.overlay_topic)
        needs_compressed_overlay = self._has_subscribers(self.compressed_topic)
        needs_base_compressed = (
            self.base_compressed_pub is not None and
            self._has_subscribers(self.base_compressed_topic) and
            self._has_fresh_base_image(now)
        )
        if not (needs_raw_overlay or needs_compressed_overlay or needs_base_compressed):
            return

        base_image = self._get_base_image(depth_m.shape, now)

        if needs_base_compressed:
            self._publish_base_compressed_image(msg, base_image)

        if not (needs_raw_overlay or needs_compressed_overlay):
            return

        overlay = self._make_overlay(depth_m, center_distance, base_image)
        if needs_raw_overlay:
            self._publish_overlay_image(msg, overlay)
        if needs_compressed_overlay:
            self._publish_compressed_image(msg, overlay)

    def _image_to_bgr(self, msg):
        encoding = msg.encoding.upper()
        channels_by_encoding = {
            'BGR8': 3,
            'RGB8': 3,
            'BGRA8': 4,
            'RGBA8': 4,
            'MONO8': 1,
            '8UC1': 1,
            'YUYV': 2,
            'YUY2': 2,
            'UYVY': 2,
        }
        channels = channels_by_encoding.get(encoding)
        if channels is None:
            self._warn_throttled(f'unsupported base image encoding: {msg.encoding}')
            return None
        if msg.step <= 0 or msg.height <= 0 or msg.width <= 0:
            self._warn_throttled('received malformed base image')
            return None

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.step * msg.height
        if raw.size < expected:
            self._warn_throttled('base image data is shorter than expected')
            return None

        rows = raw[:expected].reshape((msg.height, msg.step))
        image = rows[:, :msg.width * channels].reshape((msg.height, msg.width, channels))

        if encoding == 'BGR8':
            return np.ascontiguousarray(image)
        if encoding == 'RGB8':
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding == 'BGRA8':
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        if encoding == 'RGBA8':
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        if encoding in ('MONO8', '8UC1'):
            return cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2BGR)
        if encoding in ('YUYV', 'YUY2'):
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)
        if encoding == 'UYVY':
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)
        return None

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

    def _get_base_image(self, depth_shape, now):
        if self.latest_base_image is None or self.latest_base_image_time is None:
            return self._depth_grayscale_base(depth_shape)
        age_s = (now - self.latest_base_image_time).nanoseconds / 1e9
        if self.base_image_timeout_s > 0.0 and age_s > self.base_image_timeout_s:
            return self._depth_grayscale_base(depth_shape)

        height, width = depth_shape[:2]
        base = self.latest_base_image
        if base.shape[0] != height or base.shape[1] != width:
            base = cv2.resize(base, (width, height), interpolation=cv2.INTER_LINEAR)
        return np.ascontiguousarray(base)

    def _has_fresh_base_image(self, now):
        if self.latest_base_image is None or self.latest_base_image_time is None:
            return False
        age_s = (now - self.latest_base_image_time).nanoseconds / 1e9
        return self.base_image_timeout_s <= 0.0 or age_s <= self.base_image_timeout_s

    def _depth_grayscale_base(self, depth_shape):
        return np.zeros((depth_shape[0], depth_shape[1], 3), dtype=np.uint8)

    def _make_overlay(self, depth_m, center_distance, base_image):
        valid = self._valid_mask(depth_m)
        clipped = np.clip(depth_m, self.min_depth_m, self.max_depth_m)
        normalized = np.zeros(depth_m.shape, dtype=np.uint8)
        scale = 255.0 / max(self.max_depth_m - self.min_depth_m, 1e-6)
        normalized[valid] = ((self.max_depth_m - clipped[valid]) * scale).astype(np.uint8)

        depth_color = cv2.applyColorMap(normalized, self.colormap)
        fallback_base = base_image
        if not np.any(fallback_base):
            gray = np.zeros(depth_m.shape, dtype=np.uint8)
            gray[valid] = (255 - normalized[valid] // 2).astype(np.uint8)
            fallback_base = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        overlay = cv2.addWeighted(
            fallback_base, self.base_alpha, depth_color, self.depth_alpha, 0.0)
        if self.invalid_depth_mode.strip().lower() == 'base_only':
            overlay[~valid] = fallback_base[~valid]
        else:
            overlay[~valid] = (20, 20, 20)

        if self.show_depth_contours:
            self._draw_depth_contours(overlay, depth_m, valid)

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

    def _draw_depth_contours(self, overlay, depth_m, valid):
        near_depth = self.contour_near_depth_m
        if near_depth <= 0.0:
            near_depth = self.min_depth_m + 0.35 * (self.max_depth_m - self.min_depth_m)
        near_mask = (valid & (depth_m <= near_depth)).astype(np.uint8) * 255
        if not np.any(near_mask):
            return
        kernel = np.ones((3, 3), np.uint8)
        near_mask = cv2.morphologyEx(near_mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(near_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [
            contour for contour in contours
            if cv2.contourArea(contour) >= self.contour_min_area_px
        ]
        if contours:
            cv2.drawContours(overlay, contours, -1, (255, 255, 255), 1, cv2.LINE_AA)

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
        if self.overlay_pub is None:
            return
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
        self._publish_jpeg(
            self.compressed_pub, source_msg.header, overlay, 'failed to JPEG-encode depth overlay')

    def _publish_base_compressed_image(self, source_msg, image):
        header = self.latest_base_image_header if self.latest_base_image_header else source_msg.header
        self._publish_jpeg(
            self.base_compressed_pub, header, image, 'failed to JPEG-encode base image')

    def _publish_jpeg(self, publisher, header, image, warn_message):
        if publisher is None:
            return
        ok, encoded = cv2.imencode(
            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self._warn_throttled(warn_message)
            return
        msg = CompressedImage()
        msg.header = header
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WristDepthOverlay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
