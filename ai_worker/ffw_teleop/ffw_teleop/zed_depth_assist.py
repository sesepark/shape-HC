import json
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ZedDepthAssist(Node):

    def __init__(self):
        super().__init__('zed_depth_assist')

        self.declare_parameter('depth_topic', '/zed/zed_node/depth/depth_registered')
        self.declare_parameter('base_image_topic', '/zed/zed_node/left/image_rect_color')
        self.declare_parameter('assist_topic', '/teleop/zed/depth_assist/compressed')
        self.declare_parameter('metrics_topic', '/teleop/zed/depth_metrics')
        self.declare_parameter('publish_fps', 10.0)
        self.declare_parameter('jpeg_quality', 75)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('min_depth_m', 0.15)
        self.declare_parameter('max_depth_m', 4.0)
        self.declare_parameter('center_roi_px', 56)
        self.declare_parameter('contour_near_depth_m', 1.20)
        self.declare_parameter('contour_min_area_px', 120.0)
        self.declare_parameter('base_image_timeout_s', 0.5)

        self.depth_topic = str(self.get_parameter('depth_topic').value).strip()
        self.base_image_topic = str(self.get_parameter('base_image_topic').value).strip()
        self.assist_topic = str(self.get_parameter('assist_topic').value).strip()
        self.metrics_topic = str(self.get_parameter('metrics_topic').value).strip()
        self.publish_fps = max(float(self.get_parameter('publish_fps').value), 0.1)
        self.jpeg_quality = int(np.clip(int(self.get_parameter('jpeg_quality').value), 1, 100))
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.center_roi_px = max(int(self.get_parameter('center_roi_px').value), 8)
        self.contour_near_depth_m = float(self.get_parameter('contour_near_depth_m').value)
        self.contour_min_area_px = max(
            float(self.get_parameter('contour_min_area_px').value), 0.0)
        self.base_image_timeout_s = max(
            float(self.get_parameter('base_image_timeout_s').value), 0.0)

        self.publish_period_ns = int(1_000_000_000 / self.publish_fps)
        self.last_publish_time = None
        self.last_warn_time = None
        self.latest_base_image = None
        self.latest_base_image_time = None
        self.latest_base_image_header = None

        self.depth_sub = self.create_subscription(
            Image, self.depth_topic, self._depth_callback, qos_profile_sensor_data)
        self.base_sub = self.create_subscription(
            Image, self.base_image_topic, self._base_image_callback, qos_profile_sensor_data)
        self.assist_pub = self.create_publisher(
            CompressedImage, self.assist_topic, qos_profile_sensor_data)
        self.metrics_pub = self.create_publisher(String, self.metrics_topic, 10)

        self.get_logger().info(
            f'ZED depth assist: depth={self.depth_topic}, base={self.base_image_topic} '
            f'-> {self.assist_topic} at {self.publish_fps:.1f} fps')

    def _warn_throttled(self, message):
        now = self.get_clock().now()
        if self.last_warn_time is None or (now - self.last_warn_time).nanoseconds > 5_000_000_000:
            self.get_logger().warn(message)
            self.last_warn_time = now

    def _base_image_callback(self, msg):
        image = self._image_to_bgr(msg)
        if image is None:
            return
        self.latest_base_image = image
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

        metrics, draw = self._depth_metrics(depth_m, msg.header.stamp)
        self._publish_metrics(metrics)

        if self.count_subscribers(self.assist_topic) <= 0:
            return

        base = self._get_base_image(depth_m.shape, now)
        image = self._make_assist_image(depth_m, metrics, draw, base)
        self._publish_jpeg(msg.header, image)

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
            self._warn_throttled(f'unsupported ZED base image encoding: {msg.encoding}')
            return None
        if msg.step <= 0 or msg.height <= 0 or msg.width <= 0:
            self._warn_throttled('received malformed ZED base image')
            return None

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.step * msg.height
        if raw.size < expected:
            self._warn_throttled('ZED base image data is shorter than expected')
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
            self._warn_throttled(f'unsupported ZED depth encoding: {msg.encoding}')
            return None

        if msg.step <= 0 or msg.height <= 0 or msg.width <= 0:
            self._warn_throttled('received malformed ZED depth image')
            return None

        stride = msg.step // dtype.itemsize
        expected = stride * msg.height
        raw = np.frombuffer(msg.data, dtype=dtype)
        if raw.size < expected:
            self._warn_throttled('ZED depth image data is shorter than expected')
            return None

        image = raw[:expected].reshape((msg.height, stride))[:, :msg.width]
        return image.astype(np.float32, copy=False) * scale

    def _valid_mask(self, depth_m):
        return (
            np.isfinite(depth_m) &
            (depth_m >= self.min_depth_m) &
            (depth_m <= self.max_depth_m)
        )

    def _median_depth_at(self, depth_m, cx, cy, size_px):
        height, width = depth_m.shape[:2]
        half = max(size_px // 2, 1)
        x0 = max(int(cx) - half, 0)
        x1 = min(int(cx) + half, width)
        y0 = max(int(cy) - half, 0)
        y1 = min(int(cy) + half, height)
        roi = depth_m[y0:y1, x0:x1]
        valid = self._valid_mask(roi)
        if not np.any(valid):
            return float('nan')
        return float(np.median(roi[valid]))

    def _clean_float(self, value, digits=4):
        if value is None:
            return None
        value = float(value)
        if not math.isfinite(value):
            return None
        return round(value, digits)

    def _stamp_sec(self, stamp):
        return float(stamp.sec + stamp.nanosec / 1e9)

    def _nearest_component(self, depth_m, valid):
        near_depth = self.contour_near_depth_m
        if near_depth <= 0.0:
            near_depth = self.min_depth_m + 0.35 * (self.max_depth_m - self.min_depth_m)
        near_depth = min(near_depth, self.max_depth_m)
        mask = (valid & (depth_m <= near_depth)).astype(np.uint8) * 255
        if not np.any(mask):
            return {}, {}

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.contour_min_area_px:
                continue
            contour_mask = np.zeros(depth_m.shape, dtype=np.uint8)
            cv2.drawContours(contour_mask, [contour], -1, 255, -1)
            contour_valid = (contour_mask > 0) & valid
            if not np.any(contour_valid):
                continue
            median_depth = float(np.median(depth_m[contour_valid]))
            candidates.append((median_depth, -area, contour, area, contour_valid))
        if not candidates:
            return {}, {}

        median_depth, _, contour, area, contour_valid = min(candidates, key=lambda item: item[:2])
        moments = cv2.moments(contour)
        if moments['m00'] > 1e-6:
            cx = float(moments['m10'] / moments['m00'])
            cy = float(moments['m01'] / moments['m00'])
        else:
            points = contour.reshape(-1, 2)
            cx = float(np.mean(points[:, 0]))
            cy = float(np.mean(points[:, 1]))

        component = {
            'valid': True,
            'cx_px': int(round(cx)),
            'cy_px': int(round(cy)),
            'area_px': int(area),
            'depth_m': self._clean_float(median_depth),
            'threshold_m': self._clean_float(near_depth),
        }
        draw = {
            'contour': contour,
            'center': (int(round(cx)), int(round(cy))),
            'valid_mask': contour_valid,
        }
        return component, draw

    def _depth_metrics(self, depth_m, stamp):
        height, width = depth_m.shape[:2]
        valid = self._valid_mask(depth_m)
        center_m = self._median_depth_at(depth_m, width // 2, height // 2, self.center_roi_px)
        left_proxy_m = self._median_depth_at(
            depth_m, int(width * 0.33), int(height * 0.78), self.center_roi_px)
        right_proxy_m = self._median_depth_at(
            depth_m, int(width * 0.67), int(height * 0.78), self.center_roi_px)
        nearest, draw = self._nearest_component(depth_m, valid)
        if nearest.get('valid'):
            nearest['offset_x_px'] = int(nearest['cx_px'] - width // 2)
            nearest['offset_y_px'] = int(nearest['cy_px'] - height // 2)

        proxy_values = [
            value for value in (left_proxy_m, right_proxy_m)
            if math.isfinite(value)
        ]
        proxy_m = min(proxy_values) if proxy_values else float('nan')
        relative_m = None
        if nearest.get('valid') and math.isfinite(proxy_m):
            relative_m = self._clean_float(float(nearest['depth_m']) - proxy_m)

        metrics = {
            'stamp_sec': self._clean_float(self._stamp_sec(stamp), 6),
            'center_m': self._clean_float(center_m),
            'nearest_valid': bool(nearest.get('valid', False)),
            'nearest_depth_m': nearest.get('depth_m') if nearest.get('valid') else None,
            'nearest_cx_px': nearest.get('cx_px'),
            'nearest_cy_px': nearest.get('cy_px'),
            'nearest_area_px': nearest.get('area_px'),
            'offset_x_px': nearest.get('offset_x_px'),
            'offset_y_px': nearest.get('offset_y_px'),
            'left_gripper_proxy_m': self._clean_float(left_proxy_m),
            'right_gripper_proxy_m': self._clean_float(right_proxy_m),
            'nearest_minus_gripper_proxy_m': relative_m,
        }
        return metrics, draw

    def _get_base_image(self, depth_shape, now):
        if self.latest_base_image is None or self.latest_base_image_time is None:
            return np.zeros((depth_shape[0], depth_shape[1], 3), dtype=np.uint8)
        age_s = (now - self.latest_base_image_time).nanoseconds / 1e9
        if self.base_image_timeout_s > 0.0 and age_s > self.base_image_timeout_s:
            return np.zeros((depth_shape[0], depth_shape[1], 3), dtype=np.uint8)

        height, width = depth_shape[:2]
        base = self.latest_base_image
        if base.shape[0] != height or base.shape[1] != width:
            base = cv2.resize(base, (width, height), interpolation=cv2.INTER_LINEAR)
        return np.ascontiguousarray(base)

    def _format_depth(self, value):
        if value is None:
            return '--'
        return f'{float(value):.2f}'

    def _make_assist_image(self, depth_m, metrics, draw, base_image):
        image = np.ascontiguousarray(base_image.copy())
        height, width = image.shape[:2]
        center = (width // 2, height // 2)
        roi_half = self.center_roi_px // 2

        center_box = (
            max(center[0] - roi_half, 0),
            max(center[1] - roi_half, 0),
            min(center[0] + roi_half, width - 1),
            min(center[1] + roi_half, height - 1),
        )
        cv2.rectangle(
            image,
            (center_box[0], center_box[1]),
            (center_box[2], center_box[3]),
            (255, 255, 255),
            1,
        )
        cv2.drawMarker(image, center, (255, 255, 255), cv2.MARKER_CROSS, 18, 1)

        contour = draw.get('contour')
        nearest_center = draw.get('center')
        if contour is not None:
            cv2.drawContours(image, [contour], -1, (0, 255, 255), 2, cv2.LINE_AA)
        if nearest_center is not None:
            cv2.circle(image, nearest_center, 6, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.line(image, center, nearest_center, (0, 255, 255), 1, cv2.LINE_AA)

        proxy_y = int(height * 0.78)
        proxy_points = [
            ('L', int(width * 0.33), proxy_y, metrics.get('left_gripper_proxy_m')),
            ('R', int(width * 0.67), proxy_y, metrics.get('right_gripper_proxy_m')),
        ]
        for label, x, y, depth in proxy_points:
            cv2.rectangle(
                image,
                (max(x - roi_half, 0), max(y - roi_half, 0)),
                (min(x + roi_half, width - 1), min(y + roi_half, height - 1)),
                (255, 0, 255),
                1,
            )
            self._put_text(image, f'{label} {self._format_depth(depth)}m', (x - roi_half, y - roi_half - 6), 0.42)

        cv2.rectangle(image, (0, 0), (width - 1, 66), (16, 18, 20), -1)
        cv2.rectangle(image, (0, 0), (width - 1, 66), (0, 255, 255), 1)
        self._put_text(
            image,
            f'ZED CENTER {self._format_depth(metrics.get("center_m"))}m '
            f'NEAR {self._format_depth(metrics.get("nearest_depth_m"))}m',
            (8, 25),
            0.56,
        )
        rel = metrics.get('nearest_minus_gripper_proxy_m')
        rel_text = '--' if rel is None else f'{float(rel):+.2f}m'
        self._put_text(
            image,
            f'OBJECT-GRIPPER {rel_text}  SPARSE DEPTH CUE',
            (8, 52),
            0.50,
        )
        return image

    def _put_text(self, image, text, origin, scale):
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale,
                    (255, 255, 255), 1, cv2.LINE_AA)

    def _publish_metrics(self, metrics):
        msg = String()
        msg.data = json.dumps(metrics, sort_keys=True)
        self.metrics_pub.publish(msg)

    def _publish_jpeg(self, header, image):
        ok, encoded = cv2.imencode(
            '.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            self._warn_throttled('failed to JPEG-encode ZED depth assist')
            return
        msg = CompressedImage()
        msg.header = header
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        self.assist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZedDepthAssist()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
