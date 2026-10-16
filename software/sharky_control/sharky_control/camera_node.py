#!/usr/bin/env python3

"""Demand-driven compressed camera publisher for SHARKY."""

import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


RUNTIME_DIRECTORY = Path('/run/sharky-camera')
LATEST_FRAME = RUNTIME_DIRECTORY / 'latest.jpg'
WEB_DEMAND = RUNTIME_DIRECTORY / 'web-active'


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('sharky_camera')
        self.declare_parameter('device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('jpeg_quality', 70)
        self.publisher = self.create_publisher(
            CompressedImage, '/camera/image/compressed', qos_profile_sensor_data
        )
        self.capture = None
        self.cv2 = None
        self.last_subscriber_time = None
        self.retry_after = 0.0
        fps = max(1.0, float(self.get_parameter('fps').value))
        self.timer = self.create_timer(1.0 / fps, self._tick)
        self.get_logger().info(
            'Camera publisher ready; capture starts only while subscribers exist'
        )

    def _open(self):
        import cv2
        device = int(self.get_parameter('device').value)
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter('width').value))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter('height').value))
        capture.set(cv2.CAP_PROP_FPS, float(self.get_parameter('fps').value))
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f'camera device {device} could not be opened')
        self.cv2 = cv2
        self.capture = capture
        self.get_logger().info('Camera capture started')

    def _close(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None
            self.cv2 = None
            self.get_logger().info('Camera capture stopped (no subscribers)')
        try:
            LATEST_FRAME.unlink()
        except FileNotFoundError:
            pass

    def _tick(self):
        now = time.monotonic()
        web_requested = False
        try:
            web_requested = time.time() - WEB_DEMAND.stat().st_mtime < 2.0
        except OSError:
            pass
        if self.publisher.get_subscription_count() == 0 and not web_requested:
            if self.last_subscriber_time is None:
                self.last_subscriber_time = now
            elif now - self.last_subscriber_time >= 5.0:
                self._close()
            return

        self.last_subscriber_time = now
        if self.capture is None:
            if now < self.retry_after:
                return
            try:
                self._open()
            except Exception as error:
                self.retry_after = now + 5.0
                self.get_logger().error(f'Camera open failed: {error}')
                return

        success, image = self.capture.read()
        if not success:
            self.get_logger().warning('Camera frame read failed; reopening')
            self._close()
            self.retry_after = now + 1.0
            return

        quality = int(self.get_parameter('jpeg_quality').value)
        success, encoded = self.cv2.imencode(
            '.jpg', image, [int(self.cv2.IMWRITE_JPEG_QUALITY), quality]
        )
        if not success:
            self.get_logger().warning('JPEG encoding failed')
            return

        message = CompressedImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'camera_optical_frame'
        message.format = 'jpeg'
        message.data = encoded.tobytes()
        self.publisher.publish(message)
        try:
            temporary = RUNTIME_DIRECTORY / 'latest.jpg.tmp'
            temporary.write_bytes(message.data)
            os.replace(temporary, LATEST_FRAME)
        except OSError as error:
            self.get_logger().warning(f'Could not update local web frame: {error}')

    def destroy_node(self):
        self._close()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraPublisher()
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
