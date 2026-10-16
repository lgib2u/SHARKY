#!/usr/bin/env python3

"""Socket-activated, low-memory fallback controller for SHARKY."""

import collections
import json
import mimetypes
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros2_pca9685.srv import SetPwm
from sensor_msgs.msg import CompressedImage

from sharky_control.gait_profile import clamp_speed_scale


IDLE_TIMEOUT_SECONDS = int(os.environ.get('SHARKY_WEB_IDLE_TIMEOUT', '600'))
PACKAGE_SHARE = Path(get_package_share_directory('sharky_control'))
STATIC_ROOT = (PACKAGE_SHARE / 'static').resolve()
LOG_BUFFER = collections.deque(maxlen=100)
LEVEL_NAMES = {10: 'DBG', 20: 'INF', 30: 'WRN', 40: 'ERR', 50: 'FTL'}
CAMERA_RUNTIME_DIRECTORY = Path('/run/sharky-camera')
CAMERA_LATEST_FRAME = CAMERA_RUNTIME_DIRECTORY / 'latest.jpg'
CAMERA_WEB_DEMAND = CAMERA_RUNTIME_DIRECTORY / 'web-active'
LOCAL_COMMAND_SOCKET = Path.home() / '.cache' / 'sharky' / 'motion.sock'

# Hardware-facing ranges spanned by the existing gait controller, including
# its right-side 180-angle flip.  Exact test angles must stay inside these
# per-channel envelopes.
SERVO_TEST_RANGES = {
    0: (15.0, 110.0),
    1: (75.0, 178.0),
    2: (70.0, 165.0),
    3: (2.0, 105.0),
    4: (30.0, 110.0),
    5: (75.0, 175.0),
    6: (70.0, 150.0),
    7: (5.0, 105.0),
}


def add_log(message):
    LOG_BUFFER.append(f'[{time.strftime("%H:%M:%S")}] {message}')


class EdgeWebNode(Node):
    def __init__(self):
        super().__init__('edge_web_interface')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Log, '/rosout', self._rosout_callback, 100)
        self.servo_client = self.create_client(SetPwm, '/pca9685/set_pwm')
        self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self._camera_callback,
            qos_profile_sensor_data,
        )
        self.last_cmd = 'none'
        self.estopped = False
        self.camera_lock = threading.Lock()
        self.camera_frame = None
        self.camera_frame_time = 0.0

    def _camera_callback(self, message):
        with self.camera_lock:
            self.camera_frame = bytes(message.data)
            self.camera_frame_time = time.monotonic()

    def latest_camera_frame(self, maximum_age=2.0):
        with self.camera_lock:
            if time.monotonic() - self.camera_frame_time > maximum_age:
                return None
            return self.camera_frame

    def _rosout_callback(self, message):
        if message.level <= 10:
            return
        if 'pca9685' in message.name and message.level < 30:
            return
        level = LEVEL_NAMES.get(message.level, '???')
        add_log(f'[{message.name}] {level}: {message.msg}')

    def send_command(self, direction, speed=None):
        message = Twist()
        if direction == 'speed':
            message.linear.z = clamp_speed_scale(speed)
        elif direction == 'stop':
            pass
        elif direction == 'disarm':
            message.angular.x = -1.0
            self.estopped = True
        elif direction == 'arm':
            message.angular.x = 1.0
            self.estopped = False
        elif self.estopped:
            raise PermissionError('controller is emergency-stopped; reset and arm first')
        elif direction == 'neutral':
            message.angular.y = 1.0
        elif direction == 'tilt_up':
            message.angular.y = 2.0
        elif direction == 'tilt_down':
            message.angular.y = -2.0
        elif direction == 'forward':
            message.linear.x = 1.0
        elif direction == 'backward':
            message.linear.x = -1.0
        elif direction == 'turn_left':
            message.angular.z = 1.0
        elif direction == 'turn_right':
            message.angular.z = -1.0
        else:
            raise ValueError('unknown command')
        self.last_cmd = direction
        payload = (
            direction if speed is None
            else f'{direction}:{message.linear.z if direction == "speed" else speed}'
        )
        local_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            local_socket.sendto(payload.encode('ascii'), str(LOCAL_COMMAND_SOCKET))
            add_log(f'CMD: {direction} (local)')
        except OSError:
            # Keep /cmd_vel as a fallback and as the public ROS control API.
            self.cmd_pub.publish(message)
            add_log(f'CMD: {direction} (ROS fallback)')
        finally:
            local_socket.close()

    def test_servo(self, channel, angle):
        if self.estopped:
            raise PermissionError('controller is emergency-stopped')
        if channel not in SERVO_TEST_RANGES:
            raise ValueError('invalid servo channel')
        minimum, maximum = SERVO_TEST_RANGES[channel]
        if not minimum <= angle <= maximum:
            raise ValueError(
                f'angle is outside channel {channel} gait envelope '
                f'({minimum:.0f}-{maximum:.0f} degrees)'
            )
        if not self.servo_client.service_is_ready():
            return False, 'PCA9685 service unavailable'

        request = SetPwm.Request()
        request.channel_num = channel
        request.target_position = angle
        future = self.servo_client.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done():
            return False, 'servo command timed out'

        result = future.result()
        if result is None:
            return False, 'servo service returned no response'
        if not result.is_success:
            return False, result.response or 'servo command rejected'

        add_log(f'GAIT-SAFE SERVO TEST: ch={channel} angle={angle:.0f}deg')
        return True, result.response or 'servo command accepted'

class CameraManager:
    """Lazily owns the camera so the normal controller pays no camera cost."""

    def __init__(self):
        self.lock = threading.Lock()
        self.capture = None
        self.cv2 = None
        self.clients = 0

    def acquire(self):
        with self.lock:
            if self.capture is None:
                import cv2
                capture = cv2.VideoCapture(0, cv2.CAP_V4L2)
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                capture.set(cv2.CAP_PROP_FPS, 10)
                capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if not capture.isOpened():
                    capture.release()
                    raise RuntimeError('camera /dev/video0 could not be opened')
                self.cv2 = cv2
                self.capture = capture
                add_log('Camera started at 640x480, target 10 fps')
            self.clients += 1

    def frame(self):
        with self.lock:
            if self.capture is None:
                return None
            success, image = self.capture.read()
            if not success:
                return None
            success, encoded = self.cv2.imencode(
                '.jpg', image, [int(self.cv2.IMWRITE_JPEG_QUALITY), 70]
            )
            return encoded.tobytes() if success else None

    def release(self):
        with self.lock:
            self.clients = max(0, self.clients - 1)
            if self.clients == 0:
                self._close_locked()

    def close(self):
        with self.lock:
            self.clients = 0
            self._close_locked()

    def _close_locked(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None
            self.cv2 = None
            add_log('Camera stopped')


CAMERA = CameraManager()


def _run(command):
    try:
        return subprocess.run(
            command, capture_output=True, text=True, timeout=2, check=False
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ''


def _read_cpu_percent(server):
    try:
        fields = [int(value) for value in Path('/proc/stat').read_text().splitlines()[0].split()[1:]]
        idle = fields[3] + fields[4]
        total = sum(fields)
        previous = server.last_cpu_sample
        server.last_cpu_sample = (idle, total)
        if previous is None or total == previous[1]:
            return None
        return round(100.0 * (1.0 - (idle - previous[0]) / (total - previous[1])), 1)
    except (OSError, ValueError, IndexError):
        return None


def _read_memory_percent():
    try:
        values = {}
        for line in Path('/proc/meminfo').read_text().splitlines():
            key, value = line.split(':', 1)
            values[key] = int(value.strip().split()[0])
        return round(100.0 * (1.0 - values['MemAvailable'] / values['MemTotal']), 1)
    except (OSError, ValueError, KeyError):
        return None


def _power_status():
    raw = _run(['vcgencmd', 'get_throttled'])
    try:
        flags = int(raw.split('=', 1)[1], 16)
    except (IndexError, ValueError):
        return {'status': 'Unknown', 'charge': 'N/A', 'level': 'secondary', 'flags': raw}
    if flags & 0x1:
        state, level = 'UNDERVOLTAGE', 'danger'
    elif flags & 0x10000:
        state, level = 'PAST UNDERVOLTAGE', 'warning'
    else:
        state, level = 'OK', 'success'
    return {'status': state, 'charge': f'0x{flags:x}', 'level': level, 'flags': flags}


def _system_stats(server):
    temperature = None
    raw_temp = _run(['vcgencmd', 'measure_temp'])
    try:
        temperature = round(float(raw_temp.split('=', 1)[1].split("'", 1)[0]), 1)
    except (IndexError, ValueError):
        pass
    disk = shutil.disk_usage('/')
    return {
        'cpu': _read_cpu_percent(server),
        'ram': _read_memory_percent(),
        'disk': round(100.0 * disk.used / disk.total, 1),
        'temp': temperature,
        'servo_load': [],
        'last_cmd': server.ros_node.last_cmd,
        'battery': _power_status(),
        'estopped': server.ros_node.estopped,
    }


class SharkyHandler(BaseHTTPRequestHandler):
    server_version = 'SHARKY-Edge/1.0'

    def log_message(self, _format, *_args):
        return

    def _json(self, payload, status=200):
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _file(self, path, content_type=None):
        try:
            body = path.read_bytes()
        except OSError:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header('Content-Type', content_type or mimetypes.guess_type(path.name)[0] or 'application/octet-stream')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'public, max-age=3600' if '/static/' in self.path else 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _camera_stream(self):
        # Ask the ROS camera service to capture and share its newest JPEG in RAM.
        def request_and_read_shared_frame():
            try:
                CAMERA_WEB_DEMAND.touch(exist_ok=True)
                if time.time() - CAMERA_LATEST_FRAME.stat().st_mtime <= 2.0:
                    return CAMERA_LATEST_FRAME.read_bytes()
            except OSError:
                pass
            return self.server.ros_node.latest_camera_frame()

        deadline = time.monotonic() + 5.0
        frame = request_and_read_shared_frame()
        while frame is None and time.monotonic() < deadline:
            time.sleep(0.1)
            frame = request_and_read_shared_frame()
        use_shared_camera = frame is not None
        if not use_shared_camera:
            CAMERA.acquire()
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        self.send_header('Connection', 'close')
        self.end_headers()
        failures = 0
        try:
            while True:
                frame = request_and_read_shared_frame() if use_shared_camera else CAMERA.frame()
                if frame is None:
                    failures += 1
                    if failures >= 10:
                        raise RuntimeError('camera stopped producing frames')
                    time.sleep(0.1)
                    continue
                failures = 0
                self.server.touch()
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                self.wfile.write(frame)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
                time.sleep(0.1)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except RuntimeError as error:
            add_log(f'CAMERA ERROR: {error}')
        finally:
            if not use_shared_camera:
                CAMERA.release()

    def do_GET(self):
        self.server.touch()
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        try:
            if parsed.path == '/':
                self._file(PACKAGE_SHARE / 'templates' / 'dashboard.html', 'text/html; charset=utf-8')
            elif parsed.path.startswith('/static/'):
                relative = Path(unquote(parsed.path[len('/static/'):]))
                # Colcon --symlink-install keeps static files as symlinks back
                # into src/.  Validate the URL lexically so those trusted
                # package symlinks remain usable without allowing traversal.
                if relative.is_absolute() or '..' in relative.parts:
                    self.send_error(404)
                else:
                    target = STATIC_ROOT / relative
                    if not target.is_file():
                        self.send_error(404)
                    else:
                        self._file(target)
            elif parsed.path == '/cmd':
                move = query.get('move', [None])[0]
                value = query.get('value', [None])[0]
                if move is None:
                    self._json({'status': 'error', 'msg': 'missing move'}, 400)
                else:
                    self.server.ros_node.send_command(move, value)
                    self._json({'status': 'ok'})
            elif parsed.path == '/stats':
                self._json(_system_stats(self.server))
            elif parsed.path == '/logs':
                self._json(list(LOG_BUFFER))
            elif parsed.path == '/camera_stream':
                self._camera_stream()
            elif parsed.path == '/servo_test':
                channel = int(query.get('channel', ['-1'])[0])
                angle = float(query.get('angle', ['nan'])[0])
                success, message = self.server.ros_node.test_servo(channel, angle)
                self._json(
                    {
                        'status': 'ok' if success else 'rejected',
                        'msg': message,
                    },
                    200 if success else 409,
                )
            elif parsed.path == '/kill_motors':
                self.server.ros_node.send_command('stop')
                self.server.ros_node.send_command('disarm')
                add_log('EMERGENCY STOP: motion stopped and controller disarmed')
                self._json({'status': 'stopped'})
            elif parsed.path == '/restart_motors':
                self.server.ros_node.estopped = False
                add_log('CONTROL RESET: re-arm before moving')
                self._json({'status': 'ready'})
            elif parsed.path == '/restart_status':
                self._json({'state': 'ready'})
            else:
                self.send_error(404)
        except PermissionError as error:
            self._json({'status': 'blocked', 'msg': str(error)}, 423)
        except (TypeError, ValueError) as error:
            self._json({'status': 'error', 'msg': str(error)}, 400)
        except RuntimeError as error:
            add_log(f'CAMERA ERROR: {error}')
            if not self.wfile.closed:
                try:
                    self._json({'status': 'unavailable', 'msg': str(error)}, 503)
                except (BrokenPipeError, ConnectionResetError):
                    pass
        except BrokenPipeError:
            pass


class SharkyHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, ros_node):
        bind_host = os.environ.get('SHARKY_BIND_HOST', '127.0.0.1')
        inherited = int(os.environ.get('LISTEN_FDS', '0')) == 1
        if inherited:
            super().__init__((bind_host, 5000), SharkyHandler, bind_and_activate=False)
            self.socket.close()
            self.socket = socket.fromfd(3, socket.AF_INET, socket.SOCK_STREAM)
            self.server_address = self.socket.getsockname()
        else:
            super().__init__((bind_host, 5000), SharkyHandler)
        self.ros_node = ros_node
        self.last_activity = time.monotonic()
        self.last_cpu_sample = None

    def touch(self):
        self.last_activity = time.monotonic()


def _idle_monitor(server):
    while True:
        time.sleep(15)
        if time.monotonic() - server.last_activity >= IDLE_TIMEOUT_SECONDS:
            add_log('Idle timeout reached; returning control to systemd socket')
            server.shutdown()
            return


def _ros_spin(node):
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass


def main():
    rclpy.init()
    node = EdgeWebNode()
    spin_thread = threading.Thread(target=_ros_spin, args=(node,), daemon=True)
    spin_thread.start()
    server = SharkyHTTPServer(node)
    def request_shutdown(_signal_number, _frame):
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, request_shutdown)
    signal.signal(signal.SIGINT, request_shutdown)
    add_log(f'Lightweight controller started; idle timeout {IDLE_TIMEOUT_SECONDS}s')
    threading.Thread(target=_idle_monitor, args=(server,), daemon=True).start()
    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        try:
            node.send_command('stop')
            node.send_command('disarm')
        except Exception:
            pass
        CAMERA.close()
        server.server_close()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2)
        node.destroy_node()


if __name__ == '__main__':
    main()
