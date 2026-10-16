#!/usr/bin/env python3

import threading
import psutil
import time
import subprocess
import collections
import cv2

from flask import Flask, render_template, jsonify, request, Response

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from rcl_interfaces.msg import Log
from ros2_pca9685.srv import SetPwm

import os
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('sharky_control')

app = Flask(
    __name__,
    template_folder=os.path.join(pkg_share, 'templates'),
    static_folder=os.path.join(pkg_share, 'static')
)

# Rolling log buffer — last 100 entries
log_buffer = collections.deque(maxlen=100)

def log(msg):
    ts = time.strftime('%H:%M:%S')
    entry = f"[{ts}] {msg}"
    log_buffer.append(entry)


#################################################
# ROS NODE
#################################################

_LEVEL = {10: 'DBG', 20: 'INF', 30: 'WRN', 40: 'ERR', 50: 'FTL'}

class WebNode(Node):

    def __init__(self):
        super().__init__('web_interface')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(
            Float32MultiArray,
            '/servo_load',
            self.load_callback,
            10
        )

        # Capture all node logs into the terminal panel.
        # Filter: skip DEBUG; skip pca9685 "Actuate servo" spam.
        self.create_subscription(Log, '/rosout', self.rosout_callback, 100)

        self.servo_load = []
        self.last_cmd = 'none'
        self.servo_client = self.create_client(SetPwm, '/pca9685/set_pwm')

    def load_callback(self, msg):
        self.servo_load = list(msg.data)

    def rosout_callback(self, msg):
        if msg.level <= 10:                          # skip DEBUG
            return
        if 'pca9685' in msg.name and msg.level < 30: # skip pca9685 INFO spam
            return
        level = _LEVEL.get(msg.level, '???')
        log(f"[{msg.name}] {level}: {msg.msg}")

    def send_cmd(self, direction):
        msg = Twist()
        self.last_cmd = direction

        if direction == "forward":
            msg.linear.x = 1.0
        elif direction == "backward":
            msg.linear.x = -1.0
        elif direction == "turn_left":
            msg.angular.z = 1.0
        elif direction == "turn_right":
            msg.angular.z = -1.0
        elif direction == "arm":
            msg.angular.x = 1.0
        elif direction == "disarm":
            msg.angular.x = -1.0
        elif direction == "stop":
            pass  # zero Twist = stop

        log(f"CMD: {direction}")
        self.cmd_pub.publish(msg)


    def test_servo(self, channel, angle):
        if not self.servo_client.service_is_ready():
            log('SERVO TEST: PCA9685 not ready')
            return False
        req = SetPwm.Request()
        req.channel_num = int(channel)
        req.target_position = float(angle)
        event = threading.Event()
        future = self.servo_client.call_async(req)
        future.add_done_callback(lambda f: event.set())
        event.wait(timeout=1.0)
        log(f'SERVO TEST: ch={channel} angle={angle:.0f}deg')
        return True


#################################################
# SYSTEM STATS
#################################################

def get_battery_status():
    try:
        result = subprocess.run(
            ['vcgencmd', 'get_throttled'],
            capture_output=True, text=True, timeout=2
        )
        raw = result.stdout.strip().split('=')[-1]
        throttled = int(raw, 16)

        if throttled & 0x1:
            return {'status': 'CRITICAL', 'charge': '<5%', 'level': 'danger'}
        elif throttled & 0x10000:
            return {'status': 'WARNING', 'charge': '~5-15%', 'level': 'warning'}
        else:
            return {'status': 'OK', 'charge': '>25%', 'level': 'success'}
    except Exception:
        return {'status': 'Unknown', 'charge': 'N/A', 'level': 'secondary'}


def get_system_stats(node):
    cpu  = psutil.cpu_percent()
    ram  = psutil.virtual_memory().percent
    disk = psutil.disk_usage('/').percent

    temp = None
    try:
        temp = psutil.sensors_temperatures()['cpu_thermal'][0].current
    except Exception:
        pass

    battery = get_battery_status()

    return {
        'cpu':        cpu,
        'ram':        ram,
        'disk':       disk,
        'temp':       temp,
        'servo_load': node.servo_load,
        'last_cmd':   node.last_cmd,
        'battery':    battery,
    }


#################################################
# CAMERA STREAM
#################################################

cap = cv2.VideoCapture(0)

def gen_frames():
    while True:
        success, frame = cap.read()
        if not success:
            continue
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


#################################################
# FLASK ROUTES
#################################################

node_handle = None

@app.route('/')
def index():
    return render_template('dashboard.html')


@app.route('/cmd')
def cmd():
    move  = request.args.get('move')
    value = request.args.get('value')

    if move == 'speed' and value:
        msg = Twist()
        msg.linear.z = float(value)
        node_handle.cmd_pub.publish(msg)
        log(f"SPEED: {value}")
    elif move:
        node_handle.send_cmd(move)

    return jsonify({'status': 'ok'})


@app.route('/stats')
def stats():
    return jsonify(get_system_stats(node_handle))


@app.route('/logs')
def logs():
    return jsonify(list(log_buffer))


@app.route('/camera_stream')
def camera_stream():
    return Response(
        gen_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/servo_test')
def servo_test():
    channel = request.args.get('channel', type=int)
    angle   = request.args.get('angle',   type=float)
    if channel is None or angle is None:
        return jsonify({'status': 'error', 'msg': 'missing params'}), 400
    angle = max(0.0, min(180.0, angle))
    ok = node_handle.test_servo(channel, angle)
    return jsonify({'status': 'ok' if ok else 'service_unavailable'})



#################################################
# MOTOR KILL / RESTART
#################################################

motor_procs = []

@app.route('/kill_motors')
def kill_motors():
    global motor_procs
    node_handle.send_cmd('disarm')
    for p in motor_procs:
        try:
            p.terminate()
        except Exception:
            pass
    motor_procs = []
    subprocess.run(['pkill', '-f', 'ros2_pca9685_node'], capture_output=True)
    subprocess.run(['pkill', '-f', 'motion_node'],        capture_output=True)
    log('MOTORS KILLED — web UI still running')
    return jsonify({'status': 'killed'})


restart_status = {'state': 'idle'}

@app.route('/restart_motors')
def restart_motors():
    threading.Thread(target=_do_restart, daemon=True).start()
    log('RESTARTING MOTORS...')
    return jsonify({'status': 'restarting'})

def _do_restart():
    global motor_procs, restart_status
    restart_status['state'] = 'restarting'
    for p in motor_procs:
        try: p.terminate()
        except Exception: pass
    motor_procs = []
    subprocess.run(['pkill', '-f', 'ros2_pca9685_node'], capture_output=True)
    subprocess.run(['pkill', '-f', 'motion_node'],        capture_output=True)
    time.sleep(1.0)
    pca    = subprocess.Popen(['ros2', 'run', 'ros2_pca9685', 'ros2_pca9685_node'])
    time.sleep(2.0)
    motion = subprocess.Popen(['ros2', 'run', 'sharky_control', 'motion_node'])
    motor_procs = [pca, motion]
    restart_status['state'] = 'ready'
    log('MOTORS RESTARTED — re-arm to move')

@app.route('/restart_status')
def get_restart_status():
    return jsonify(restart_status)


#################################################
# MAIN
#################################################

def ros_spin():
    rclpy.spin(node_handle)

def main():
    global node_handle

    rclpy.init()
    node_handle = WebNode()

    log('Sharky web interface started')
    log(f'Battery: {get_battery_status()["status"]}')

    threading.Thread(target=ros_spin, daemon=True).start()

    # Default to loopback so the unauthenticated hardware controls are not
    # exposed to the LAN accidentally. Set SHARKY_BIND_HOST deliberately when
    # placing the interface behind an authenticated local reverse proxy.
    app.run(host=os.environ.get('SHARKY_BIND_HOST', '127.0.0.1'), port=5000)

if __name__ == '__main__':
    main()
