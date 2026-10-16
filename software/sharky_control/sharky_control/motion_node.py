#!/usr/bin/env python3

import threading
import time
import socket
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sharky_control.gait_controller import GaitController


LOCAL_COMMAND_SOCKET = Path.home() / '.cache' / 'sharky' / 'motion.sock'


class MotionNode(Node):

    def __init__(self):
        super().__init__('motion_node')

        self.controller    = GaitController()
        self._current_cmd  = None   # set by ROS callback, consumed by worker
        self._pending_action = None  # arm/disarm/pose, consumed by worker
        self._state_lock = threading.Lock()
        self._last_motion_time = None
        self._watchdog_triggered = False
        self.declare_parameter('command_timeout_sec', 1.0)
        self.command_timeout_sec = max(
            0.2,
            float(self.get_parameter('command_timeout_sec').value)
        )

        # Worker thread — continuously executes the current command
        self._worker = threading.Thread(target=self._gait_loop, daemon=True)
        self._worker.start()

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )
        self._start_local_command_socket()
        self.watchdog_timer = self.create_timer(0.1, self._watchdog_callback)

        self.get_logger().info(
            f"Motion node ready; command timeout is {self.command_timeout_sec:.1f}s."
        )


    def _start_local_command_socket(self):
        """Accept web-controller commands without relying on DDS discovery."""
        LOCAL_COMMAND_SOCKET.parent.mkdir(parents=True, exist_ok=True)
        try:
            LOCAL_COMMAND_SOCKET.unlink()
        except FileNotFoundError:
            pass

        self._local_socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self._local_socket.bind(str(LOCAL_COMMAND_SOCKET))
        LOCAL_COMMAND_SOCKET.chmod(0o600)
        self._local_thread = threading.Thread(
            target=self._local_command_loop,
            daemon=True,
        )
        self._local_thread.start()


    def _local_command_loop(self):
        while True:
            try:
                payload = self._local_socket.recv(128).decode('ascii')
                command, separator, value = payload.partition(':')
                msg = Twist()
                if command == 'speed' and separator:
                    msg.linear.z = float(value)
                elif command == 'stop':
                    pass
                elif command == 'disarm':
                    msg.angular.x = -1.0
                elif command == 'arm':
                    msg.angular.x = 1.0
                elif command == 'neutral':
                    msg.angular.y = 1.0
                elif command == 'tilt_up':
                    msg.angular.y = 2.0
                elif command == 'tilt_down':
                    msg.angular.y = -2.0
                elif command == 'forward':
                    msg.linear.x = 1.0
                elif command == 'backward':
                    msg.linear.x = -1.0
                elif command == 'turn_left':
                    msg.angular.z = 1.0
                elif command == 'turn_right':
                    msg.angular.z = -1.0
                else:
                    self.get_logger().warning(
                        f"Rejected unknown local command: {command}"
                    )
                    continue
                self.cmd_callback(msg)
            except (OSError, UnicodeDecodeError, ValueError) as error:
                self.get_logger().error(f"Local command socket error: {error}")
                time.sleep(0.1)


    def _gait_loop(self):
        while True:
            action = self._take_action()
            if action == 'arm':
                self.controller.arm()
                continue
            if action == 'disarm':
                self.controller.disarm()
                continue
            if action == 'neutral':
                if self.controller.armed:
                    self.get_logger().warning("Moving to defined neutral pose")
                    self.controller.stop_requested = False
                    self.controller.neutral_stance()
                else:
                    self.get_logger().info("Ignoring neutral pose, robot disarmed.")
                continue
            if action in ('tilt_up', 'tilt_down'):
                if self.controller.armed:
                    direction = 'up' if action == 'tilt_up' else 'down'
                    self.get_logger().warning(f"Moving to tilt-{direction} pose")
                    self.controller.stop_requested = False
                    self.controller.tilt_pose(direction)
                else:
                    self.get_logger().info(
                        f"Ignoring {action.replace('_', '-')} pose, robot disarmed."
                    )
                continue
            if action == 'stand':
                if self.controller.armed:
                    self.get_logger().info("Returning all legs to standing pose")
                    self.controller.stop_requested = False
                    self.controller.stand_pose()
                continue

            with self._state_lock:
                cmd = self._current_cmd
            if cmd and self.controller.armed:
                if   cmd == 'forward':    self.controller.walk_forward(1)
                elif cmd == 'backward':   self.controller.walk_backward(1)
                elif cmd == 'turn_left':  self.controller.turn_left(1)
                elif cmd == 'turn_right': self.controller.turn_right(1)
            else:
                time.sleep(0.05)


    def _queue_action(self, action):
        """Queue service-using work without blocking an executor callback."""
        with self._state_lock:
            # Once queued, a disarm must be processed before any later action.
            if self._pending_action == 'disarm' and action != 'disarm':
                return
            self._pending_action = action


    def _take_action(self):
        with self._state_lock:
            action = self._pending_action
            self._pending_action = None
            return action


    def _watchdog_callback(self):
        with self._state_lock:
            current_cmd = self._current_cmd
            last_motion_time = self._last_motion_time

        if current_cmd is None or last_motion_time is None:
            return

        elapsed = time.monotonic() - last_motion_time
        if elapsed <= self.command_timeout_sec:
            return

        with self._state_lock:
            self._current_cmd = None
            self._last_motion_time = None
        self.controller.stop_requested = True
        self._queue_action('stand')
        if not self._watchdog_triggered:
            self.get_logger().warning(
                "Motion command timed out; stopping gait and remaining armed."
            )
            self._watchdog_triggered = True


    # ======================================================
    # CMD_VEL INTERPRETATION
    # ======================================================

    def cmd_callback(self, msg: Twist):

        # -------------------------
        # ARM / DISARM
        # -------------------------
        if msg.angular.x == 1.0:
            with self._state_lock:
                self._current_cmd = None
                self._last_motion_time = None
            self.controller.stop_requested = True
            self._queue_action('arm')
            return

        if msg.angular.x == -1.0:
            with self._state_lock:
                self._current_cmd = None
                self._last_motion_time = None
            self.controller.stop_requested = True
            self._queue_action('disarm')
            return

        # -------------------------
        # NEUTRAL POSE
        # Uses only the gait controller's defined 90-degree neutral angles.
        # -------------------------
        if msg.angular.y == 1.0:
            with self._state_lock:
                self._current_cmd = None
                self._last_motion_time = None
            self.controller.stop_requested = True
            self._queue_action('neutral')
            return

        # -------------------------
        # BOUNDED BODY TILT POSES
        # -------------------------
        if abs(msg.angular.y) == 2.0:
            with self._state_lock:
                self._current_cmd = None
                self._last_motion_time = None
            self.controller.stop_requested = True
            self._queue_action('tilt_up' if msg.angular.y > 0.0 else 'tilt_down')
            return

        # -------------------------
        # SPEED CONTROL
        # linear.z = speed scale
        # -------------------------
        if msg.linear.z > 0.0:
            self.controller.set_speed(msg.linear.z)
            return

        # -------------------------
        # STOP — clear queue and interrupt current gait
        # -------------------------
        if msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0:
            with self._state_lock:
                self._current_cmd = None
                self._last_motion_time = None
            self.controller.stop_requested = True
            self._queue_action('stand')
            self._watchdog_triggered = False
            return

        # If not armed, ignore motion
        if not self.controller.armed:
            self.get_logger().info("Ignoring motion, robot disarmed.")
            return

        # -------------------------
        # SET CURRENT MOVEMENT COMMAND
        # -------------------------
        if msg.linear.x > 0.1:
            command = 'forward'
        elif msg.linear.x < -0.1:
            command = 'backward'
        elif msg.angular.z > 0.2:
            command = 'turn_left'
        elif msg.angular.z < -0.2:
            command = 'turn_right'
        else:
            command = None

        if command is not None:
            with self._state_lock:
                self._current_cmd = command
                self._last_motion_time = time.monotonic()
            self.controller.stop_requested = False
            self._watchdog_triggered = False


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()

    # Both nodes must share one executor so service responses are processed
    # while the worker thread polls future.done() in gait_controller.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(node.controller)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()
