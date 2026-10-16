#!/usr/bin/env python3

import math
import time
import rclpy
from rclpy.node import Node
from ros2_pca9685.srv import SetPwm
from std_srvs.srv import SetBool

from sharky_control.gait_profile import (
    DEFAULT_SPEED_SCALE,
    HARDWARE_LIMITS,
    clamp_speed_scale,
    front_support_target,
    load_profile,
    tilt_targets,
    turn_targets,
)


FL = {'name': 'FL', 'rot': 0, 'foot': 1, 'flip': False}
FR = {'name': 'FR', 'rot': 2, 'foot': 3, 'flip': True}
RL = {'name': 'RL', 'rot': 4, 'foot': 5, 'flip': False}
RR = {'name': 'RR', 'rot': 6, 'foot': 7, 'flip': True}

LEGS = [FL, FR, RL, RR]
INTERPOLATION_HZ = 20.0


class GaitController(Node):

    def __init__(self):
        super().__init__('gait_controller')

        self.cli = self.create_client(SetPwm, '/pca9685/set_pwm')
        self.enable_cli = self.create_client(SetBool, '/pca9685/set_enabled')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for PCA9685 service...')
        while not self.enable_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for PCA9685 enable service...')

        self.get_logger().info('Connected to PCA9685')

        # STATE
        self.armed         = False
        self.speed_scale   = DEFAULT_SPEED_SCALE
        self.stop_requested = False
        self.profile, profile_source = load_profile()
        self.get_logger().info(
            f"Loaded gait profile '{self.profile['name']}' from {profile_source}"
        )

        # Track last known servo angles
        self.last_positions = {}

        # Do not move hardware merely because the node started. Servo motion
        # begins only after an explicit arm or movement command.


    # ======================================================
    # Speed Control
    # ======================================================

    def set_speed(self, scale):
        self.speed_scale = clamp_speed_scale(scale)
        self.get_logger().info(f"Speed scale set to {self.speed_scale}")


    # ======================================================
    # Arm / Disarm
    # ======================================================

    def arm(self):
        # Profiles can be uploaded atomically while disarmed. Re-read before
        # every arm so activation never requires a code rebuild or node restart.
        try:
            self.profile, profile_source = load_profile()
            self.get_logger().info(
                f"Using gait profile '{self.profile['name']}' from {profile_source}"
            )
        except (OSError, ValueError) as error:
            self.get_logger().error(f"ARM rejected: invalid gait profile: {error}")
            return

        if not self._set_outputs_enabled(True):
            self.get_logger().error("ARM rejected: PCA9685 could not be enabled")
            return

        self.armed = True
        self.stop_requested = False
        self.get_logger().warning("ARMED")
        self.stand_pose()

    def disarm(self):
        self.armed = False
        self.stop_requested = True
        self.last_positions.clear()
        self._set_outputs_enabled(False)
        self.get_logger().warning("DISARMED; PWM outputs disabled")

    def _set_outputs_enabled(self, enabled):
        req = SetBool.Request()
        req.data = bool(enabled)
        future = self.enable_cli.call_async(req)
        deadline = time.time() + 2.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.001)

        if not future.done():
            self.get_logger().error("Timed out changing PCA9685 enable state")
            return False

        result = future.result()
        if result is None or not result.success:
            message = result.message if result is not None else "no response"
            self.get_logger().error(f"PCA9685 enable request failed: {message}")
            return False
        return True


    # ======================================================
    # Coordinated Servo Motion
    # ======================================================

    def move_servo_timed(self, channel, target_angle, duration=0.5, flip=False):
        return self.move_group_timed(
            [(channel, target_angle, flip)],
            duration,
        )

    def move_group_timed(self, targets, duration):
        """Move a phase's servos together along an eased trajectory.

        Target tuples are ``(channel, logical_angle, flip)``.  Requests are all
        queued per frame before responses are awaited.  This removes both the
        one-servo-at-a-time staircase and the old instantaneous angle jumps.
        """
        resolved = []

        for channel, logical_angle, flip in targets:
            target_angle = 180.0 - logical_angle if flip else logical_angle
            minimum, maximum = HARDWARE_LIMITS[int(channel)]
            if not minimum <= target_angle <= maximum:
                self.get_logger().error(
                    f"Servo {channel} target {target_angle:.1f} outside immutable "
                    f"{minimum:.1f}-{maximum:.1f} degree envelope"
                )
                self.stop_requested = True
                return False

            start_angle = self.last_positions.get(channel)
            if start_angle is not None and abs(target_angle - start_angle) < 0.5:
                continue
            resolved.append((channel, start_angle, target_angle))

        if not resolved:
            return True

        # Profile timings describe nominal 1.0x motion.  The controller starts
        # at 3.0x and accepts validated rates from 0.5x through 5.0x.
        effective_duration = duration / self.speed_scale
        frame_count = max(1, math.ceil(effective_duration * INTERPOLATION_HZ))
        phase_started = time.monotonic()

        for frame in range(1, frame_count + 1):
            if self.stop_requested:
                return False

            fraction = frame / frame_count
            eased = fraction * fraction * (3.0 - 2.0 * fraction)
            pending = []
            for channel, start_angle, target_angle in resolved:
                # On the first command after ARM there is no trustworthy
                # starting angle.  Command the safe pose directly; all later
                # phases are interpolated from acknowledged positions.
                if start_angle is None:
                    frame_angle = target_angle
                else:
                    frame_angle = start_angle + (target_angle - start_angle) * eased

                req = SetPwm.Request()
                req.channel_num = int(channel)
                req.target_position = float(frame_angle)
                pending.append((channel, frame_angle, self.cli.call_async(req)))

            if not self._await_servo_frame(pending):
                return False

            frame_deadline = phase_started + (
                effective_duration * frame / frame_count
            )
            remaining = frame_deadline - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
        return True

    def _await_servo_frame(self, pending):
        deadline = time.monotonic() + 2.0
        while (any(not future.done() for _, _, future in pending)
               and time.monotonic() < deadline):
            time.sleep(0.001)

        for channel, frame_angle, future in pending:
            if not future.done():
                self.get_logger().error(f"Servo {channel} command timed out")
                self.stop_requested = True
                return False

            result = future.result()
            if result is None or not result.is_success:
                message = result.response if result is not None else "no response"
                self.get_logger().error(f"Servo {channel} command failed: {message}")
                self.stop_requested = True
                return False

            self.last_positions[channel] = frame_angle
        return True

    def _angle(self, name):
        return float(self.profile['angles'][name])

    def _timing(self, name):
        return float(self.profile['timings'][name])

    def _foot_down(self, leg):
        key = 'foot_down_front' if leg in (FL, FR) else 'foot_down_rear'
        return self._angle(key)

    def _rot_forward(self, leg):
        key = 'rot_forward_front' if leg in (FL, FR) else 'rot_forward_rear'
        return self._angle(key)


    # ======================================================
    # Poses
    # ======================================================

    def neutral_stance(self):
        self.move_group_timed(
            [(leg['rot'], self._angle('rot_neutral'), leg['flip']) for leg in LEGS],
            self._timing('pose_rotation'),
        )
        if self.stop_requested:
            return
        self.move_group_timed(
            [(leg['foot'], self._angle('foot_neutral'), leg['flip']) for leg in LEGS],
            self._timing('pose_feet'),
        )

    def stand_pose(self):
        self.move_group_timed(
            [(leg['rot'], self._angle('rot_neutral'), leg['flip']) for leg in LEGS],
            self._timing('pose_rotation'),
        )
        if self.stop_requested:
            return
        self.move_group_timed(
            [(leg['foot'], self._foot_down(leg), leg['flip']) for leg in LEGS],
            self._timing('pose_feet'),
        )

    def tilt_pose(self, direction):
        """Tilt the body using bounded front/rear foot extension."""
        targets = tilt_targets(self.profile, direction)
        self.move_group_timed(
            [(leg['rot'], self._angle('rot_neutral'), leg['flip']) for leg in LEGS],
            self._timing('pose_rotation'),
        )
        if self.stop_requested:
            return
        self.move_group_timed(
            [(leg['foot'], targets[leg['name']], leg['flip']) for leg in LEGS],
            self._timing('pose_feet'),
        )


    # ======================================================
    # Leg Primitives (Timed)
    # ======================================================

    def lift(self, leg):
        self.move_servo_timed(
            leg['foot'], self._angle('foot_up'), self._timing('lift'), leg['flip']
        )

    def lower(self, leg):
        self.move_servo_timed(
            leg['foot'], self._foot_down(leg), self._timing('lower'), leg['flip']
        )

    def forward(self, leg):
        self.move_servo_timed(
            leg['rot'], self._rot_forward(leg), self._timing('transfer'), leg['flip']
        )

    def backward(self, leg):
        self.move_servo_timed(
            leg['rot'], self._angle('rot_back'), self._timing('transfer'), leg['flip']
        )


    # ======================================================
    # Crawl Gait — Forward / Backward
    # ======================================================

    def _crawl_step(
        self,
        active_leg,
        active_target,
        support_target,
        brace_front=False,
    ):
        """One stable crawl phase: lift, coordinated transfer, lower."""
        bracing_leg = None
        bracing_target = None
        if brace_front and active_leg in (FL, FR):
            # Shift the other planted front arm forward before unloading the
            # active one.  Retain that preload through the transfer phase so
            # it supports the nose instead of being immediately swept back.
            bracing_leg = FR if active_leg == FL else FL
            bracing_target = front_support_target(self.profile)
            if not self.move_servo_timed(
                bracing_leg['rot'],
                bracing_target,
                self._timing('transfer'),
                bracing_leg['flip'],
            ):
                return False

        self.lift(active_leg)
        if self.stop_requested:
            return False

        targets = []
        for leg in LEGS:
            if leg == active_leg:
                target = active_target(leg)
            elif leg == bracing_leg:
                target = bracing_target
            else:
                target = support_target(leg)
            targets.append((leg['rot'], target, leg['flip']))
        if not self.move_group_timed(targets, self._timing('transfer')):
            return False

        self.lower(active_leg)
        return not self.stop_requested

    def _stop_or_stand(self):
        if not self.stop_requested:
            return False
        self.stop_requested = False
        self.stand_pose()
        return True

    def walk_forward(self, steps=1):
        if not self.armed:
            return

        gait_order = [FL, RR, FR, RL]

        for _ in range(steps):
            for leg in gait_order:
                if self._stop_or_stand():
                    return
                if not self._crawl_step(
                    leg,
                    self._rot_forward,
                    lambda _support: self._angle('rot_back'),
                    brace_front=True,
                ):
                    self._stop_or_stand()
                    return

        self.stand_pose()


    def walk_backward(self, steps=1):
        if not self.armed:
            return

        gait_order = [FL, RR, FR, RL]

        for _ in range(steps):
            for leg in gait_order:
                if self._stop_or_stand():
                    return
                if not self._crawl_step(
                    leg,
                    lambda _active: self._angle('rot_back'),
                    self._rot_forward,
                ):
                    self._stop_or_stand()
                    return

        self.stand_pose()


    # ======================================================
    # Pivot Turns — diagonal pairs rotate body in place
    # ======================================================

    def _pivot_step(self, leg, target):
        self.lift(leg)
        if self.stop_requested:
            return False
        self.move_servo_timed(
            leg['rot'], target(leg), self._timing('transfer'), leg['flip']
        )
        if self.stop_requested:
            return False
        self.lower(leg)
        return not self.stop_requested

    def turn_left(self, steps=1):
        """Pivot left: left-side legs step back, right-side legs step forward."""
        if not self.armed:
            return

        for _ in range(steps):
            if self._stop_or_stand():
                return

            targets = turn_targets(self.profile, 'left')
            sequence = [FL, RR, FR, RL]
            for leg in sequence:
                if not self._pivot_step(leg, lambda item: targets[item['name']]):
                    self._stop_or_stand()
                    return

        self.stand_pose()


    def turn_right(self, steps=1):
        """Pivot right: right-side legs step back, left-side legs step forward."""
        if not self.armed:
            return

        for _ in range(steps):
            if self._stop_or_stand():
                return

            targets = turn_targets(self.profile, 'right')
            sequence = [FR, FL, RR, RL]
            for leg in sequence:
                if not self._pivot_step(leg, lambda item: targets[item['name']]):
                    self._stop_or_stand()
                    return

        self.stand_pose()


def main(args=None):
    rclpy.init(args=args)
    node = GaitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
