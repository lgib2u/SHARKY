#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros2_pca9685.srv import SetPwm
import time

# -----------------------------
# Servo channel mapping
# -----------------------------

FL = {'name': 'FL', 'rot': 0, 'foot': 1, 'flip': False}
FR = {'name': 'FR', 'rot': 2, 'foot': 3, 'flip': True}
RL = {'name': 'RL', 'rot': 4, 'foot': 5, 'flip': False}
RR = {'name': 'RR', 'rot': 6, 'foot': 7, 'flip': True}

LEGS = [FL, FR, RL, RR]

# -----------------------------
# Angle definitions (tune these)
# -----------------------------

ROT_FORWARD = 30.0
ROT_BACK = 100.0
ROT_NEUTRAL = 90.0

FOOT_UP = 60.0
FOOT_DOWN = 150.0
FOOT_NEUTRAL = 90.0

# Timing (seconds)
LIFT_TIME = 0.2
SWING_TIME = 0.3
PLACE_TIME = 0.2
STEP_PAUSE = 0.2


class QuadrupedTest(Node):
    def __init__(self):
        super().__init__('quadruped_gait_test')

        self.cli = self.create_client(SetPwm, '/pca9685/set_pwm')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for PCA9685 service...')

        self.get_logger().info('Connected to PCA9685 service')

    # -----------------------------
    # Low-level servo command
    # -----------------------------

    def set_servo(self, channel, angle, flip=False):
        if flip:
            angle = 180.0 - angle

        req = SetPwm.Request()
        req.channel_num = int(channel)
        req.target_position = float(angle)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    # -----------------------------
    # Leg primitives
    # -----------------------------

    def lift_leg(self, leg):
        self.set_servo(leg['foot'], FOOT_UP, leg['flip'])

    def lower_leg(self, leg):
        self.set_servo(leg['foot'], FOOT_DOWN, leg['flip'])

    def swing_forward(self, leg):
        self.set_servo(leg['rot'], ROT_FORWARD, leg['flip'])

    def swing_backward(self, leg):
        self.set_servo(leg['rot'], ROT_BACK, leg['flip'])

    def neutral_leg(self, leg):
        self.set_servo(leg['rot'], ROT_NEUTRAL, leg['flip'])
        self.set_servo(leg['foot'], FOOT_NEUTRAL, leg['flip'])

    # -----------------------------
    # Poses
    # -----------------------------

    def neutral_stance(self):
        for leg in LEGS:
            self.neutral_leg(leg)
        time.sleep(1.0)

    # -----------------------------
    # Single leg step
    # -----------------------------

    def step_leg(self, leg):
        self.lift_leg(leg)
        time.sleep(LIFT_TIME)

        self.swing_forward(leg)
        time.sleep(SWING_TIME)

        self.lower_leg(leg)
        time.sleep(PLACE_TIME)

    # -----------------------------
    # Crawl gait (diagonal pairs)
    # -----------------------------

    def walk_forward(self, steps=2):
        for _ in range(steps):
            # Diagonal pair 1
            self.step_leg(FL)
            self.step_leg(RR)

            self.swing_backward(FR)
            self.swing_backward(RL)
            time.sleep(STEP_PAUSE)

            # Diagonal pair 2
            self.step_leg(FR)
            self.step_leg(RL)

            self.swing_backward(FL)
            self.swing_backward(RR)
            time.sleep(STEP_PAUSE)

    # -----------------------------
    # Main routine
    # -----------------------------

    def run(self):
        self.get_logger().info('Entering neutral stance')
        self.neutral_stance()

        self.get_logger().info('Walking forward')
        self.walk_forward(steps=3)

        self.get_logger().info('Returning to neutral')
        self.neutral_stance()


def main():
    rclpy.init()
    node = QuadrupedTest()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down safely')
        node.neutral_stance()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
