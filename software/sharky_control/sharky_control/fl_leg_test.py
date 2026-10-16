import rclpy
from rclpy.node import Node
from ros2_pca9685.srv import SetPwm
import time

FL_ROT = 0
FL_FOOT = 1

class FLLegTest(Node):
    def __init__(self):
        super().__init__('fl_leg_test')
        self.cli = self.create_client(SetPwm, '/pca9685/set_pwm')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for PCA9685 service...')

    def set_servo(self, ch, angle):
        req = SetPwm.Request()
        req.channel_num = int(ch)
        req.target_position = float(angle)
        #self.cli.call(req)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def run(self):
        self.get_logger().info('Lifting FL leg')
        self.set_servo(FL_FOOT, 60)
        time.sleep(1)

        self.get_logger().info('Rotating FL leg forward')
        self.set_servo(FL_ROT, 60)
        time.sleep(1)

        self.get_logger().info('Lowering FL leg')
        self.set_servo(FL_FOOT, 120)
        time.sleep(1)

        self.get_logger().info('Reset position')
        self.set_servo(FL_ROT, 90)
        self.set_servo(FL_FOOT, 90)

def main():
    rclpy.init()
    node = FLLegTest()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
