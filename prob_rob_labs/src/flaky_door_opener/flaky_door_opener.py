import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import Empty

max_torque = 5.0
torque_hold = 10

class FlakyDoorOpener(Node):

    def __init__(self):
        super().__init__('flaky_door_opener')
        self.log = self.get_logger()
        self.pub_torque = self.create_publisher(
            Float64, '/hinged_glass_door/torque', 1)
        self.sub_command = self.create_subscription(
            Empty, '/door_open', self.handle_command, 1)
        self.torque = 0
        self.torque_counter = 0
        self.opened = 0
        self.times = 0

    def handle_command(self, _):
        if self.torque == 0:
            self.torque = random.choice([1.0, 0.0, 0.0, 0.0, 0.0]) * max_torque

            self.torque_counter = 0
        elif self.torque_counter < torque_hold:
            self.torque_counter += 1
        else:
            self.torque = 0.0
        self.log.info(f'door open requested using torque {self.torque}')
        self.pub_torque.publish(Float64(data=self.torque))
        if self.torque == max_torque: self.opened +=1
        self.times +=1
        self.log.info(f"Current Probability of door openning: {self.opened / self.times}")

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    flaky_door_opener = FlakyDoorOpener()
    flaky_door_opener.spin()
    flaky_door_opener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
