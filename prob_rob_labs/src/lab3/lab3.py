import rclpy
import numpy as np
import math
from rclpy.node import Node
from std_msgs.msg import Float64


heartbeat_period = 0.01

class Lab3(Node):

    def __init__(self):
        super().__init__('lab3')
        self.log = self.get_logger()
        # self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.feature_sub_ = self.create_subscription(Float64, "/feature_mean", self.sub_callback, 10)
        self.feature_mean = 0
        self.read = []
        self.false = 0
        self.true = 0

        self.declare_parameter('threshold', 220.0)
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value

        self.declare_parameter('horizon', 500)
        self.horizon = self.get_parameter('horizon').get_parameter_value().integer_value

    # def heartbeat(self):
    #     #self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)

    def sub_callback(self, msg):
        self.feature_mean = msg.data
        self.read.append(self.feature_mean)
        if len(self.read) == self.horizon:
            self.read.pop(0)
        
            list = [1 if x > self.threshold else 0 for x in self.read]
            prob = sum(list) / len(list)
            self.log.info(f"Probability: {prob}")

    def calculate_average(self):
        average = sum(self.read) / (self.horizon-1)
        self.log.info(f'Average is now: {average}')


def main():
    rclpy.init()
    lab3 = Lab3()
    lab3.spin()
    lab3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
