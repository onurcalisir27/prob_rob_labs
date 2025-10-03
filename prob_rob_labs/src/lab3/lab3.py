import rclpy
import numpy as np
import math
import time
from rclpy.node import Node
from std_msgs.msg import Float64


heartbeat_period = 0.01

class Lab3(Node):

    def __init__(self):
        super().__init__('lab3')
        self.log = self.get_logger()
        # self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.feature_sub_ = self.create_subscription(Float64, "/feature_mean", self.sub_callback, 10)
        self.torque_pub_ = self.create_publisher(Float64, "/hinged_glass_door/torque", 10)

        self.declare_parameter('threshold', 240.0)
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value

        self.declare_parameter('horizon', 500)
        self.horizon = self.get_parameter('horizon').get_parameter_value().integer_value

        self.state = "closed"
        self.z_given_x_closed = []
        self.z_given_x_open = []
        self.count = 0

        self.true_positives = 0
        self.false_positives = 0
        self.false_negatives = 0
        self.true_negatives = 0

    def calculate_probabilities(self):
        #self.log.info('heartbeat')
        for measurement in self.z_given_x_closed:
            if measurement >= self.threshold:
                # P(z=closed | x=closed)
                self.true_negatives +=1
            else:
                # P(z=open | x=closed)
                self.false_positives +=1

        for measurement in self.z_given_x_open:
            if measurement < self.threshold:
                # P(z=open | x=open)
                self.true_positives +=1
            else:
                # P(z=closed | x=open)
                self.false_negatives +=1
        
        P_z_closed_x_closed = self.true_negatives / (self.true_negatives+self.false_positives)

        P_z_open_x_closed = self.false_positives / (self.true_negatives+self.false_positives)

        P_z_open_x_open = self.true_positives / (self.true_positives+self.false_negatives)
        
        P_z_closed_x_open = self.false_negatives / (self.true_positives+self.false_negatives)
        
        self.log.info(f"Probabilities with horizon:{self.horizon}, threshold:{self.threshold} ")
        self.log.info(f"P(z=closed | x=closed) = {P_z_closed_x_closed}")
        self.log.info(f"P(z=open | x=closed) = {P_z_open_x_closed}")
        self.log.info(f"P(z=open | x=open) = {P_z_open_x_open}")
        self.log.info(f"P(z=closed | x=open) = {P_z_closed_x_open}")

    def spin(self):
        rclpy.spin(self)

    def sub_callback(self, msg):
        self.count += 1
        self.z = msg.data

        if self.state == "closed":
            if self.count > self.horizon:
                self.state = "open"
                self.count = 0
                self.open_door()

            else:
                self.z_given_x_closed.append(self.z)

        elif self.state == "open":
            if self.count > self.horizon:
                self.state = "finished"
                self.count = 0
            else:
                self.z_given_x_open.append(self.z) 
        
        elif self.state == "finished":
            self.calculate_probabilities()
            rclpy.shutdown()

    def open_door(self):
        torque_msg = Float64()
        torque_msg.data = 10.0
        self.torque_pub_.publish(torque_msg)
        time.sleep(3)
   
def main():
    rclpy.init()
    lab3 = Lab3()
    lab3.spin()
    lab3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
