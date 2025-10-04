import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class Lab3(Node):

    def __init__(self):
        super().__init__('lab3')
        self.log = self.get_logger()
        # self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.feature_sub_ = self.create_subscription(Float64, "/feature_mean", self.sub_callback, 10)
        self.torque_pub_ = self.create_publisher(Float64, "/hinged_glass_door/torque", 10)
        self.vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)

        self.declare_parameter('threshold', 235.0)
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value

        self.declare_parameter('horizon', 1000)
        self.horizon = self.get_parameter('horizon').get_parameter_value().integer_value

        self.state = "closed"
        self.z_given_x_closed = []
        self.z_given_x_open = []
        self.count = 0
        self.trials = 0

        self.P_z_closed_x_closed = 0
        self.P_z_open_x_closed = 0
        self.P_z_open_x_open = 0
        self.P_z_closed_x_open = 0

        self.bel = np.array([[0.1],
                            [0.9]])
        
        self.measurement_model = np.array([[self.P_z_open_x_open, self.P_z_open_x_closed],
                                           [self.P_z_closed_x_open, self.P_z_closed_x_closed]])

        self.prediction_model = np.array([1.0, 0.0],
                                         [0.0, 1.0])

        self.log.info(f"Using measurement model: {self.measurement_model}")
        self.log.info(f"Starting measuring with horizon:{self.horizon}, threshold:{self.threshold} ")

    def calculate_probabilities(self):
        true_negatives = 0
        true_positives = 0
        false_negatives = 0
        false_positives = 0

        for measurement in self.z_given_x_closed:
            if measurement >= self.threshold:
                # P(z=closed | x=closed)
                true_negatives +=1
            else:
                # P(z=open | x=closed)
                false_positives +=1

        for measurement in self.z_given_x_open:
            if measurement < self.threshold:
                # P(z=open | x=open)
                true_positives +=1
            else:
                # P(z=closed | x=open)
                false_negatives +=1

        self.P_z_closed_x_closed = true_negatives / (true_negatives + false_positives)
        self.P_z_open_x_closed = false_positives / (true_negatives + false_positives)
        self.P_z_open_x_open = true_positives / (true_positives + false_negatives)
        self.P_z_closed_x_open = false_negatives / (true_positives + false_negatives)
        total_samples = len(self.z_given_x_closed) + len(self.z_given_x_open)

        self.log.info(f"Probabilities from {len(self.z_given_x_closed)} closed samples, {len(self.z_given_x_open)} open samples")
        self.log.info(f"Threshold Value at: {self.threshold}, total samples: {total_samples}")
        self.log.info(f"P(z=closed | x=closed) = {self.P_z_closed_x_closed}")
        self.log.info(f"P(z=open | x=closed) = {self.P_z_open_x_closed}")
        self.log.info(f"P(z=open | x=open) = {self.P_z_open_x_open}")
        self.log.info(f"P(z=closed | x=open) = {self.P_z_closed_x_open}")

        self.measurement_model = np.array([[self.P_z_open_x_open, self.P_z_open_x_closed],
                                           [self.P_z_closed_x_open, self.P_z_closed_x_closed]])

    def spin(self):
        rclpy.spin(self)

    def sub_callback(self, msg):
        self.count += 1
        z = msg.data

        if self.trials == 3:
            self.state = "finished"

        if self.state == "closed":
            if self.count > self.horizon:
                self.log.info("Closed measurements ended, open the door now")
                self.state = "opening"
                self.count = 0
            else:
                self.z_given_x_closed.append(z)

        elif self.state == "opening":
            self.move_door(10.0)
            if self.count > 100:
                self.log.info("Door is open, resume measurements")
                self.state = "open"
                self.count = 0

        elif self.state == "open":
            if self.count > self.horizon:
                self.log.info("Open measurements ended, close the door now")
                self.state = "closing"
                self.count = 0
            else:
                self.z_given_x_open.append(z)

        elif self.state == "closing":
            self.move_door(-10.0)
            if self.count > 100:
                self.state = "closed"
                self.count = 0
                self.log.info(f"Door is closed, finished trial numer:{self.trials}, starting new trial")
                self.trials +=1

        elif self.state == "finished":
            self.move_door(-10.0)
            if self.count > 50:
                self.calculate_probabilities()
                self.state = "decision"
                self.count = 0
        
        elif self.state == "decision":
            if z > self.threshold:
                self.bayes_update(1) # closed is 1
            else:
                self.bayes_update(0) # open is 0

            if self.bel[0] < 0.9:
                self.move_door(10.0) # if low confidence on the door being open, open it
            
            if self.bel[0] > 0.999:
                self.state = "drive"
                self.count = 0
        
        elif self.state == "drive":
            msg = Twist()
            msg.linear.x = 1.0
            if self.count < 50:
                self.vel_pub_.publish(msg)
            else:
                rclpy.shutdown()
            
    def move_door(self, value):
        torque_msg = Float64()
        torque_msg.data = value
        self.torque_pub_.publish(torque_msg)

    def bayes_update(self, z):
        bel_bar = self.prediction_model @ self.bel
        unnormalized_posterior = self.measurement_model[z, :] * bel_bar.flatten()
        posterior = unnormalized_posterior / sum(unnormalized_posterior)
        self.bel = np.array([posterior]).transpose()
        self.log.info(f"Updated Belief: belief open: {self.bel[0]}, belief closed: {self.bel[1]}")

def main():
    rclpy.init()
    lab3 = Lab3()
    lab3.spin()
    lab3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
