import rclpy
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
        
        self.log.info(f"Probabilities from total samples:{self.horizon*self.trials}, threshold:{self.threshold} ")
        self.log.info(f"P(z=closed | x=closed) = {self.P_z_closed_x_closed}")
        self.log.info(f"P(z=open | x=closed) = {self.P_z_open_x_closed}")
        self.log.info(f"P(z=open | x=open) = {self.P_z_open_x_open}")
        self.log.info(f"P(z=closed | x=open) = {self.P_z_closed_x_open}")

    def spin(self):
        rclpy.spin(self)

    def sub_callback(self, msg):
        self.count += 1
        self.z = msg.data

        if self.trials == 5:
            self.state = "finished"

        if self.state == "closed":
            if self.count > self.horizon:
                self.log.info("Closed measurements ended, open the door now")
                self.state = "opening"
                self.count = 0
            else:
                self.z_given_x_closed.append(self.z)

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
                self.z_given_x_open.append(self.z) 

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
                rclpy.shutdown()

    def move_door(self, value):
        torque_msg = Float64()
        torque_msg.data = value
        self.torque_pub_.publish(torque_msg)
   
def main():
    rclpy.init()
    lab3 = Lab3()
    lab3.spin()
    lab3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
