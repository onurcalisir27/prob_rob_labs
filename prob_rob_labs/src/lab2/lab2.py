import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

heartbeat_period = 0.1

class Lab2(Node):

    def __init__(self):
        super().__init__('lab2')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        self.vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.torque_pub_ = self.create_publisher(Float64, "/hinged_glass_door/torque", 10)
        self.state = "init"
        self.declare_parameter('desired_speed', 1.0)
        self.desired_speed = self.get_parameter('desired_speed').get_parameter_value().double_value
        self.count = 0

    def heartbeat(self):
        self.log.info("Update Count")
        if self.state == "init":
            self.state = "open"

        elif self.state == "open":
            self.log.info("Opening Door")
            torque_msg = Float64()
            torque_msg.data = 5.0
            self.torque_pub_.publish(torque_msg)
            self.count += 1
            if self.count == 40:
                torque_msg = Float64()
                torque_msg.data = 0.0
                self.torque_pub_.publish(torque_msg)
                self.state = "drive"
                self.count = 0
        
        elif self.state == "drive":
            self.log.info("Driving Bot")
            vel_msg = Twist()
            vel_msg.linear.x = self.desired_speed
            self.vel_pub_.publish(vel_msg)
            self.count += 1
            if self.count == 40:
                self.state = "close"
                vel_msg = Twist()
                vel_msg.linear.x = 0.0
                self.vel_pub_.publish(vel_msg)
                self.count = 0

        elif self.state == "close":
            self.log.info("Closing Door")
            torque_msg = Float64()
            torque_msg.data = -5.0
            self.torque_pub_.publish(torque_msg)
            self.count += 1
            if self.count == 20:
                torque_msg = Float64()
                torque_msg.data = 0.0
                self.torque_pub_.publish(torque_msg)
                self.state = "end"
                self.count = 0
        
        elif self.state == "end":
            self.log.info('Im Done!')

    def spin(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    lab2 = Lab2()
    lab2.spin()
    lab2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
