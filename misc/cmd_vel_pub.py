import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPub(Node):
    def __init__(self):
        super().__init__('cmd_vel_pub')
        self.pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.05, self.cmd_vel_callback)
    def cmd_vel_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.5
        self.pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_pub = CmdVelPub()
    rclpy.spin(cmd_vel_pub)
    cmd_vel_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
