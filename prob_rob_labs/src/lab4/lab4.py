import rclpy
import numpy as np
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

heartbeat_period = 0.1

class Lab4(Node):

    def __init__(self):
        super().__init__('lab4')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)
        
        self.vel_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)

        self.gt_pose_pub_ = self.create_publisher(PoseStamped, "/tb3/ground_truth/pose", 10)
        self.gt_twist_pub_ = self.create_publisher(TwistStamped, "/tb3/ground_truth/twist", 10)
       
        self.state_sub_ = self.create_subscription(LinkStates, "/gazebo/link_states", self.link_callback, 10)
        self.ground_truth_pose = 0
        self.ground_truth_twist = 0
        self.declare_parameter('reference_frame', "odom")
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
       
    def heartbeat(self):
        self.log.info("Heartbeat")
        self.publish_gt(self.ground_truth_pose, self.ground_truth_twist)
        
    def link_callback(self, msg):
        ind = msg.name.index("waffle_pi::base_footprint")
        self.ground_truth_pose = msg.pose[ind]
        self.ground_truth_twist = msg.twist[ind]
        
    def publish_gt(self, gt_pose, gt_twist):

        header = Header()
        header.frame_id = self.reference_frame
        header.stamp = self.get_clock().now().to_msg()
        timestamp = self.get_clock().now().to_msg()
        self.log.info(f"Time stamp: {timestamp}")
        
        pose_msg = PoseStamped()
        pose_msg.pose = gt_pose
        pose_msg.header = header

        twist_msg = TwistStamped()
        twist_msg.twist = gt_twist
        twist_msg.header = header

        self.gt_pose_pub_.publish(pose_msg)
        self.gt_twist_pub_.publish(twist_msg)


    def spin(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    lab4 = Lab4()
    lab4.spin()
    lab4.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
