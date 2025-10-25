import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf_transformations import euler_from_quaternion
from prob_rob_msgs.msg import Odom2D

heartbeat_period = 0.1

class Lab4OdomError(Node):

    def __init__(self):
        super().__init__('lab4_odom_error')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.ekf_sub = Subscriber(self, Odometry, "/ekf_odom")
        self.gt_sub = Subscriber(self, PoseStamped, "/tb3/ground_truth/pose") 
        self.gt_twist_sub = Subscriber(self, TwistStamped, "/tb3/ground_truth/twist")
        self.sync = ApproximateTimeSynchronizer([self.ekf_sub, self.gt_sub, self.gt_twist_sub], 
                                                queue_size=10, 
                                                slop=0.05)
        self.sync.registerCallback(self.error_callback)

        self.error_pub = self.create_publisher(Odom2D, "/odom_error", 10)

    def heartbeat(self):
        self.log.info('heartbeat')

    def spin(self):
        rclpy.spin(self)

    def error_callback(self, ekf_msg, gt_msg, gt_twist_msg):
        ekf_x = ekf_msg.pose.pose.position.x
        ekf_y = ekf_msg.pose.pose.position.y 
        gt_x =  gt_msg.pose.position.x
        gt_y = gt_msg.pose.position.y
        position_error = math.sqrt( (gt_x - ekf_x)**2 + (gt_y - ekf_y)**2)
        
        ekf_theta = self.quaternion_to_yaw(ekf_msg.pose.pose.orientation)
        gt_theta = self.quaternion_to_yaw(gt_msg.pose.orientation)
        
        angle_error = self.unwrap(gt_theta - ekf_theta)

        # Velocity errors
        ekf_v = ekf_msg.twist.twist.linear.x
        ekf_w = ekf_msg.twist.twist.angular.z
        gt_v = gt_twist_msg.twist.linear.x
        gt_w = gt_twist_msg.twist.angular.z

        v_err = gt_v - ekf_v
        w_err = gt_w - ekf_w
        error_msg = Odom2D()
        error_msg.pos_error=position_error
        error_msg.ang_error=angle_error
        error_msg.v_error=v_err
        error_msg.w_error=w_err
        self.error_pub.publish(error_msg)
        
    def quaternion_to_yaw(self, quaternion):
        euler = euler_from_quaternion([quaternion.x, 
                                       quaternion.y, 
                                       quaternion.z, 
                                       quaternion.w])
        return euler[2]
    
    def unwrap(self, angle):
        while angle > math.pi:
            angle = angle - (2 * math.pi)
        while angle < -math.pi:
            angle = angle + (2 * math.pi)
        return angle

def main():
    rclpy.init()
    lab4_odom_error = Lab4OdomError()
    lab4_odom_error.spin()
    lab4_odom_error.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
