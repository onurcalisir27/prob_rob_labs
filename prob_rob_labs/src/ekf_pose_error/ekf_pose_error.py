import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray

class EkfPoseError(Node):

    def __init__(self):
        super().__init__('ekf_pose_error')
        self.log = self.get_logger()

        self.ekf_sub = Subscriber(self, Odometry, "/ekf_pose")
        self.gt_sub = Subscriber(self, PoseStamped, "/tb3/ground_truth/pose") 
        
        self.sync = ApproximateTimeSynchronizer([self.ekf_sub, self.gt_sub], 
                                                queue_size=10, 
                                                slop=0.5)
        
        self.sync.registerCallback(self.error_callback)
        self.error_pub = self.create_publisher(Float32MultiArray, "/pose_error", 10)

    def spin(self):
        rclpy.spin(self)

    def error_callback(self, ekf_msg, gt_msg):
        ekf_x = ekf_msg.pose.pose.position.x
        ekf_y = ekf_msg.pose.pose.position.y 
        gt_x =  gt_msg.pose.position.x
        gt_y = gt_msg.pose.position.y
        position_error = math.sqrt((gt_x - ekf_x)**2 + (gt_y - ekf_y)**2)
        
        ekf_theta = self.quaternion_to_yaw(ekf_msg.pose.pose.orientation)
        gt_theta = self.quaternion_to_yaw(gt_msg.pose.orientation)
        
        angle_error = self.unwrap(gt_theta - ekf_theta)

        # Publish
        error_msg = Float32MultiArray()
        error_msg = {position_error, angle_error}
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
    ekf_pose_error = EkfPoseError()
    ekf_pose_error.spin()
    ekf_pose_error.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
