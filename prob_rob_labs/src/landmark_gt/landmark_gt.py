import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion
import math
from prob_rob_msgs.msg import Landmark

class LandmarkGt(Node):

    def __init__(self):
        super().__init__('landmark_gt')
        self.log = self.get_logger()
        self.gt_sub = self.create_subscription(LinkStates, "/gazebo/link_states", self.gt_callback, 10)
        self.landmark_gt_pub = self.create_publisher(Landmark, "/landmark_gt", 10)
        

    def gt_callback(self,msg):
        timestamp = Header()
        timestamp.frame_id = "camera_link"
        timestamp.stamp = self.get_clock().now().to_msg()

        # Cyan Landmark
        landmark_ind = msg.name.index("landmark_5::link")
        self.landmark_pose = msg.pose[landmark_ind]

        bot_ind = msg.name.index("waffle_pi::camera_link")
        self.bot_pose = msg.pose[bot_ind]

        camera_x = self.bot_pose.position.x
        camera_y = self.bot_pose.position.y
        camera_yaw = self.quaternion_to_yaw(self.bot_pose.orientation)

        landmark_x = self.landmark_pose.position.x   
        landmark_y = self.landmark_pose.position.y

        self.range = math.sqrt((camera_x - landmark_x)**2 + (camera_y - landmark_y)**2)
        self.bearing = math.atan2(landmark_y-camera_y, landmark_x-camera_x) - camera_yaw

        self.log.info(f'True range is {self.range} and true bearing is {self.bearing}')

        self.landmark_gt_pub.publish(Landmark(header=timestamp, distance=self.range, bearing=self.bearing))

    def quaternion_to_yaw(self, quaternion):
        euler = euler_from_quaternion([quaternion.x, 
                                        quaternion.y, 
                                        quaternion.z, 
                                        quaternion.w])
        return euler[2]

    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    landmark_gt = LandmarkGt()
    landmark_gt.spin()
    landmark_gt.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
