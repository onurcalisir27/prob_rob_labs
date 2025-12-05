import rclpy
from rclpy.node import Node
import numpy
from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler

publish_freq = 1.0/30.0 # publish at 30 Hz
class EkfLocalization(Node):

    def __init__(self):
        super().__init__('ekf_localization')
        self.log = self.get_logger()
        self.timer = self.create_timer(publish_freq, self.transform_publisher)
        self.create_subscription(Odometry, '/ekf_pose', self.localization_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.prev_trace = None
        self.Transform = None

    def transform_publisher(self):

        if self.Transform is None:
            return
        # Send the transformation every 30 Hz
        self.Transform.header.stamp = self.get_clock().now().to_msg()
        # self.log.info("Sending Map Transform!")
        self.tf_broadcaster.sendTransform(self.Transform)

    def localization_callback(self, msg):
        
        # Get current trace
        trace = self.trace(msg)

        # Check if the trace went down (measurement update)
        if self.prev_trace is not None:
            if trace < self.prev_trace - 1e-6:
                self.log.info("Measurement Update! Update Transform")
                self.Transform = self.update_transform(msg)

        self.prev_trace = trace

    def update_transform(self, msg):
        try:
            t = self.tf_buffer.lookup_transform(
                                'base_footprint',
                                'odom',
                                time=rclpy.time.Time(),
                                timeout=rclpy.duration.Duration(seconds=1.0))
            self.log.info("Got Odom-Base Transform")
        except TransformException as ex:
            self.log.info('Couldnt get transform')
            return
        
        # From EKF Localization we are getting Map->Base
        # We want to publish to Map->Odom
        # we can look-up Odom->Base
        # Map->Base = Map->Odom @ Odom->Base 
        # Map->Odom = Map->Base @ inv(Odom->Base)
        # MO = MB * BO!

        pose = msg.pose.pose
        T_MB = self.transformation_matrix(pose.position, pose.orientation)
        T_BO = self.transformation_matrix(t.transform.translation, t.transform.rotation)

        T_MO = T_MB @ T_BO

        translation = T_MO[0:2, 2]
        theta = numpy.arctan2(T_MO[1,0], T_MO[0,0])
        
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, theta)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        return transform
    
    def spin(self):
        rclpy.spin(self)

    def trace(self, msg):
        cov = msg.pose.covariance
        return cov[0] + cov[7] + cov[35]
    
    def transformation_matrix(self, position, orientation):
        theta = self.q2yaw(orientation)
        rot =  numpy.array([
            [numpy.cos(theta), -numpy.sin(theta)],
            [numpy.sin(theta), numpy.cos(theta)]
        ])

        t = numpy.array([
            position.x, 
            position.y
            ])
        
        T = numpy.identity(3)
        T[0:2, 0:2] = rot
        T[0:2, 2] = t
        return T
    
    # quaternion to yaw
    def q2yaw(self, quaternion):
        euler = euler_from_quaternion([quaternion.x, 
                                       quaternion.y, 
                                       quaternion.z, 
                                       quaternion.w])
        return euler[2]

def main():
    rclpy.init()
    ekf_localization = EkfLocalization()
    ekf_localization.spin()
    ekf_localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

