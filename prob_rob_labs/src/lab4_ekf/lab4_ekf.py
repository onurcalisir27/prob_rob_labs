import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from rclpy.clock import Clock

heartbeat_period = 0.1
dt = 0.1

class Lab4Ekf(Node):

    def __init__(self):
        super().__init__('lab4_ekf')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.imu_sub = Subscriber(self, Imu, "/imu")
        self.joints_sub = Subscriber(self, JointState, "/joint_states")
        self.cmd_sub = Subscriber(self, Twist, "/cmd_vel")

        self.input_sub = self.create_subscription(Twist, "/cmd_vel", self.input_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/ekf_odom", 10)

        queue_size = 10
        slop = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.imu_sub,
                                                      self.joints_sub],
                                                     queue_size,
                                                     slop)

        self.time_sync.registerCallback(self.SyncCallback)

        # Assignment 3 Findings
        # self.G = 1
        self.a = 0.8
        self.b = 0.3

        # State : [theta, x, y, v, w]^T
        self.state = np.zeros((5, 1))

        # This should also be somewhat tunable
        self.declare_parameter("initial_covariance", 0.001)
        initial_covariance = self.get_parameter("initial_covariance").get_parameter_value().double_value
        self.Cov = initial_covariance * np.identity(5)

        # State Transition Jacobian :
        self.G = np.array([[1.0, 0.0, 0.0, 0.0, dt],
                           [-dt*self.state[3, 0]*np.sin(self.state[0, 0]),
                            1.0, 0.0, dt*np.cos(self.state[0, 0]), 0.0],
                           [dt*self.state[3, 0]*np.cos(self.state[0, 0]),
                            0.0, 1.0, dt*np.sin(self.state[0, 0]), 0.0],
                           [0.0, 0.0, 0.0, self.a, 0.0],
                           [0.0, 0.0, 0.0, 0.0, self.b]])
        # Input : [uv, uw]
        self.u = np.zeros((2, 1))

        # Input covariance and B matrix
        self.B = np.array([[0.0, 0.0],
                           [0.0, 0.0],
                           [0.0, 0.0],
                           [1-self.a, 0.0],
                           [0.0, 1-self.b]])

        # TUNABLE-ish
        sigma_u = np.array([[0.05, 0.0],
                            [0.0, 0.05]])
        
        self.R = self.B @ sigma_u @ self.B.T

        # Turtlebot params
        self.wheel_r = 33e-3
        self.wheel_R = 143.5e-3 / 2.0

        # z = C x
        # z = [wr, wl, wg]^T
        self.C = np.array([
            [0.0, 0.0, 0.0, 1/self.wheel_r, self.wheel_R/self.wheel_r],
            [0.0, 0.0, 0.0, 1/self.wheel_r, -self.wheel_R/self.wheel_r],
            [0.0, 0.0, 0.0, 0.0, 1.0]])

        # From the /imu message, the covariance of the imu angular velocity reading is 4.0e-08
        self.Sigma_z = np.array([[0.0005, 0.0, 0.0],
                                 [0.0, 0.0005, 0.0],
                                 [0.0, 0.0, 4.0e-08]])

    def SyncCallback(self, imu, encoders):
        imu_timestamp = imu.header.stamp
        encoder_timestamp = encoders.header.stamp

        right = encoders.name.index("wheel_right_joint")
        left = encoders.name.index("wheel_left_joint")

        wr = encoders.velocity[right]
        wl = encoders.velocity[left]
        wg = imu.angular_velocity.z

        z = np.array([[wr],
                      [wl],
                      [wg]])
        
        self.log.info(f"Received Measurement z: \n{z}")

        self.state_update()
        self.measurement_update(z)
        self.publish_odom(encoder_timestamp)

    def heartbeat(self):
        self.log.info('heartbeat')

    def input_callback(self, msg):
        self.u[0, 0] = msg.linear.x
        self.u[1, 0] = msg.angular.z

    def state_update(self):
        '''
        Update the steps following the update equations

        x[n] = x[n-1] + v_x * dt * cos(theta[n-1])
        y[n] = y[n-1] + v_x * dt * sin(theta[n-1])
        theta[n] = theta[n-1] + w * dt
        v[n] = a * v[n-1] + (1-a) * uv[n-1]
        w[n] = b * w[n-1] + (1-b) * uw[n-1]

        '''
        # update state and jacobian
        self.G[1, 0] = -dt*self.state[3, 0]*np.sin(self.state[0, 0])
        self.G[1, 3] = dt*np.cos(self.state[0, 0])
        self.G[2, 0] = dt*self.state[3, 0]*np.cos(self.state[0, 0])
        self.G[2, 3] = dt*np.sin(self.state[0, 0])

        # update x and y with old theta and v
        self.state[1, 0] = self.state[1, 0] +  \
            self.state[3, 0] * dt * np.cos(self.state[0, 0])
        self.state[2, 0] = self.state[2, 0] + \
            self.state[3, 0] * dt * np.sin(self.state[0, 0])
        
        self.state[0, 0] = self.state[0, 0] + self.state[4, 0] * dt

        self.state[3, 0] = self.a * self.state[3, 0] + \
            (1.0-self.a) * self.u[0, 0]
        self.state[4, 0] = self.b * self.state[4, 0] + \
            (1.0-self.b) * self.u[1, 0]

        # update covariance
        self.Cov = self.G @ self.Cov @ self.G.T + self.R

    def measurement_update(self, z):

        innovation = z - self.C @ self.state
        K = self.kalman_gain()
        self.state = self.state + K @ innovation
        self.Cov = (np.identity(5) - K @ self.C) @ self.Cov

    def kalman_gain(self):

        temp = self.C @ self.Cov @ self.C.T + self.Sigma_z
        return self.Cov @ self.C.T @ np.linalg.inv(temp)

    def publish_odom(self, timestamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position.x = self.state[1, 0]
        odom_msg.pose.pose.position.y = self.state[2, 0]

        odom_msg.pose.pose.orientation = self.to_quaternion(self.state[0, 0])

        odom_msg.pose.covariance[0] = self.Cov[1, 1]
        odom_msg.pose.covariance[1] = self.Cov[1, 2]
        odom_msg.pose.covariance[5] = self.Cov[1, 0]
        odom_msg.pose.covariance[6] = self.Cov[2, 1]
        odom_msg.pose.covariance[7] = self.Cov[2, 2]
        odom_msg.pose.covariance[11] = self.Cov[2, 0]
        odom_msg.pose.covariance[30] = self.Cov[0, 1]
        odom_msg.pose.covariance[31] = self.Cov[0, 2]
        odom_msg.pose.covariance[35] = self.Cov[0, 0]

        odom_msg.twist.twist.linear.x = self.state[3, 0]
        odom_msg.twist.twist.angular.z = self.state[4, 0]

        self.odom_pub.publish(odom_msg)

    def unwrap(self, angle):
        while angle > np.pi:
            angle = angle - (2 * np.pi)
        while angle < -np.pi:
            angle = angle + (2 * np.pi)
        return angle

    def to_quaternion(self, theta):

        quaternions = quaternion_from_euler(0.0, 0.0, theta)
        q = Quaternion()
        q.x = quaternions[0]
        q.y = quaternions[1]
        q.z = quaternions[2]
        q.w = quaternions[3]
        return q

    def spin(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    lab4_ekf = Lab4Ekf()
    lab4_ekf.spin()
    lab4_ekf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
