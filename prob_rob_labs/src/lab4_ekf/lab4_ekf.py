import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from message_filters import Subscriber, ApproximateTimeSynchronizer

from rclpy.clock import Clock
heartbeat_period = 0.1
dt = 0.1

class Lab4Ekf(Node):

    def __init__(self):
        super().__init__('lab4_ekf')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.imu_sub = Subscriber(self, Imu, "/imu")
        self.joints_pub = Subscriber(self, JointState, "/joint_states")

        self.timer = self.create_timer(1, self.TimerCallback)

        queue_size = 10
        max_delay = 0.05
        self.time_sync = ApproximateTimeSynchronizer([self.imu_sub,
                                                      self.joints_pub],
                                                     queue_size,
                                                     max_delay)

        self.time_sync.registerCallback(self.SyncCallback)

        # Assignment 3 Findings
        # self.G = 1
        self.a = 0.9
        self.b = 0.7

        # [theta, x, y, v, w]
        self.state = np.zeros((5, 1))
        self.F = np.array([[1.0, 0.0, 0.0, 0.0, dt],
                           [-dt*self.state[3, 0]*np.sin(self.state[0, 0]),
                            1.0, 0.0, dt*np.cos(self.state[0, 0]), 0.0],
                           [dt*self.state[3, 0]*np.cos(self.state[0, 0]), 0.0,
                            1.0, dt*np.sin(self.state[0, 0]), 0.0],
                           [0.0, 0.0, 0.0, self.a, 0.0],
                           [0.0, 0.0, 0.0, 0.0, self.b]
                           ])
        # [uv, uw]
        self.u = np.zeros((2, 1))

    def heartbeat(self):
        self.log.info('heartbeat')

    def state_update(self):
        '''
        Update the steps following the update equations

        x[n] = x[n-1] + v_x * dt * cos(theta[n-1])
        y[n] = y[n-1] + v_x * dt * sin(theta[n-1])
        theta[n] = theta[n-1] + w * dt

        v[n] = a * v[n-1] + (1-a) * uv[n-1]
        w[n] = b * w[n-1] + (1-b) * uw[n-1]

        '''

        # X Y Theta
        self.state[1, 0] = self.state[1, 0] +  \
            self.state[3, 0] * dt * np.cos(self.state[0, 0])
        self.state[2, 0] = self.state[2, 0] + \
            self.state[3, 0] * dt * np.sin(self.state[0, 0])
        self.state[0, 0] = self.state[0, 0] + self.state[4, 0] * dt
        # Vx W
        self.state[3, 0] = self.a * self.state[3, 0] + \
            (1.0-self.a) * self.u[0, 0]
        self.state[4, 0] = self.b * self.state[4, 0] + \
            (1.0-self.b) * self.u[1, 0]

        self.F[1, 0] = 1.0
        self.F[1,]
        self.F[2, 0] = 1.0

    def measurement_model(self, z):
        # z = [wr, wl, wg]
        self.wheel_r = 33e-3
        self.wheel_s = 143.5e-3

        v = 0.5 * self.wheel_r * (z[0, 0] + z[1, 0])
        w = 0.5 * self.wheel_r / self.wheel_s * (z[0, 0] - z[1, 0])

        # z = C x
        return v, w

    def SyncCallback(self, imu, joint_states):
        temp_sec = imu.header.stamp.sec
        fluid_sec = joint_states.header.stamp.sec
        self.get_logger().info(f'Sync callback with {temp_sec} and {fluid_sec} as times')

        if (imu.header.stamp.sec > 2.0):
            return

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
