import rclpy
from rclpy.node import Node
import numpy as np

heartbeat_period = 0.1
dt = 0.1

class Lab4Ekf(Node):

    def __init__(self):
        super().__init__('lab4_ekf')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        # [theta, x, y, v, w]
        self.state = np.zeros((5,1))
        self.F = np.zeros((5,5))
        self.F[0,0] = 1.0
        self.F[4,0] = dt
        self.F

        # self.G = 1
        self.a = 0.9
        self.b = 0.7

        # [uv, uw]
        self.u = np.zeros((2,1))

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
        self.state[1,0] = self.state[1,0] + self.state[3,0] * dt * np.cos(self.state[0,0])
        self.state[2,0] = self.state[2,0] + self.state[3,0] * dt * np.sin(self.state[0,0])
        self.state[0,0] = self.state[0,0] + self.state[4,0] * dt
        # Vx W
        self.state[3,0] = self.a * self.state[3,0] + (1.0-self.a) * self.u[0,0]
        self.state[4,0] = self.b * self.state[4,0] + (1.0-self.b) * self.u[1,0]

        self.F[0,0] = 1.0
        





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
