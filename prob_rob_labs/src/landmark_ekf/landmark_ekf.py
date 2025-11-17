import rclpy
import numpy
import json
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from prob_rob_msgs.msg import Point2DArrayStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

path = "/run/host/workdir/local_oc2356/ros2_ws/src/prob_rob_labs_ros_2/landmark_map.json"

class LandmarkEkf(Node):

    def __init__(self):
        super().__init__('landmark_ekf')
        self.log = self.get_logger()

        # Get map
        self.declare_parameter("map_path", path)
        file_path = self.get_parameter("map_path").get_parameter_value().string_value
        with open(file_path, 'r') as file:
            self.map = json.load(file)

        # Get camera params
        self.camera_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_callback, 10)
        self.fx= None
        self.fy= None
        self.cx= None
        self.cy= None
        
        # get odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.state = numpy.zeros((3, 1)) # x, y , theta
        self.I = numpy.identity(3)
        self.Cov = 0.01 * self.I
        self.M = numpy.array([[1.0e-05, 0.0], # Twist covariance from odom topic
                              [0.0, 0.001]])
        self.G = self.I # state transition jacobian at rest
        self.V = numpy.zeros((3,2)) # input jacobian

        self.system_time = None
        self.initialized = False
        self.last_vel = None
        self.odom_pub = self.create_publisher(Odometry, "/ekf_pose", 10)

        # Get measurement model
        self.landmark_ratio = 0.5 / (2*0.1) # height / 2*radius
        self.measurement_model = []
        for color, landmark in self.map.items():
            topic = f'/vision_{color}/corners'
            sub = self.create_subscription(Point2DArrayStamped, topic, lambda msg, c=color, z=landmark: self.measurement_cb(msg,c,z), 10)
            self.measurement_model.append(sub)
        self.Q = numpy.zeros((2,2)) # measurement covariance
        self.H = numpy.zeros((2,3))
        self.H[1,2] = -1.0

    def measurement_cb(self, msg, c, z):
        if not self.initialized:
            self.initialized = True # initialize with first measurement
            self.system_time = msg.header.stamp
            return # skip the first measurement as initializer
        
        # get measurement
        range, bearing = self.vision_process(msg.points, c)
        if range is None:
            # No viable measurement
            return
        
        timestamp = msg.header.stamp
        dt = (timestamp - self.system_time)
        if dt <= 0:
            return # late-arriving sample or no time elapsed
        
        self.system_time = timestamp

        if dt > 0 and self.last_vel is not None:
            v, w = self.last_vel
            self.prediction(v, w, dt)
            self.measurement_update(z, range, bearing)
            self.publish_odom(timestamp)
        else:
            return

    def odom_callback(self, msg):
        # store velocity
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.last_vel = (v,w)

        if not self.initialized:
            return # wait for first measurement
        
        timestamp = msg.header.stamp
        dt = (timestamp - self.system_time)
        if dt <= 0 :
            return # late-arriving sample or no time elapsed
        self.system_time = timestamp

        # run prediction step
        self.prediction(v, w, dt)
        
    def prediction(self, v, w, dt):
        theta = self.state[2,0]
        if abs(w) < 0.01:
            # Linear model update
            self.G[0,2] = -v*dt*numpy.sin(theta)
            self.G[1,2] = v*dt*numpy.cos(theta)

            self.V[0,0] = dt*numpy.cos(theta)
            self.V[0,1] = 0.0
            self.V[1,0] = dt*numpy.sin(theta)
            self.V[1,1] = 0.0
            self.V[2,1] = dt

            self.state[0,0] = self.state[0,0] + v * dt * numpy.cos(theta)
            self.state[1,0] = self.state[1,0] + v * dt * numpy.sin(theta) 
            self.state[2,0] = self.unwrap(theta + w * dt)

        else:
            # Arc model update
            self.G[0,2] = - v/w * numpy.cos(theta) + v/w * numpy.cos(theta + w * dt)
            self.G[1,2] = - v/w * numpy.sin(theta) + v/w * numpy.sin(theta + w * dt)

            self.V[0,0] = (-numpy.sin(theta)+numpy.sin(theta+w*dt))/w
            self.V[0,1] = v*(numpy.sin(theta)-numpy.sin(theta+w*dt))/w**2 + v*numpy.cos(theta+w*dt)*dt/w
            self.V[1,0] = (numpy.cos(theta)-numpy.cos(theta+w*dt))/w
            self.V[1,1] = -v*(numpy.cos(theta)-numpy.cos(theta+w*dt))/w**2 + v*numpy.sin(theta+w*dt)*dt/w
            self.V[2,1] = dt

            self.state[0,0] = self.state[0,0] + (-v/w * numpy.sin(theta) + v/w * numpy.sin(theta+w*dt))
            self.state[1,0] = self.state[1,0] + (v/w * numpy.cos(theta) - v/w * numpy.cos(theta+w*dt))
            self.state[2,0] = self.unwrap(theta + w * dt)

        # covariance update is the same
        self.Cov = self.G @ self.Cov @ self.G.T + self.V @ self.M @ self.V.T
    
    def measurement_update(self, landmark, range, bearing):
        # notation change: we only really care about the difference m-state so we will replace m with that
        my = landmark['y'] - self.state[1,0]
        mx = landmark['x'] - self.state[0,0]

        q = (mx**2 + my**2)
        z1 = numpy.sqrt(q)
        z2 = self.unwrap(numpy.arctan2(my, mx) - self.state[2,0])

        self.H[0,0] = - mx / z1
        self.H[0,1] = -my / z1
        self.H[1,0] = my / q
        self.H[1,1] = -mx / q

        # Frame Transformation Required
        # Covariance Transformation too?
        dz = numpy.array([[range - z1], 
                          [self.unwrap(bearing - z2)]])


        # [2x2] = [2x3]@[3x3]@[3x2] + [2x2] 
        S = self.H @ self.Cov @ self.H.T + self.Q
        # [3x2] = [3x3]@[3x2]@[2x2] 
        K = self.Cov @ self.H.T @ numpy.linalg.inv(S)
        # [3x1] = [3x1] + [3x2]@[2x1]
        self.state = self.state + K @ dz
        self.state[2,0] = self.unwrap(self.state[2,0])
        # [3x3] = [3x3] - [3x2]@[2x3])@[3x3]
        self.Cov = (self.I - K@self.H)@self.Cov

    def vision_process(self, points, color):
        # Accept a measurement if enough points are presented
        if len(points) > 4: 
            self.log.info(f"Received measurements from {color} landmark")
            x = []
            y = []
            for point in points:
                x.append(point.x)
                y.append(point.y)

            x_min = min(x)
            x_max = max(x)
            y_min = min(y)
            y_max = max(y)

            epsilon = 1e-4
            dy = y_max - y_min
            dx = x_max - x_min
            self.ratio = dy / (dx + epsilon)

            threshold1 = 1.5
            threshold2 = 0.7
            # Drop offending measurement if landmark appears distorted out of the threshold
            if self.ratio > self.landmark_ratio * threshold1 or self.ratio < self.landmark_ratio * threshold2:
                self.log.info(f"Measurement is offending the model, the measured ration is off by {self.ratio/self.landmark_ratio}")
                return None

            center_x = 0.5*(x_max + x_min)
            tmp = (self.cx -center_x) / self.fx
            bearing = numpy.arctan(tmp)
            range = 0.5 * self.fy / (dy * numpy.cos(bearing))
            self.log.info(f"Measurement: {range} meters, {bearing} rad")
            self.measurement_variance(range)

            return range, bearing
        else:
            return None
        
    def measurement_variance(self, range):
        self.Q[0,0] = max(0.001, -0.002094 + 0.001508 * range)
        self.Q[1,1] = max(0.0001, 0.000127 - 0.000004 * range)

    def publish_odom(self, timestamp):
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        odom_msg.pose.pose.position.x = self.state[0, 0]
        odom_msg.pose.pose.position.y = self.state[1, 0]
        odom_msg.pose.pose.orientation = self.to_quaternion(self.state[2, 0])

        odom_msg.pose.covariance[0] = self.Cov[0, 0]
        odom_msg.pose.covariance[1] = self.Cov[0, 1]
        odom_msg.pose.covariance[5] = self.Cov[0, 2]
        odom_msg.pose.covariance[6] = self.Cov[1, 0]
        odom_msg.pose.covariance[7] = self.Cov[1, 1]
        odom_msg.pose.covariance[11] = self.Cov[1, 2]
        odom_msg.pose.covariance[30] = self.Cov[2, 0]
        odom_msg.pose.covariance[31] = self.Cov[2, 1]
        odom_msg.pose.covariance[35] = self.Cov[2, 2]
        self.odom_pub.publish(odom_msg)

    def unwrap(self, angle):
        while angle > numpy.pi:
            angle = angle - (2 * numpy.pi)
        while angle < -numpy.pi:
            angle = angle + (2 * numpy.pi)
        return angle

    def to_quaternion(self, theta):
        quaternions = quaternion_from_euler(0.0, 0.0, theta)
        q = Quaternion()
        q.x = quaternions[0]
        q.y = quaternions[1]
        q.z = quaternions[2]
        q.w = quaternions[3]
        return q
    
    def camera_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
    
    def spin(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    landmark_ekf = LandmarkEkf()
    landmark_ekf.spin()
    landmark_ekf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

'''
twist
  covariance:
  - 1.0e-05, 0.0, 0.0, 0.0, 0.0, 0.0
  - 0.0, 1.0e-05, 0.0, 0.0, 0.0, 0.0
  - 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0
  - 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0
  - 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0
  - 0.0, 0.0, 0.0, 0.0, 0.0, 0.001
'''