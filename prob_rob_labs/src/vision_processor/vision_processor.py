import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from prob_rob_msgs.msg import Point2DArrayStamped
from prob_rob_msgs.msg import Distance2D
import numpy

heartbeat_period = 0.1

class VisionProcessor(Node):
    '''
    Write a ROS node that subscribes to the /camera/camera_info topic and the corner-points topic for a landmark of your choice and calculates the distance and bearing to
    the landmark. 

    Your node will need to know the landmark height to use it in calculations and the landmark color to know which topic to subscribe to. Both should be the node parameters.

    Publish the calculated distance and bearing. 

    Submit the code.

    Bring up the node and drive the robot around pointing it to the landmark from various distances.
    
    Make sure the node handles the case when the landmark goes out of sight (it should stop calculating and publishing the measurement and it may not crash).

    Make sense out of the values you are getting. 
    
    You can use the rqt_plot tool to visualize the measurement.
    
    Place the robot a few meters away from the landmark and make it face it directly. 
    
    Next, start rotating the robot in place and watch the distance measurement plot. 
    
    The measurement should remain more or less constant (which is expected), except when the landmark reaches the edge
    of the cameras field of view. 
    
    The reason this happens is that the cornerpoints from one side of the landmark are missing, which may result in incorrect calculation of the symmetry axis, which
    further skews the bearing measurement, followed by skewing the distance measurement. 
    
    If you notice this problem, extend the code to drop the offending measurement.
    '''

    def __init__(self):
        super().__init__('vision_processor')
        self.log = self.get_logger()
        self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info_callback, 10)
        self.cyan_points_sub = self.create_subscription(Point2DArrayStamped, "/vision_cyan/corners", self.cyan_callback, 10)

        self.landmark_pub = self.create_publisher(Distance2D, "/landmark", 10)
        
        self.declare_parameter("true_height", 0.5)
        self.true_height = self.get_parameter("true_height").get_parameter_value().double_value

        self.distance_ave = 0
        self.theta_ave = 0
        self.run = 0
        
    def heartbeat(self):
        self.log.info('heartbeat')

    def camera_info_callback(self, msg):

        #     [fx 0  cx]
        # K = [ 0 fy cy]
        #     [ 0 0   1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def cyan_callback(self, msg):
        if len(msg.points) != 0: 
            points = msg.points 
            self.log.info(f"Number of points: {len(points)}")

            x = []
            y = []
            for point in points:
                x.append(point.x)
                y.append(point.y)

            x_min = min(x)
            x_max = max(x)
            y_min = min(y)
            y_max = max(y)

            height = y_max - y_min
            center_x = 0.5*(x_max + x_min)
            center_y = 0.5*(y_max + y_min)

            self.log.info(f"Got height = {height}, and center at ({center_x}, {center_y})")
            
            tmp = (self.cx -center_x) / self.fx
            theta = numpy.arctan(tmp)
            d = self.true_height * self.fy / (height * numpy.cos(theta))
            self.publish_filtered_landmark(theta, d)

    def publish_filtered_landmark(self, theta, d):
        self.run += 1
        self.theta_ave = (self.theta_ave + theta) / (self.run)
        self.distance_ave = (self.distance_ave + d) / (self.run)
        self.landmark_pub.publish(Distance2D(distance=self.distance_ave, bearing=self.theta_ave))
        
    def spin(self):
        rclpy.spin(self)


def main():
    rclpy.init()
    vision_processor = VisionProcessor()
    vision_processor.spin()
    vision_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
