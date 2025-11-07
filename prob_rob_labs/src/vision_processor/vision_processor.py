import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from prob_rob_msgs.msg import Point2DArrayStamped
from prob_rob_msgs.msg import Landmark
import numpy
from std_msgs.msg import Header

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        self.log = self.get_logger()
        
        self.declare_parameter("landmark_height", 0.5)
        self.true_height = self.get_parameter("landmark_height").get_parameter_value().double_value
        self.declare_parameter("landmark_color", 'cyan')
        self.landmark_color = self.get_parameter("landmark_color").get_parameter_value().string_value
        
        landmark_color_list = {"red", "green", "yellow", "magenta", "cyan"}
        if self.landmark_color not in landmark_color_list:
            self.log.info("User passed incorrect landmark color, please try again")
            self.destroy_node()

        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera_info", self.camera_info_callback, 10)
        self.cyan_points_sub = self.create_subscription(Point2DArrayStamped, "/vision_" + self.landmark_color + "/corners", self.vision_callback, 10)
        self.landmark_pub = self.create_publisher(Landmark, "/landmark", 10)
        
        self.true_width = 2*0.1
        self.true_ratio = self.true_height / self.true_width

    def camera_info_callback(self, msg):

        #     [fx 0  cx]
        # K = [ 0 fy cy]
        #     [ 0 0   1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def vision_callback(self, msg):
        # Only process if there are points present
        if len(msg.points) > 4: 
            points = msg.points 
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
            if self.ratio > self.true_ratio * threshold1 or self.ratio < self.true_ratio * threshold2:
                self.log.info(f"Measurement is offending the model, the measured ration is off by {self.ratio/self.true_ratio}")
                return

            center_x = 0.5*(x_max + x_min)
            center_y = 0.5*(y_max + y_min)

            self.log.info(f"Got pixel height = {dy}, and pixel center at ({center_x}, {center_y})")
            
            tmp = (self.cx -center_x) / self.fx
            theta = numpy.arctan(tmp)
            d = self.true_height * self.fy / (dy * numpy.cos(theta))

            self.landmark_pub.publish(Landmark(header=msg.header, distance=d, bearing=theta, signature=self.landmark_color))

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
