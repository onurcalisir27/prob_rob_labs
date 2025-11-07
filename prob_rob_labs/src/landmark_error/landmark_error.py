import rclpy
from rclpy.node import Node
import math
from message_filters import Subscriber, ApproximateTimeSynchronizer
from prob_rob_msgs.msg import Landmark
import csv
from datetime import datetime

class LandmarkError(Node):

    def __init__(self):
        super().__init__('landmark_error')
        self.log = self.get_logger()
        # self.timer = self.create_timer(heartbeat_period, self.heartbeat)

        self.landmark_gt_sub = Subscriber(self, Landmark, "/landmark_gt")
        self.landmark_sub = Subscriber(self, Landmark, "/landmark")

        self.landmark_error_pub = self.create_publisher(Landmark, "/landmark_error", 10)

        # Slop is tunable
        queue_size = 20
        slop = 0.1
        self.time_sync = ApproximateTimeSynchronizer([self.landmark_gt_sub,
                                                      self.landmark_sub],
                                                     queue_size,
                                                     slop)
        self.time_sync.registerCallback(self.SyncCallback)

        # CSV
        csv_stamp = datetime.now().strftime("%H%M%S")
        self.csv_file = open(f'measurement_errors_{csv_stamp}.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'measured_range', 'measured_bearing',
            'true range', 'true_bearing',
            'range_error', 'bearing_error'
        ])

    def SyncCallback(self, landmark_gt, landmark):

        self.distance_error = abs(landmark_gt.distance - landmark.distance)
        self.bearing_error = landmark_gt.bearing - landmark.bearing

        self.log.info(f"The measured distance to the landmark was: {landmark.distance} which was off by {self.distance_error}")
        self.log.info(f"The measured bearing to the landmark was: {landmark.bearing} which was off by {self.bearing_error}")

        timestamp = landmark.header
        self.landmark_error_pub.publish(Landmark(header=timestamp, distance=self.distance_error, bearing=self.bearing_error))

        self.csv_writer.writerow([
             landmark.distance,
             landmark.bearing,
             landmark_gt.distance,
             landmark_gt.bearing,
             self.distance_error,
             self.bearing_error
        ])


    def spin(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    landmark_error = LandmarkError()
    landmark_error.spin()
    landmark_error.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
