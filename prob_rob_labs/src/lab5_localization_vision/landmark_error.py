# landmark_error.py

import math
import csv
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Float64
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener, TransformException


# landmark positions from landmarks.world
LANDMARKS = {
    'cyan': (0.0, 0.0),
    'red': (8.5, -5.0),
    'green': (8.5, 5.0),
    'yellow': (-11.5, 5.0),
    'magenta': (-11.5, -5.0),
}

def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def yaw_from_quaternion(q):
    return math.atan2(2*(q.w*q.z +q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class LandmarkError(Node):
    def __init__(self):
        super().__init__('landmark_error')
        self.declare_parameter('landmark_color', 'cyan')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'camera_rgb_frame')
        self.declare_parameter('output_csv', '')
        color = self.get_parameter('landmark_color').value.lower()
        if color not in LANDMARKS:
            raise ValueError(f'Unknown landmark color: {color}')

        self.landmark_x, self.landmark_y = LANDMARKS[color]
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        output_csv = self.get_parameter('output_csv').value
        if not output_csv:
            output_csv = f'lab5_{color}_{datetime.now():%Y%m%d_%H%M%S}.csv'
        self.csv_file = open(output_csv, 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'time',
            'measured_distance',
            'measured_bearing',
            'true_distance',
            'true_bearing',
            'distance_error',
            'bearing_error'
        ])
        self.get_logger().info(f'Saving CSV to {output_csv}')

        # TF: camera pose relative to robot base
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.distance_error_pub = self.create_publisher(Float64, f'/landmark_{color}/error/distance', 10)
        self.bearing_error_pub = self.create_publisher(Float64, f'/landmark_{color}/error/bearing', 10)

        self.measurement_sub = Subscriber(self, Vector3Stamped, f'/landmark_{color}/measurement', qos_profile=qos_profile_sensor_data)
        self.gt_sub = Subscriber(self, PoseStamped, '/tb3/ground_truth/pose', qos_profile=qos_profile_sensor_data)

        # synchronize by timestamp
        self.sync = ApproximateTimeSynchronizer([self.measurement_sub, self.gt_sub], queue_size=50, slop=0.10)
        self.sync.registerCallback(self.callback)

        self.n = 0
        self.warned_tf = False
        self.get_logger().info('Landmark Error Analyzer Started')

    def callback(self, measurement, ground_truth):
        try:
            # lookup_transform: from camera to robot base
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, Time())
        except TransformException as exc:
            if not self.warned_tf:
                self.get_logger().warning(f'Camera TF unavailable: {exc}')
                self.warned_tf = True
            return

        # ground-truth robot pose in world frame
        robot_x = ground_truth.pose.position.x
        robot_y = ground_truth.pose.position.y
        robot_yaw = yaw_from_quaternion(ground_truth.pose.orientation)

        # Transform robot pose to camera pose
        offset = transform.transform.translation
        camera_x = (robot_x + math.cos(robot_yaw)*offset.x - math.sin(robot_yaw)*offset.y)
        camera_y = (robot_y + math.sin(robot_yaw)*offset.x + math.cos(robot_yaw)*offset.y)
        camera_yaw = wrap(robot_yaw + yaw_from_quaternion(transform.transform.rotation))

        # true distance and bearing
        dx = self.landmark_x - camera_x
        dy = self.landmark_y - camera_y
        true_distance = math.hypot(dx, dy)
        true_bearing = wrap(math.atan2(dy, dx) - camera_yaw)

        # Measured distance and bearing from Assignment 2
        measured_distance = measurement.vector.x
        measured_bearing = measurement.vector.y

        # Calculate measurement errors
        distance_error = measured_distance - true_distance
        bearing_error = wrap(measured_bearing - true_bearing)

        # Publish errors
        self.distance_error_pub.publish(Float64(data=distance_error))
        self.bearing_error_pub.publish(Float64(data=bearing_error))

        time_s = measurement.header.stamp.sec + measurement.header.stamp.nanosec * 1e-9
        self.writer.writerow([
            time_s,
            measured_distance,
            measured_bearing,
            true_distance,
            true_bearing,
            distance_error,
            bearing_error
        ])

        # Print results every 50 samples
        self.n += 1
        if self.n % 50 == 0:
            self.csv_file.flush()
            self.get_logger().info(
                f'd={measured_distance:.3f} m, '
                f'd_true={true_distance:.3f} m, '
                f'd_error={distance_error:.3f} m | '
                f'bearing={measured_bearing:.3f} rad, '
                f'bearing_true={true_bearing:.3f} rad, '
                f'bearing_error={bearing_error:.3f} rad'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.csv_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
