
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64
from prob_rob_msgs.msg import Point2DArrayStamped
from geometry_msgs.msg import Vector3Stamped

from .landmark_pixel import landmark_pixel


class LandmarkMeasurement(Node):

    def __init__(self):
        super().__init__('landmark_measurement')

        # Node parameters
        self.declare_parameter('landmark_color', 'cyan')
        self.declare_parameter('landmark_height', 0.5)
        self.declare_parameter('min_distance', 0.95)
        self.declare_parameter('max_distance', 4.5)
        self.declare_parameter('max_abs_bearing', 0.52)

        color = self.get_parameter('landmark_color').value
        self.height = self.get_parameter('landmark_height').value
        self.min_distance = self.get_parameter('min_distance').value
        self.max_distance = self.get_parameter('max_distance').value
        self.max_abs_bearing = self.get_parameter('max_abs_bearing').value

        # Camera parameters
        self.fx = None
        self.fy = None
        self.cx = None
        self.image_width = None
        
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_callback, qos_profile_sensor_data)
        self.create_subscription(Point2DArrayStamped, f'/vision_{color}/corners', self.corners_callback, qos_profile_sensor_data)

        self.distance_pub = self.create_publisher(Float64, f'/landmark_{color}/distance', 10)
        self.bearing_pub = self.create_publisher(Float64, f'/landmark_{color}/bearing', 10)
        self.measurement_pub = self.create_publisher(Vector3Stamped, f'/landmark_{color}/measurement', 10)

    def camera_callback(self, msg):
        # Projection matrix P
        self.fx = msg.p[0]
        self.fy = msg.p[5]
        self.cx = msg.p[2]
        self.image_width = msg.width

    def corners_callback(self, msg):
        # valid camera parameters
        if self.fx is None or self.fx <= 0 or self.fy <= 0:
            return
        # pixel height and center
        result = landmark_pixel(msg.points)
        if result is None:
            return
        height_px, center_x = result

        # reject corners touching the image boundary
        xs = [point.x for point in msg.points]
        margin = 2

        if (min(xs) <= margin or max(xs) >= self.image_width - margin):
            return

        # calculate bearing and distance, discard invalid measurements
        bearing = math.atan2(self.cx - center_x, self.fx)
        distance = (self.height * self.fy / (height_px * math.cos(bearing)))
        if (
            not math.isfinite(distance)
            or not (self.min_distance < distance < self.max_distance)
            or abs(bearing) >= self.max_abs_bearing
        ):
            return

        self.distance_pub.publish(Float64(data=distance))
        self.bearing_pub.publish(Float64(data=bearing))

        self.get_logger().info(f'distance: {distance:.3f} m, ' f'bearing: {bearing:.3f} rad')
        self.distance_pub.publish(Float64(data=distance))

        # publish distance and bearing together with a timestamp
        measurement_msg = Vector3Stamped()
        measurement_msg.header = msg.header
        measurement_msg.vector.x = distance
        measurement_msg.vector.y = bearing
        self.measurement_pub.publish(measurement_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LandmarkMeasurement()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
