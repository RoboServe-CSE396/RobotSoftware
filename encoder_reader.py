import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import tf_transformations
from math import pi, cos, sin
import serial
import time

# Define the baud rate for the software serial port
SOFTWARE_SERIAL_BAUD_RATE = 57600

isStartingPoint = True

class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")

        # Parameters
        self.declare_parameter("wheel_base", 0.5)
        self.declare_parameter("wheel_radius", 0.1)
        self.declare_parameter("encoder_ticks_per_rev", 1000)

        self.wheel_base = self.get_parameter("wheel_base").get_parameter_value().double_value
        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.encoder_ticks_per_rev = self.get_parameter("encoder_ticks_per_rev").get_parameter_value().integer_value

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # State variables
        self.encoder1 = 0
        self.encoder2 = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.left_encoder_prev = 0
        self.right_encoder_prev = 0

        # Open serial port
        self.ser = serial.Serial("/dev/ttyUSB1", SOFTWARE_SERIAL_BAUD_RATE)
        time.sleep(5)  # Wait for the serial port to initialize

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer
        self.timer = self.create_timer(0.02, self.publish_odometry)  # 50 Hz update rate

    def compute_odometry(self):
        # Read data from the serial port
        data = self.ser.readline().strip().decode()  # Decode bytes to string
        self.left_encoder_prev = self.encoder1
        self.right_encoder_prev = self.encoder2
        try:
            # Split the string into two integers
            self.encoder1, self.encoder2 = map(int, data.split())
        except ValueError:
            self.get_logger().error(f"Invalid data received from serial port: {data}")
            return None

        # Compute distances
        left_ticks = self.encoder1 - self.left_encoder_prev
        right_ticks = self.encoder2 - self.right_encoder_prev

        left_distance = (2 * pi * self.wheel_radius * left_ticks) / self.encoder_ticks_per_rev
        right_distance = (2 * pi * self.wheel_radius * right_ticks) / self.encoder_ticks_per_rev

        # Compute the change in pose
        delta_s = (right_distance + left_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Update the robot's pose
        self.x += delta_s * cos(self.theta + delta_theta / 2.0)
        self.y += delta_s * sin(self.theta + delta_theta / 2.0)
        self.theta = self.normalize_angle(self.theta + delta_theta)

        # Calculate velocities
        vx = delta_s / dt
        vy = 0.0  # Set vy to 0.0 if not available
        vth = delta_theta / dt

        # Print vx and theta for debugging
        self.get_logger().info(f"x: {round(self.x, 5)}, y: {round(self.y, 5)}, theta: {round(self.theta, 5)}")

        return self.x, self.y, self.theta, vx, vy, vth

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def publish_odometry(self):
        global isStartingPoint
        odom_data = self.compute_odometry()
        if odom_data is None:
            return

        x, y, theta, vx, vy, vth = odom_data

        # Create odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        quaternion = tf_transformations.quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        # Publish the odometry message
        self.odom_pub.publish(odom)

        # Create and publish TF transform message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        if x!= 0 or y!= 0 or isStartingPoint:
            t.transform.translation.x = x
            t.transform.translation.y = y
        if x!= 0 or y!= 0:
            isStartingPoint = False
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(t)

        # Listen to other TF


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()  # Close the serial port when exiting
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
