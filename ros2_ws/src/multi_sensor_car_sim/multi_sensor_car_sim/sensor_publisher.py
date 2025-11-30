#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState, LaserScan
from geometry_msgs.msg import Twist
import random

class MultiSensorNode(Node):
    """
    ROS2 node that simulates multiple sensors: battery state, lidar, and wheel speed.
    Publishes messages at a fixed rate.
    """

    def __init__(self):
        super().__init__('multi_sensor_node')

        # Publishers
        self.battery_pub = self.create_publisher(BatteryState, '/battery_state', 10)
        self.lidar_pub = self.create_publisher(LaserScan, '/lidar_front', 10)
        self.wheel_pub = self.create_publisher(Twist, '/wheel_speed', 10)

        # Timer for periodic publishing
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_sensors)

        self.get_logger().info('MultiSensorNode initialized.')

    def publish_sensors(self):
        """Publish simulated sensor data for battery, lidar, and wheel speed."""

        # Battery state
        battery_msg = BatteryState()
        battery_msg.voltage = round(random.uniform(11.0, 12.6), 2)
        battery_msg.current = round(random.uniform(0.5, 2.0), 2)
        battery_msg.percentage = random.uniform(0.3, 1.0)
        self.battery_pub.publish(battery_msg)

        # Lidar data (front scan)
        lidar_msg = LaserScan()
        lidar_msg.angle_min = -1.57
        lidar_msg.angle_max = 1.57
        lidar_msg.angle_increment = 0.01
        lidar_msg.range_min = 0.1
        lidar_msg.range_max = 10.0
        lidar_msg.ranges = [random.uniform(0.1, 10.0) for _ in range(int((lidar_msg.angle_max - lidar_msg.angle_min)/lidar_msg.angle_increment))]
        self.lidar_pub.publish(lidar_msg)

        # Wheel speed (Twist message)
        wheel_msg = Twist()
        wheel_msg.linear.x = random.uniform(0.0, 5.0)
        wheel_msg.angular.z = random.uniform(-1.0, 1.0)
        self.wheel_pub.publish(wheel_msg)

        # Logging to console
        self.get_logger().info(
            f'Published battery: {battery_msg.voltage}V, lidar ranges: {len(lidar_msg.ranges)}, wheel speed: {wheel_msg.linear.x:.2f} m/s'
        )

def main(args=None):
    rclpy.init(args=args)
    node = MultiSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down MultiSensorNode.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

