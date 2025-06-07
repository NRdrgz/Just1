import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import qos_profile_sensor_data


class ScanImuSync(Node):
    """
    Node for synchronizing scan and imu data.
    """

    def __init__(self):
        super().__init__("scan_imu_sync")
        self.scan_sub = Subscriber(
            self, LaserScan, "/scan", qos_profile=qos_profile_sensor_data
        )
        self.imu_sub = Subscriber(
            self, Imu, "/imu/data", qos_profile=qos_profile_sensor_data
        )

        self.ts = ApproximateTimeSynchronizer(
            [self.scan_sub, self.imu_sub], queue_size=100, slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        self.scan_pub = self.create_publisher(LaserScan, "/scan/synced", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/synced", 10)

    def sync_callback(self, scan, imu):
        self.scan_pub.publish(scan)
        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = ScanImuSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
