import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from message_filters import Subscriber, ApproximateTimeSynchronizer
import copy


class ScanImuSync(Node):
    """
    Node for synchronizing scan and imu data, publishing them with the same timestamp.
    """

    def __init__(self):
        super().__init__("scan_imu_sync")
        self.scan_sub = Subscriber(self, LaserScan, "/scan", 10)
        self.imu_sub = Subscriber(self, Imu, "/imu/data", 10)

        self.ts = ApproximateTimeSynchronizer(
            [self.scan_sub, self.imu_sub], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        self.scan_pub = self.create_publisher(LaserScan, "/scan/synced", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/synced", 10)

    def sync_callback(self, scan, imu):
        # Use scan's timestamp as the synchronized time
        synced_time = scan.header.stamp

        # Deep copy and update timestamps
        synced_scan = copy.deepcopy(scan)
        synced_scan.header.stamp = synced_time

        synced_imu = copy.deepcopy(imu)
        synced_imu.header.stamp = synced_time

        self.scan_pub.publish(synced_scan)
        self.imu_pub.publish(synced_imu)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ScanImuSync()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
