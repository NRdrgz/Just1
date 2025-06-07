import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math
import time
from typing import Dict, Tuple


class MPU6050Error(Exception):
    """Custom exception for MPU6050 related errors."""

    pass


class MPU6050Publisher(Node):
    """
    ROS2 node for publishing MPU6050 IMU data.

    This node reads data from the MPU6050 IMU sensor and publishes it as ROS2 Imu messages.
    It includes calibration, error handling, and configurable parameters.
    """

    # MPU6050 registers and addresses
    DEVICE_ADDRESS = 0x68
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_CONFIG = 0x1B
    ACCEL_CONFIG = 0x1C

    def __init__(self):
        super().__init__("mpu6050_publisher")

        # Declare parameters
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("publish_rate", 15)
        self.declare_parameter("calibration_samples", 100)
        self.declare_parameter("gyro_range", 250)  # in degrees/s
        self.declare_parameter("accel_range", 2)  # in g

        # Get parameters
        self.i2c_bus = self.get_parameter("i2c_bus").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.calibration_samples = self.get_parameter("calibration_samples").value

        # Initialize I2C bus
        try:
            self.bus = smbus.SMBus(self.i2c_bus)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize I2C bus: {str(e)}")
            raise MPU6050Error("I2C bus initialization failed")

        # Initialize sensor
        self._initialize_sensor()

        # Calibration offsets
        self.accel_offsets = (0.0, 0.0, 0.0)
        self.gyro_offsets = (0.0, 0.0, 0.0)

        # Perform calibration
        self._calibrate_sensor()

        # Create publisher
        self.publisher_ = self.create_publisher(Imu, "imu/data_raw", 10)

        # Create timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info("MPU6050 publisher node initialized successfully")

    def _initialize_sensor(self):
        """Initialize the MPU6050 sensor with proper configuration."""
        try:
            # Wake up the MPU6050
            self.bus.write_byte_data(self.DEVICE_ADDRESS, self.PWR_MGMT_1, 0)

            # Configure gyroscope range
            gyro_range = self.get_parameter("gyro_range").value
            gyro_config = {250: 0x00, 500: 0x08, 1000: 0x10, 2000: 0x18}.get(
                gyro_range, 0x00
            )
            self.bus.write_byte_data(self.DEVICE_ADDRESS, self.GYRO_CONFIG, gyro_config)

            # Configure accelerometer range
            accel_range = self.get_parameter("accel_range").value
            accel_config = {2: 0x00, 4: 0x08, 8: 0x10, 16: 0x18}.get(accel_range, 0x00)
            self.bus.write_byte_data(
                self.DEVICE_ADDRESS, self.ACCEL_CONFIG, accel_config
            )

        except Exception as e:
            self.get_logger().error(f"Failed to initialize MPU6050: {str(e)}")
            raise MPU6050Error("Sensor initialization failed")

    def _calibrate_sensor(self):
        """Calibrate the sensor by collecting offset values."""
        self.get_logger().info("Starting sensor calibration...")

        accel_sum = [0.0, 0.0, 0.0]
        gyro_sum = [0.0, 0.0, 0.0]

        for _ in range(self.calibration_samples):
            data = self._read_sensor_data()
            accel_sum = [a + b for a, b in zip(accel_sum, data["accel"])]
            gyro_sum = [a + b for a, b in zip(gyro_sum, data["gyro"])]
            time.sleep(0.01)  # Small delay between readings

        # Calculate average offsets
        self.accel_offsets = tuple(-x / self.calibration_samples for x in accel_sum)
        self.gyro_offsets = tuple(-x / self.calibration_samples for x in gyro_sum)

        self.get_logger().info("Sensor calibration completed")

    def _read_word(self, register: int) -> int:
        """
        Read a word from the MPU6050.

        We can only read one byte at a time and MPU6050 writes data with 2 bytes.
        So we need to read the high and low bytes separately.
        The value is stored in two's complement format, so we need to convert it properly.
        """
        try:
            # Read the high byte (most significant byte)
            high = self.bus.read_byte_data(self.DEVICE_ADDRESS, register)
            # Read the low byte (least significant byte)
            low = self.bus.read_byte_data(self.DEVICE_ADDRESS, register + 1)
            # Shift the high byte 8 bits to the left and add the low byte
            value = (high << 8) + low
            # If the value is negative (MSB is 1), we need to convert it from two's complement
            if value >= 0x8000:
                value = -((65535 - value) + 1)
            return value
        except Exception as e:
            self.get_logger().error(f"Failed to read from MPU6050: {str(e)}")
            raise MPU6050Error("Sensor read failed")

    def _read_sensor_data(self) -> Dict[str, Tuple[float, float, float]]:
        """Read raw sensor data."""
        try:
            accel_x = self._read_word(self.ACCEL_XOUT_H)
            accel_y = self._read_word(self.ACCEL_XOUT_H + 2)
            accel_z = self._read_word(self.ACCEL_XOUT_H + 4)
            gyro_x = self._read_word(self.ACCEL_XOUT_H + 8)
            gyro_y = self._read_word(self.ACCEL_XOUT_H + 10)
            gyro_z = self._read_word(self.ACCEL_XOUT_H + 12)

            return {
                "accel": (accel_x, accel_y, accel_z),
                "gyro": (gyro_x, gyro_y, gyro_z),
            }
        except Exception as e:
            self.get_logger().error(f"Failed to read sensor data: {str(e)}")
            raise MPU6050Error("Sensor data read failed")

    def timer_callback(self):
        """Timer callback to read and publish IMU data."""
        try:
            # Get the IMU data
            data = self._read_sensor_data()
            accel_x, accel_y, accel_z = data["accel"]
            gyro_x, gyro_y, gyro_z = data["gyro"]

            # Apply calibration offsets
            accel_x += self.accel_offsets[0]
            accel_y += self.accel_offsets[1]
            accel_z += self.accel_offsets[2]
            gyro_x += self.gyro_offsets[0]
            gyro_y += self.gyro_offsets[1]
            gyro_z += self.gyro_offsets[2]

            # Convert raw accelerometer values to m/s²
            accel_scale = 9.80665 / 16384.0
            # Convert raw gyro values to rad/s
            gyro_scale = math.pi / (180.0 * 131.0)

            # Create an Imu message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            # Fill linear acceleration (in m/s²)
            imu_msg.linear_acceleration.x = accel_x * accel_scale
            imu_msg.linear_acceleration.y = accel_y * accel_scale
            imu_msg.linear_acceleration.z = accel_z * accel_scale

            # Fill angular velocity (in rad/s)
            imu_msg.angular_velocity.x = gyro_x * gyro_scale
            imu_msg.angular_velocity.y = gyro_y * gyro_scale
            imu_msg.angular_velocity.z = gyro_z * gyro_scale

            # Publish the IMU message
            self.publisher_.publish(imu_msg)

        except MPU6050Error as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in timer callback: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MPU6050Publisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
