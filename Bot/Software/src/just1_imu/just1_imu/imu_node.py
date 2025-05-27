import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import smbus
import math

# MPU6050 registers and addresses
DEVICE_ADDRESS = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

# Initialize the I2C bus
bus = smbus.SMBus(1)
# Wake up the MPU6050 by writing 0 to the PWR_MGMT_1 register
bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 0)

# Read a word from the MPU6050
# We can only read one byte at a time and MPU6050 writes data with 2 bytes
# So we need to read the high and low bytes separately
def read_word(register):
    high = bus.read_byte_data(DEVICE_ADDRESS, register)
    low = bus.read_byte_data(DEVICE_ADDRESS, register + 1)
    # Shift the high byte 8 bits to the left and add the low byte
    value = (high << 8) + low
    # If the value is negative, we need to convert it to a positive value (two's complement)
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

# Get the IMU data
def get_imu_data():
    accel_x = read_word(ACCEL_XOUT_H)
    accel_y = read_word(ACCEL_XOUT_H + 2)
    accel_z = read_word(ACCEL_XOUT_H + 4)
    gyro_x = read_word(ACCEL_XOUT_H + 8)
    gyro_y = read_word(ACCEL_XOUT_H + 10)
    gyro_z = read_word(ACCEL_XOUT_H + 12)
    return {
        'accel': (accel_x, accel_y, accel_z),
        'gyro': (gyro_x, gyro_y, gyro_z)
    }

# Define the ROS2 node that will publish IMU data
class MPU6050Publisher(Node):
    def __init__(self):
        super().__init__('mpu6050_publisher')
        # Create a publisher for the Imu message on the topic 'imu/data_raw'
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        # Set a timer to call the callback at 10 Hz (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Get the IMU data
        data = get_imu_data()
        accel_x, accel_y, accel_z = data['accel']
        gyro_x, gyro_y, gyro_z = data['gyro']

        # Convert raw accelerometer values to m/s²
        # MPU6050 default sensitivity is 16384 LSB/g, and 1g = 9.80665 m/s²
        accel_scale = 9.80665 / 16384.0
        # Convert raw gyro values to rad/s
        # MPU6050 default sensitivity is 131 LSB/(°/s), and we convert deg/s to rad/s
        gyro_scale = math.pi / (180.0 * 131.0)

        # Create an Imu message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        # Fill linear acceleration (in m/s²)
        imu_msg.linear_acceleration.x = accel_x * accel_scale
        imu_msg.linear_acceleration.y = accel_y * accel_scale
        imu_msg.linear_acceleration.z = accel_z * accel_scale

        # Fill angular velocity (in rad/s)
        imu_msg.angular_velocity.x = gyro_x * gyro_scale
        imu_msg.angular_velocity.y = gyro_y * gyro_scale
        imu_msg.angular_velocity.z = gyro_z * gyro_scale

        # Orientation is left unset (default zeros) because we're not computing it here

        # Publish the IMU message
        self.publisher_.publish(imu_msg)
        self.get_logger().info('Published IMU data')

def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
