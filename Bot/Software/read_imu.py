import smbus
import time

# MPU6050 registers and addresses
DEVICE_ADDRESS = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

# Initialize the I2C bus
bus = smbus.SMBus(1)
# Wake up the MPU6050 by writing 0 to the PWR_MGMT_1 register
bus.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 0)

# Read a word from the MPU6050
# We can only read one byte at a time and MPU6050 write data with 2 bytes
# So we need to read the high and low bytes separately
def read_word(register):
    high = bus.read_byte_data(DEVICE_ADDRESS, register)
    low = bus.read_byte_data(DEVICE_ADDRESS, register + 1)
    value = (high << 8) + low
    # If the value is negative, we need to convert it to a positive value
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

while True:
    data = get_imu_data()
    print(f"Accel: {data['accel']}, Gyro: {data['gyro']}")
    time.sleep(0.1)
