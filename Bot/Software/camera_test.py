from picamera2 import Picamera2
import time

# Initialize the camera
picam2 = Picamera2()

# Configure the camera
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)

# Start the camera
picam2.start()

# Allow the camera to warm up
time.sleep(2)

# Capture an image
image = picam2.capture_array()
print(image.shape)


# Stop the camera
picam2.stop()
