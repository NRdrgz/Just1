import cv2

# Open the video device
cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

# Set resolution (optional)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Allow the camera to warm up
ret, frame = cap.read()

if ret:
    # Save the frame as an image
    cv2.imwrite("capture.jpg", frame)
    print("Image saved as capture.jpg")
else:
    print("Failed to capture image")

# Release the device
cap.release()
