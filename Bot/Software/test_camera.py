from cv_bridge import CvBridge
import cv2
import subprocess

bridge = CvBridge()
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
print(cap.isOpened())
ret, frame = cap.read()
print(frame)

msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
print(msg)
