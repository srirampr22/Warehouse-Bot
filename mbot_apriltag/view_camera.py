import cv2
import time
from gst_cam import camera

w, h = 1280, 720
cap = cv2.VideoCapture(camera(0, w, h))
display_width, display_height = 320, 240

time.sleep(3)

while True:
    ret, frame = cap.read()
    # flip for mirror image
    resized_frame = cv2.resize(frame, (display_width, display_height))
    resized_frame = cv2.flip(resized_frame, 1)

    if not ret:
        break

    cv2.imshow("Camera", resized_frame)

    key = cv2.waitKey(10)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
cap.release()
