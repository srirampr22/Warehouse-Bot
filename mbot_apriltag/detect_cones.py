import cv2
import time
from gst_cam import camera

w, h = 1280, 720
cap = cv2.VideoCapture(camera(0, w, h))

time.sleep(3)

display_width, display_height = 320, 240

while True:
    ret, frame = cap.read()
    # flip for mirror image
    frame = cv2.flip(frame, 1)

    if not ret:
        break

    # Convert frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of red color in HSV
    lower_red1 = (0, 120, 70)
    upper_red1 = (10, 255, 255)
    lower_red2 = (170, 120, 70)
    upper_red2 = (180, 255, 255)

    # Create masks for red color
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Bitwise-AND mask and original image
    red_detection = cv2.bitwise_and(frame, frame, mask=red_mask)

    # Edge detection on the red-detected image
    edges = cv2.Canny(red_detection, 50, 150, apertureSize=3)

    # Find contours
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        # If the polygon has 3 vertices, it is a triangle
        if len(approx) == 3:
            cv2.drawContours(red_detection, [cnt], 0, (0, 255, 0), 3)  # Drawing on the original frame

    # Resize frame and red_detection for display
    resized_frame = cv2.resize(frame, (display_width, display_height))
    resized_red_detection = cv2.resize(red_detection, (display_width, display_height))

    # Display the original (with triangles) and the red detection images
    cv2.imshow("Camera with Triangles", resized_frame)
    cv2.imshow("Red Detection", resized_red_detection)

    key = cv2.waitKey(10)
    if key == ord('q'):
        break

cv2.destroyAllWindows()
cap.release()
