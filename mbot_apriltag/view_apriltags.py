# import cv2
# import time
# import numpy as np
# import yaml
# from dt_apriltags import Detector
# from gst_cam import camera
# import sys
# import lcm
# import threading

# # mbot
# sys.path.append('/usr/lib/python3.9/site-packages/')
# from mbot_lcm_msgs.pose2D_t import pose2D_t 
# from mbot_lcm_msgs.path2D_t import path2D_t 

# current_pose = None  # Global variable to store the current pose
# stop_odom = False

# # mbot

# #Coordinate system: 
# # Camera: Z = away from camera, X = to the right of image of camera, Y = towards the ground of image of camera
# # Tag: Z = into tag, X = to the right of tag, Y = towards the ground of tag


# SMALL_TAG_IDS = [10, 20, 30, 40, 50, 60, 70, 80]
# SMALL_TAG_SIZE_MM = 10.8
# LARGE_TAG_IDS = [1, 2, 3, 4, 5, 6, 7, 8]
# LARGE_TAG_SIZE_MM = 54

# CAMERA_WIDTH = 1280
# CAMERA_HEIGHT = 720

# display_width, display_height = 320, 240
# tags = None
# tag_id = []
# dist, angle, azimuth = [], [], []
# has_block = False

# # Load camera parameters from yaml
# def get_camera_params():
#     with open("camera_params.yaml", 'r') as f:
#         params = yaml.safe_load(f)

#     camera_matrix = np.array(params['camera_matrix'], dtype=np.float32)
#     K = camera_matrix
#     fx = K[0, 0]
#     fy = K[1, 1]
#     cx = K[0, 2]
#     cy = K[1, 2]
#     camera_params = [fx, fy, cx, cy]
#     distortion_coefficients = np.array(params['distortion_coefficients'], dtype=np.float32)
#     return camera_matrix, camera_params, distortion_coefficients

# #Using pose R and t of detected apriltags, return the angle and distance of the tag from the camera
# # Angle = angle between camera's Z axis and tag's Z axis, in degrees
# # Distance = XZ plane distance between camera and tag, in meters
# def get_angle_and_dist_of_tag(tag_id, pose_R_mat, pose_t_mat):
#     distance = np.sqrt(float(pose_t_mat[0]**2 + pose_t_mat[2]**2))
#     if tag_id in SMALL_TAG_IDS:
#         distance *= (SMALL_TAG_SIZE_MM / LARGE_TAG_SIZE_MM) #Detector assumes everything is large tag size. Scale down distance for small tags
#     yaw = np.arctan2(pose_R_mat[0][2], pose_R_mat[2][2]) * 180 / np.pi

#     # Calculate azimuth angle
#     azimuth = np.arctan2(pose_t_mat[0], pose_t_mat[2]) * 180 / np.pi

#     return distance, yaw, azimuth

# # def mbot_pose_listener(lc):
# def pose_handler(channel, data):
#     global current_pose
#     msg = pose2D_t.decode(data)
#     # print("Received message on channel \"%s\"\n   timestamp   = %s\n   x           = %s\n   y           = %s\n   theta       = %s\n" % (channel, str(msg.utime), str(msg.x), str(msg.y), str(msg.theta)))
#     current_pose = msg


# def calculate_goal_pose1(current_pose, distance, angle, azimuth):

#     angle_rad = np.radians(angle)

#     # goal_x = current_pose.x + distance * np.cos(current_pose.theta + angle_rad)
#     # goal_y = current_pose.y + distance * np.sin(current_pose.theta + angle_rad)

#     rajni = distance * np.cos(180 - angle_rad)

#     goal_x = current_pose.x 
#     goal_y = current_pose.y + rajni

#     goal_theta = 0

#     # Create a new pose object or return a tuple
#     goal_pose = pose2D_t()  # if pose2D_t is a class
#     goal_pose.x = goal_x
#     goal_pose.y = goal_y
#     goal_pose.theta = goal_theta

#     chitra = np.sqrt(distance**2 + rajni**2)

#     return goal_pose, chitra

# def calculate_goal_pose2(goal_pose_1, chitra, offset = 0.5):

#     # angle_rad = np.radians(angle)

#     # goal_x = current_pose.x + distance * np.cos(current_pose.theta + angle_rad)
#     # goal_y = current_pose.y + distance * np.sin(current_pose.theta + angle_rad)

#     goal_x = goal_pose_1.x 
#     goal_y = goal_pose_1.y + (chitra -  offset)

#     goal_theta = np.pi/2

#     # Create a new pose object or return a tuple
#     goal_pose_2 = pose2D_t()  # if pose2D_t is a class
#     goal_pose_2.x = goal_x
#     goal_pose_2.y = goal_y 
#     goal_pose_2.theta = goal_theta

#     return goal_pose_2

# def publish_goal_path(lc, goal_pose_1, goal_pose_2, current_pose):
#     goal_path = path2D_t()
#     goal_path.path.append(current_pose)
#     goal_path.path.append(goal_pose_1)
#     goal_path.path.append(goal_pose_2)
#     goal_path.path_length = 3
#     lc.publish("CONTROLLER_PATH", goal_path.encode())

# def lcm_handler_thread(lc):
#     global stop_odom
#     while stop_odom is False:
#         lc.handle()


# def capture_and_process_frames():
#     cap = cv2.VideoCapture(camera(0, CAMERA_WIDTH, CAMERA_HEIGHT))  # Modify the camera index as needed
#     camera_matrix, camera_params, distortion_coefficients = get_camera_params()
#     global tags
#     global tag_id
#     global dist, angle, azimuth


#     detector = Detector(searchpath=['apriltags3py/apriltags/lib', 'apriltags3py/apriltags/lib'],
#                             families='tagCustom48h12',
#                             nthreads=4,
#                             quad_decimate=2,
#                             quad_sigma=0.4,
#                             refine_edges=1,
#                             decode_sharpening=1,
#                             max_hamming=1,
#                             debug=0)

#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             break

#         # Your frame processing logic here
#         undistort_frame = cv2.undistort(frame, camera_matrix, distortion_coefficients)
#         gray = cv2.cvtColor(undistort_frame, cv2.COLOR_BGR2GRAY)
#         tags = detector.detect(gray, True, camera_params, LARGE_TAG_SIZE_MM * (1/1000))

#         resized_frame = cv2.resize(undistort_frame, (display_width, display_height))
#         # resized_frame = cv2.flip(resized_frame, 1)

#         for det in tags:
#             # Draw bounding box
#             for i in range(4):
#                 start_point = tuple(det.corners[i-1].astype(int))
#                 end_point = tuple(det.corners[i].astype(int))
#                 cv2.line(resized_frame, start_point, end_point, (0, 255, 0), 2)

#             # Draw tag family and ID on the image
#             tag_id.append(det.tag_id)
#             dist, angle, azimuth = get_angle_and_dist_of_tag(det.tag_id, det.pose_R, det.pose_t)
#             # tag_info = "ID {}: angle {}, distance {}, azimuth {}".format(det.tag_id, angle, dist, azimuth)
#             tag_info = "ID: {}".format(det.tag_id)

#             cv2.putText(resized_frame, tag_info, (int(det.center[0]), int(det.center[1])), 
#             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
#             print(f"ID {det.tag_id}: angle {angle}, distance {dist}, azimuth {azimuth}")

#         cv2.imshow("Camera", resized_frame)

#         key = cv2.waitKey(1)
#         if key == ord('q'):
#             break

#     cap.release()
#     cv2.destroyAllWindows()
    
# def main():
#     global tags
#     global tag_id
#     global dist, angle, azimuth
#     global current_pose

#     lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
#     subscription = lc.subscribe("MBOT_ODOMETRY", pose_handler)
#     threading.Thread(target=lcm_handler_thread, args=(lc,)).start() 
#     camera_thread = threading.Thread(target=capture_and_process_frames)
#     camera_thread.start()
#     time.sleep(10)

#     # angles = [0, np.pi/9, 2*np.pi/9, 3*np.pi/9, 4*np.pi/9, 5*np.pi/9, 6*np.pi/9,  7*np.pi/9, 8*np.pi/9, np.pi,                 
#     # -8*np.pi/9, -7*np.pi/9, -6*np.pi/9, -5*np.pi/9, -4*np.pi/9, -3*np.pi/9, -2*np.pi/9, -np.pi/9]

#     # iteration_num = 0 
#     # while True:
#     #     print(f"Main loop iteration {iteration_num}") 
        
#     #     new_pose = pose2D_t()
#     #     new_pose.x = current_pose.x
#     #     new_pose.y = current_pose.y
#     #     new_pose.theta = angles[iteration_num]
#     #     rotate_path = path2D_t()
#     #     rotate_path.path.append(current_pose)
#     #     rotate_path.path.append(new_pose)
#     #     rotate_path.path_length = 2
#     #     lc.publish("CONTROLLER_PATH", rotate_path.encode())
#     #     time.sleep(10)
#     #     print("Path complete")

#     #     # print(tags)
#     #     if(iteration_num == (len(angles) - 1)):
#     #         break

#     #     if(tags == []):
#     #         print("Tags not found")
#     #         iteration_num += 1
#     #         continue
      
#     #     for new_tag in tag_id:
#     #         if new_tag in LARGE_TAG_IDS and (new_tag % 2 != 0) and (has_block is False):
#     #             goal_pose_1, chitra = calculate_goal_pose1(current_pose, dist, angle, azimuth)
#     #             goal_pose_2 = calculate_goal_pose2(goal_pose_1, chitra, offset = 0.25)

#     #             # Publish goal pose
#     #             print(f"Mid pose: {goal_pose_1.x}, {goal_pose_1.y}, {goal_pose_1.theta}")
#     #             print(f"Goal pose: {goal_pose_2.x}, {goal_pose_2.y}, {goal_pose_2.theta}")
#     #             while(abs(current_pose.x - goal_pose_2.x) > 0.1 and abs(current_pose.y - goal_pose_2.y) > 0.1):
#     #                 publish_goal_path(lc, goal_pose_1, goal_pose_2, current_pose)

#     #             LARGE_TAG_IDS.remove(tag_id)
#     #             SMALL_TAG_IDS.remove(tag_id*10)
#     #             has_block = True
#     #             break
#     #         elif new_tag in LARGE_TAG_IDS and has_block:
#     #             goal_pose_1, chitra = calculate_goal_pose1(current_pose, dist, angle, azimuth)
#     #             goal_pose_2 = calculate_goal_pose2(goal_pose_1, chitra, offset = 0.0)

#     #             # Publish goal pose
#     #             print(f"Mid pose: {goal_pose_1.x}, {goal_pose_1.y}, {goal_pose_1.theta}")
#     #             print(f"Goal pose: {goal_pose_2.x}, {goal_pose_2.y}, {goal_pose_2.theta}")
#     #             while(abs(current_pose.x - goal_pose_2.x) > 0.1 and abs(current_pose.y - goal_pose_2.y) > 0.1):
#     #                 publish_goal_path(lc, goal_pose_1, goal_pose_2, current_pose)

#     #             LARGE_TAG_IDS.remove(tag_id)
#     #             SMALL_TAG_IDS.remove(tag_id*10)
#     #             has_block = False
#     #             break


#     #     iteration_num += 1

#     global stop_odom
#     stop_odom = True

#     return 0

# if __name__ == "__main__":
#     main()


import cv2
import time
import numpy as np
import yaml
from dt_apriltags import Detector
from gst_cam import camera

#Coordinate system: 
# Camera: Z = away from camera, X = to the right of image of camera, Y = towards the ground of image of camera
# Tag: Z = into tag, X = to the right of tag, Y = towards the ground of tag


SMALL_TAG_IDS = [10, 20, 30, 40, 50, 60, 70, 80]
SMALL_TAG_SIZE_MM = 10.8
LARGE_TAG_IDS = [1, 2, 3, 4, 5, 6, 7, 8]
LARGE_TAG_SIZE_MM = 54

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# Load camera parameters from yaml
def get_camera_params():
    with open("camera_params.yaml", 'r') as f:
        params = yaml.safe_load(f)

    camera_matrix = np.array(params['camera_matrix'], dtype=np.float32)
    K = camera_matrix
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    camera_params = [fx, fy, cx, cy]
    distortion_coefficients = np.array(params['distortion_coefficients'], dtype=np.float32)
    return camera_matrix, camera_params, distortion_coefficients

#Using pose R and t of detected apriltags, return the angle and distance of the tag from the camera
# Angle = angle between camera's Z axis and tag's Z axis, in degrees
# Distance = XZ plane distance between camera and tag, in meters
def get_angle_and_dist_of_tag(tag_id, pose_R_mat, pose_t_mat):
    distance = np.sqrt(float(pose_t_mat[0]**2 + pose_t_mat[2]**2))
    if tag_id in SMALL_TAG_IDS:
        distance *= (SMALL_TAG_SIZE_MM / LARGE_TAG_SIZE_MM) #Detector assumes everything is large tag size. Scale down distance for small tags
    yaw = np.arctan2(pose_R_mat[0][2], pose_R_mat[2][2]) * 180 / np.pi
    return distance, yaw
    
def main():
    camera_matrix, camera_params, distortion_coefficients = get_camera_params()
    cap = cv2.VideoCapture(camera(0, CAMERA_WIDTH, CAMERA_HEIGHT))

    time.sleep(3)

    # Initialize AprilTag detector
    detector = Detector(searchpath=['apriltags3py/apriltags/lib', 'apriltags3py/apriltags/lib'],
                            families='tagCustom48h12',
                            nthreads=4,
                            quad_decimate=2,
                            quad_sigma=0.4,
                            refine_edges=1,
                            decode_sharpening=1,
                            max_hamming=1,
                            debug=0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.undistort(frame, camera_matrix, distortion_coefficients)

        # Detect AprilTags
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray, True, camera_params, LARGE_TAG_SIZE_MM * (1/1000))
        # print(tags)

        for det in tags:
            # Draw bounding box
            for i in range(4):
                start_point = tuple(det.corners[i-1].astype(int))
                end_point = tuple(det.corners[i].astype(int))
                cv2.line(frame, start_point, end_point, (0, 255, 0), 2)

            # Draw tag family and ID on the image
            dist, angle = get_angle_and_dist_of_tag(det.tag_id, det.pose_R, det.pose_t)
            tag_info = "ID {}:  angle {:.4f}, distance {:.4f}".format(det.tag_id, angle, dist)
            if det.tag_id in LARGE_TAG_IDS:
                cv2.putText(frame, tag_info, (int(det.center[0]), int(det.center[1])), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            print(f"ID {det.tag_id}: angle {angle}, distance {dist}")

        cv2.imshow("Camera", frame)

        key = cv2.waitKey(10)
        if key == ord('q'):
            break

    cv2.destroyAllWindows()
    cap.release()

if __name__ == "__main__":
    main()