import cv2
import time
import numpy as np
import yaml
from dt_apriltags import Detector
from gst_cam import camera
import sys
import lcm
import queue
from multiprocessing import Process, Event, Manager, Lock

# LCM mesages setup 
sys.path.append('/usr/lib/python3.9/site-packages/')
from mbot_lcm_msgs.pose2D_t import pose2D_t 
from mbot_lcm_msgs.path2D_t import path2D_t 

# Create a global variable for the current pose 
current_pose = None 

#Coordinate system NOTES: 
# Camera: Z = away from camera, X = to the right of image of camera, Y = towards the ground of image of camera
# Tag: Z = into tag, X = to the right of tag, Y = towards the ground of tag

# Hardcoded values for apriltag parameters 
SMALL_TAG_IDS = [10, 20, 30, 40, 50, 60, 70, 80]
SMALL_TAG_SIZE_MM = 10.8
LARGE_TAG_IDS = [1, 2, 3, 4, 5, 6, 7, 8]
LARGE_TAG_SIZE_MM = 54

# Hardcoded OpenCV parmaters
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
display_width, display_height = 320, 240

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

    # Calculate azimuth angle
    azimuth = np.arctan2(pose_t_mat[0], pose_t_mat[2]) * 180 / np.pi

    return distance, yaw, azimuth

# LCM listener callback function
def lcm_pose_handler(channel, data, msg_dict):

    # Decode the incoming message 
    msg = pose2D_t.decode(data)

    # Print a response
    # print("Received message on channel \"%s\"\n   timestamp   = %s\n   x           = %s\n   y           = %s\n   theta       = %s\n" % (channel, str(msg.utime), str(msg.x), str(msg.y), str(msg.theta)))
    
    # Put the message on the queue 
    msg_dict["now"] = msg


def lcm_get_pose_thread(msg_dict, stop_event, data_lock): 
    """
        This function defines the thread to be executed as an LCM handler. 
        When messages come in, they are processed, and added to the queue.
    """

    # First create the LCM instance
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

    # Then the subscription 
    subscription = lc.subscribe("MBOT_ODOMETRY", lambda channel, data: lcm_pose_handler(channel, data, msg_dict))

    # Continuously try and poll for new messages on the channel
    try:
        while not stop_event.is_set():
            if data_lock.acquire(block=False):
                lc.handle()
                data_lock.release()
            else: 
                continue
    except KeyboardInterrupt:
        pass

    lc.unsubscribe(subscription)


def calculate_goal_pose1(current_pose, distance, angle, azimuth):

    angle_rad = np.radians(angle)

    # goal_x = current_pose.x + distance * np.cos(current_pose.theta + angle_rad)
    # goal_y = current_pose.y + distance * np.sin(current_pose.theta + angle_rad)

    rajni = distance * np.cos(180 - angle_rad)

    goal_x = current_pose.x 
    goal_y = current_pose.y + rajni

    goal_theta = 0

    # Create a new pose object or return a tuple
    goal_pose = pose2D_t()  # if pose2D_t is a class
    goal_pose.x = goal_x
    goal_pose.y = goal_y
    goal_pose.theta = goal_theta

    chitra = np.sqrt(distance**2 + rajni**2)

    return goal_pose, chitra

def calculate_goal_pose2(goal_pose_1, chitra, offset = 0.5):

    # angle_rad = np.radians(angle)

    # goal_x = current_pose.x + distance * np.cos(current_pose.theta + angle_rad)
    # goal_y = current_pose.y + distance * np.sin(current_pose.theta + angle_rad)

    goal_x = goal_pose_1.x 
    goal_y = goal_pose_1.y + (chitra -  offset)

    goal_theta = np.pi/2

    # Create a new pose object or return a tuple
    goal_pose_2 = pose2D_t()  # if pose2D_t is a class
    goal_pose_2.x = goal_x
    goal_pose_2.y = goal_y 
    goal_pose_2.theta = goal_theta

    return goal_pose_2

# Function to publish paths over LCM 
def publish_goal_path(goal_pose_1, goal_pose_2, current_pose):
    goal_path = path2D_t()
    goal_path.path.append(current_pose)
    goal_path.path.append(goal_pose_1)
    goal_path.path.append(goal_pose_2)

    lc.publish("CONTROLLER_PATH", goal_path.encode())
    

def test_path_sending():

    # Initiate the LCM message pose handler thread. Queue is used for data pushback 
    # First create data manager to handle data locks 
    data_manager = Manager() 
    pose_dict = data_manager.dict()  
    data_lock = Lock() 

    # Start the LCM handling in a separate thread
    pose_thread_stop_event = Event()
    pose_thread = Process(target=lcm_get_pose_thread, args=(pose_dict, pose_thread_stop_event, data_lock))
    pose_thread.start()

    # These are the angles that we want to sample 
    angles = [-2*np.pi/3, -np.pi/3, 0, np.pi/3, 2*np.pi/3, np.pi]

    # Create a second LCM object that just handles publishing paths 
    lcm_path_publisher = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

    iteration_num = 0 
    angle_index = 0
    while True:
        # Get the current pose in the queue. Create a small path from current pose to next pose based on angle index. 
        # Get the lock for the current pose 
        print(f"Getting lock for iteration {iteration_num}") 
        data_lock.acquire(block=True) 
        current_pose = pose_dict
        data_lock.release()

        # Print state just as a sanity check 
        print("SENDING ROTATE PATH TO CONTROLLER. CURRENT POSE IS ", current_pose)
        print("NEXT ANGLE IS: ", angles[angle_index])

        new_pose = pose2D_t()
        new_pose.x = current_pose.x
        new_pose.y = current_pose.y
        new_pose.theta = angles[angle_index]
        rotate_path = path2D_t()
        rotate_path.path.append(current_pose)
        rotate_path.path.append(new_pose)
        rotate_path.path_length = 2
        lcm_path_publisher.publish("CONTROLLER_PATH", rotate_path.encode())

        # Wait for it to get to the next path. 
        time.sleep(1)

        iteration_num += 1
        angle_index += 1

        # Reset to first angle 
        if angle_index == len(angles): 
            angle_index = 0

    cv2.destroyAllWindows()
    cap.release()

    # Close the pose thread
    pose_thread_stop_event.set() # Set the stop event 
    pose_thread.join() # Then join the thread

def main():

    # Start out by getting camera params and opening a capturer for camera 
    camera_matrix, camera_params, distortion_coefficients = get_camera_params()
    cap = cv2.VideoCapture(0)

    # Initiate the LCM message pose handler thread. Queue is used for data pushback 
    # First create data manager to handle data locks 
    data_manager = Manager() 
    pose_dict = data_manager.dict()  

    # Start the LCM handling in a separate thread
    pose_thread_stop_event = Event()
    pose_thread = Process(target=lcm_get_pose_thread, args=(pose_dict, pose_thread_stop_event))
    pose_thread.start()

    # Create an object to record apriltags
    tags = None

    # These are the angles that we want to sample 
    angles = [-2*np.pi/3, -np.pi/3, 0, np.pi/3, 2*np.pi/3, np.pi]

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

    # Create a second LCM object that just handles publishing paths 
    lcm_path_publisher = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

    iteration_num = 0 
    angle_index = 0
    while True:
        # print(f"Main loop iteration {iteration_num}") 

        # First read the current frame into the buffer
        ret, frame = cap.read()
        if not ret:
            break
        
        # Then undistort the image based on the image coefficients. 
        frame = cv2.undistort(frame, camera_matrix, distortion_coefficients)

        # Get the current pose in the queue. Create a small path from current pose to next pose based on angle index. 
        current_pose = pose_dict["now"]

        # Print state just as a sanity check 
        print("SENDING ROTATE PATH TO CONTROLLER. CURRENT POSE IS ", current_pose)
        print("NEXT ANGLE IS: ", angles[angle_index])

        new_pose = pose2D_t()
        new_pose.x = current_pose.x
        new_pose.y = current_pose.y
        new_pose.theta = angles[angle_index]
        rotate_path = path2D_t()
        rotate_path.path.append(current_pose)
        rotate_path.path.append(new_pose)
        rotate_path.path_length = 2
        lcm_path_publisher.publish("CONTROLLER_PATH", rotate_path.encode())

        # Wait for it to get to the next path. 
        time.sleep(1)

        # Detect tags in the current frame
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray, True, camera_params, LARGE_TAG_SIZE_MM * (1/1000))

        # if(tags != []):
        #     break

        tag_id = None
        for det in tags:
            # Draw bounding box
            for i in range(4):
                start_point = tuple(det.corners[i-1].astype(int))
                end_point = tuple(det.corners[i].astype(int))
                cv2.line(frame, start_point, end_point, (0, 255, 0), 2)

            # Draw tag family and ID on the image
            if(det.tag_id in LARGE_TAG_IDS):
                tag_info = "ID:{}".format(det.tag_id)
                tag_id = det.tag_id
                dist, angle, azimuth = get_angle_and_dist_of_tag(det.tag_id, det.pose_R, det.pose_t)

                cv2.putText(frame, tag_info, (int(det.center[0]), int(det.center[1])), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                print(f"ID {det.tag_id}: angle {angle}, distance {dist}, azimuth {azimuth}")

                break

        # Show the frame to the viewer
        print("Showing the current frame") 
        resized_frame = cv2.resize(frame, (display_width, display_height))
        cv2.imshow("Camera", resized_frame)

        # Wait one second to see if someone quits, otherwise continue 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
                  

        iteration_num += 1
        angle_index += 1

        # Reset to first angle 
        if angle_index == len(angles): 
            angle_index = 0

    cv2.destroyAllWindows()
    cap.release()

    # Close the pose thread
    pose_thread_stop_event.set() # Set the stop event 
    pose_thread.join() # Then join the thread


if __name__ == "__main__":
    test_path_sending()
