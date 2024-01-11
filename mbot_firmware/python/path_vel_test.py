#!/usr/bin/env python3

import time
import lcm
import sys
import threading
import numpy as np

sys.path.append('/usr/lib/python3.9/site-packages/')
from mbot_lcm_msgs.path2D_t import path2D_t
from mbot_lcm_msgs.pose2D_t import pose2D_t

current_pose = pose2D_t()  # Global variable to store the current pose

def pose_handler(channel, data):
    global current_pose
    msg = pose2D_t.decode(data)
    # print("Received message on channel \"%s\"\n   timestamp   = %s\n   x           = %s\n   y           = %s\n   theta       = %s\n" % (channel, str(msg.utime), str(msg.x), str(msg.y), str(msg.theta)))
    current_pose = msg

def lcm_handler_thread(lc):
    while True:
        lc.handle()


def path_send(lc):
    global current_pose 
    angles = [ 0, np.pi/3, 2*np.pi/3, np.pi, -2*np.pi/3, -np.pi/3, 0]

    for angle in angles:
        print("Trying to create path for angle", angle)
        command = path2D_t()
        goal_pose = pose2D_t()
        goal_pose.x = current_pose.x
        goal_pose.y = current_pose.y
        goal_pose.theta = angle
        command.path.append(current_pose)
        command.path.append(goal_pose)
        command.path_length = 2
        print("Devised path: ", command)
        lc.publish("CONTROLLER_PATH", command.encode())
        time.sleep(2)
    
    return 0



# Main function
def main():
    # Constants and parameters
    # WHEEL_RADIUS = 0.00415  # meters
    # DUTY_CYCLE = 0.5  # rad/s
    # MAX_SPEED = 15.0

    # DISTANCE_TO_MOVE = 0.5 # meters

    # Initialize LCM
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    subscription = lc.subscribe("MBOT_ODOMETRY", pose_handler)
    threading.Thread(target=lcm_handler_thread, args=(lc,)).start()
    

    # Move the robot forward for the specified distance at a constant velocity

    return path_send(lc)
    
if __name__ == "__main__":
    print("Running")
    main()
