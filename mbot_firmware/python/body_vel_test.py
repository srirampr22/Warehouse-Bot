#!/usr/bin/env python3

import time
import lcm
import sys

sys.path.append('/usr/lib/python3.9/site-packages/')
from mbot_lcm_msgs.twist2D_t import twist2D_t


def move_forward(lc, duty_cycle, wheel_radius, max_speed, dist):
    command = twist2D_t()

    duration = dist / (duty_cycle* max_speed * 2 * 3.1415 * wheel_radius)
    print("Duration: ", duration)
    
    start_time = time.perf_counter()
    end_time = start_time + duration
    
    print("START TIME:", start_time)
    print("END TIME:", end_time)
    print("DUTY CYCLE:", duty_cycle)
    print("VELOCITY:", duty_cycle*max_speed)
    

    while time.perf_counter() <= end_time:
          # Introduce a delay to avoid flooding the channel with too many messages
        command.vx = 0.3
        command.vy = 0.0
        command.wz = 0.0
        lc.publish("MBOT_VEL_CMD", command.encode())
        time.sleep(0.005)


    command.vx = 0
    command.vy = 0
    command.wz = 0
    lc.publish("MBOT_VEL_CMD", command.encode())

# Main function
def main():
    # Constants and parameters
    WHEEL_RADIUS = 0.00415  # meters
    DUTY_CYCLE = 0.5  # rad/s
    MAX_SPEED = 15.0

    DISTANCE_TO_MOVE = 0.5 # meters

    # Initialize LCM
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    

    # Move the robot forward for the specified distance at a constant velocity
    move_forward(lc, DUTY_CYCLE, WHEEL_RADIUS, MAX_SPEED, DISTANCE_TO_MOVE)
    
if __name__ == "__main__":
    print("Running")
    main()
