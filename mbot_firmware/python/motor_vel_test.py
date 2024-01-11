#!/usr/bin/env python3

import time
import lcm
import sys
import csv


sys.path.append('/usr/lib/python3.9/site-packages/')
from mbot_lcm_msgs.mbot_motor_vel_t import mbot_motor_vel_t



def my_handler(channel, data):
    msg = mbot_motor_vel_t.decode(data)
    # Write data to CSV
    print("11111111111111")
    writer.writerow([msg.utime, msg.velocity[0], msg.velocity[1]])


# Initialize LCM
lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

file = open('motor_velocities.csv', mode='a', newline='')

writer = csv.writer(file)
writer.writerow(['time', 'left_motor_vel', 'right_motor_vel'])
subscription = lc.subscribe("MBOT_MOTOR_VEL", my_handler)

def move_forward(lc, duty_cycle, wheel_radius, max_speed, dist):
    command = mbot_motor_vel_t()
    data = None  

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
        lc.handle()
        command.velocity[0] = -duty_cycle* max_speed
        command.velocity[1] = duty_cycle* max_speed*10/9.3
        lc.publish("MBOT_MOTOR_VEL_CMD", command.encode())
        time.sleep(0.005)
        
        # lc.subscribe("MBOT_MOTOR_VEL", data)
        # print(copy.velocity[0],copy.velocity[1])
        # time.sleep(0.005)

    command.velocity[0] = 0
    command.velocity[1] = 0
    lc.publish("MBOT_MOTOR_VEL_CMD", command.encode())

# Main function
def main():
    # Constants and parameters
    WHEEL_RADIUS = 0.00415  # meters
    DUTY_CYCLE = 0.3  # rad/s
    MAX_SPEED = 16.0

    DISTANCE_TO_MOVE = 0.5 # meters

    # Move the robot forward for the specified distance at a constant velocity
    move_forward(lc, DUTY_CYCLE, WHEEL_RADIUS, MAX_SPEED, DISTANCE_TO_MOVE)
    
if __name__ == "__main__":
    print("Running")
    main()
