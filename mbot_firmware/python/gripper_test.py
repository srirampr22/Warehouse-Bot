#!/usr/bin/env python3

import time
import lcm
import sys

sys.path.append('/usr/lib/python3.9/site-packages/')
from mbot_lcm_msgs.mbot_servo_pwm_t import mbot_servo_pwm_t


def move_gripper(lc, duty_cycle, duration):
    command = mbot_servo_pwm_t()

    # duration = dist / (duty_cycle* max_speed * 2 * 3.1415 * wheel_radius)
    print("Duration: ", duration)
    
    start_time = time.perf_counter()
    end_time = start_time + duration
    
    print("START TIME:", start_time)
    print("END TIME:", end_time)
    print("DUTY CYCLE:", duty_cycle)
    

    while time.perf_counter() <= end_time:
          # Introduce a delay to avoid flooding the channel with too many messages
        command.pwm = (time.perf_counter()- start_time)/20
        # command.pwm[1] = -0.3*10/9.3
        lc.publish("MBOT_SERVO_PWM_CMD", command.encode())
        time.sleep(0.005)


    command.pwm = 0
    # command.pwm[1] = 0
    lc.publish("MBOT_SERVO_PWM_CMD", command.encode())

# Main function
def main():
    # Constants and parameters
    DUTY_CYCLE = 0.5  # rad/s


    # Initialize LCM
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

    # Move the robot forward for the specified distance at a constant velocity
    move_gripper(lc, DUTY_CYCLE, duration = 20.0)
    
if __name__ == "__main__":
    print("Running")
    main()
