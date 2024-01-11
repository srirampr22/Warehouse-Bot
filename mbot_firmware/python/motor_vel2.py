import lcm
import time
import csv
import sys
import threading
from mbot_lcm_msgs.mbot_motor_vel_t import mbot_motor_vel_t

sys.path.append('/usr/lib/python3.9/site-packages/')

# Funstion that handles subsrcibng to lcm message and write to csv file
def my_handler(channel, data, writer):
    msg = mbot_motor_vel_t.decode(data)
    # Write data to CSV
    writer.writerow([float(msg.utime), msg.velocity[0], msg.velocity[1]])

def lcm_thread_function(lc, writer):
    subscription = lc.subscribe("MBOT_MOTOR_VEL", lambda channel, data: my_handler(channel, data, writer))
    while True:
        lc.handle()

def move_forward(lc, const_velocity_radps, wheel_radius, dist):
    command = mbot_motor_vel_t()
    duration = dist / (const_velocity_radps * 2 * 3.1415 * wheel_radius)
    
    end_time = time.perf_counter() + duration

    while time.perf_counter() <= end_time:
        command.velocity[0] = -const_velocity_radps
        command.velocity[1] = const_velocity_radps
        lc.publish("MBOT_MOTOR_VEL_CMD", command.encode())
        time.sleep(0.0001)  

    # Stop the robot
    command.velocity[0] = 0
    command.velocity[1] = 0
    lc.publish("MBOT_MOTOR_VEL_CMD", command.encode())

def main():
    # Constants and parameters
    WHEEL_RADIUS = 0.00415  # meters
    CONST_VELOCITY_RADPS = 7  # rad/s
    DISTANCE_TO_MOVE = 0.5 # meters

    # Initialize LCM
    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

    # Open a CSV file to write the data
    with open('motor_velocities.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['time', 'left_motor_vel', 'right_motor_vel'])

        # Start LCM listening in a separate thread
        lcm_thread = threading.Thread(target=lcm_thread_function, args=(lc, writer))
        lcm_thread.start()

        # Move the robot forward
        move_forward(lc, CONST_VELOCITY_RADPS, WHEEL_RADIUS, DISTANCE_TO_MOVE)

        # Wait for the LCM thread to finish
        lcm_thread.join(timeout=5)  # Adjust the timeout as needed

if __name__ == "__main__":
    main()
