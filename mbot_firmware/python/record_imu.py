import lcm
from mbot_lcm_msgs.mbot_motor_vel_t import mbot_motor_vel_t
from mbot_lcm_msgs.mbot_imu_t import mbot_imu_t
from mbot_lcm_msgs.pose2D_t import pose2D_t
import sys
import csv

sys.path.append('/usr/lib/python3.9/site-packages/')

def main():
    # Open a CSV file to write the data
    with open('imu_msg.csv', mode='w', newline='') as file:
        writer = csv.writer(file)
        # Writing the header
        writer.writerow(['time', 'gyro_z'])

        def my_handler(channel, data):
            # print("1")
            msg = mbot_imu_t.decode(data)
            # Write data to CSV
            # print("2")
            writer.writerow([float(msg.utime), msg.gyro[2]])

        lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        subscription = lc.subscribe("MBOT_IMU", my_handler)

        try:
            while True:
                lc.handle()
        except KeyboardInterrupt:
            pass

if __name__ == "__main__":
    print("Running")
    main()