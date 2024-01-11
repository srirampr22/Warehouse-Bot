import lcm
from mbot_lcm_msgs.mbot_motor_vel_t import mbot_motor_vel_t
from mbot_lcm_msgs.twist2D_t import twist2D_t
from mbot_lcm_msgs.pose2D_t import pose2D_t
import sys
import csv

sys.path.append('/usr/lib/python3.9/site-packages/')

def main():
    # Open a CSV file to write the data
    file_vel = open('vel_msg.csv', mode='w', newline='')
    file_odom = open('odom_msg.csv', mode='w', newline='')

    writer1 = csv.writer(file_odom)
    writer2 = csv.writer(file_vel)
    # Writing the header
    writer1.writerow(['time', 'x', 'y', 'theta'])
    writer2.writerow(['time', 'vx', 'vy', 'wz'])

    

    def odom_handler(channel, data):
        odom_msg = pose2D_t.decode(data)
        # Write data to CSV
        writer1.writerow([float(odom_msg.utime), odom_msg.x, odom_msg.y, odom_msg.theta ])
    
    def vel_handler(channel, data):
        vel_msg = twist2D_t.decode(data)
        # Write data to CSV
        writer2.writerow([float(vel_msg.utime), vel_msg.vx, vel_msg.vy, vel_msg.wz ])

    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    lc.subscribe("MBOT_ODOMETRY", odom_handler)
    lc.subscribe("MBOT_VEL", vel_handler)


    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    print("Running")
    main()