import lcm
# from mbot_lcm_msgs.mbot_motor_vel_t import mbot_motor_vel_t
# from mbot_lcm_msgs.twist2D_t import twist2D_t
from mbot_lcm_msgs.pose2D_t import pose2D_t
import sys
import csv

sys.path.append('/usr/lib/python3.9/site-packages/')

def main():
    # Open a CSV file to write the data
    # Open a CSV file to write the data
    # file_cmd_vel = open('file_cmd_vel_right.csv', mode='w', newline='')
    # file_enc_vel = open('file_enc_vel_right.csv', mode='w', newline='')

    file_slam = open('file_slam_pose.csv', mode='w', newline='')
    file_sol = open('file_solution_pose.csv', mode='w', newline='')

    # writer_cmd_vel = csv.writer(file_cmd_vel)
    # writer_enc_vel = csv.writer(file_enc_vel)

    writer_slam = csv.writer(file_slam)
    writer_sol = csv.writer(file_sol)

    # writer_enc_vel.writerow(['time', 'left_motor_vel', 'right_motor_vel'])
    # # writer_enc_vel.writerow(['time', 'left_motor_vel', 'right_motor_vel'])

    # writer_cmd_vel.writerow(['time', 'vx', 'vy', 'wz'])

    writer_slam.writerow(['time', 'x', 'y', 'theta'])
    writer_sol.writerow(['time', 'x', 'y', 'theta'])

    # def enc_vel_handler(channel, data):
    #     enc_vel_msg = mbot_motor_vel_t.decode(data)
    #     # Write data to CSV
    #     writer_enc_vel.writerow([float(enc_vel_msg.utime), enc_vel_msg.velocity[0], enc_vel_msg.velocity[1]])

    # def cmd_vel_handler(channel, data):
    #     cmd_vel_msg = twist2D_t.decode(data)
    #     # Write data to CSV
    #     writer_cmd_vel.writerow([float(cmd_vel_msg.utime), cmd_vel_msg.vx, cmd_vel_msg.vy, cmd_vel_msg.wz])

    def slam_pose_handler(channel, data):
        slam_pose_msg = pose2D_t.decode(data)
        # Write data to CSV
        writer_slam.writerow([float(slam_pose_msg.utime), slam_pose_msg.x, slam_pose_msg.y, slam_pose_msg.theta])

    def sol_pose_handler(channel, data):
        sol_pose_msg = pose2D_t.decode(data)
        # Write data to CSV
        writer_sol.writerow([float(sol_pose_msg.utime), sol_pose_msg.x, sol_pose_msg.y, sol_pose_msg.theta])

    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    # lc.subscribe("MBOT_MOTOR_VEL", enc_vel_handler)
    # lc.subscribe("MBOT_VEL_CMD", cmd_vel_handler)
    lc.subscribe("SLAM_POSE", slam_pose_handler)
    lc.subscribe("SOLUTION_POSE", sol_pose_handler)

    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:   
        pass

if __name__ == "__main__":
    print("Running")
    main()
