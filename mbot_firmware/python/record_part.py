import lcm
from mbot_lcm_msgs.particle_t import particle_t
from mbot_lcm_msgs.particles_t import particles_t
from mbot_lcm_msgs.pose2D_t import pose2D_t
import sys
import csv

sys.path.append('/usr/lib/python3.9/site-packages/')

def main():
    file_part = open('file_particles.csv', mode='w', newline='')
    file_slam = open('file_slam.csv', mode='w', newline='')

    writer_part = csv.writer(file_part)
    writer_slam = csv.writer(file_slam)

    writer_part.writerow(['time', 'particles'])
    writer_slam.writerow(['time', 'x', 'y', 'theta'])
    

    def slam_pose_handler(channel, data):
        slam_pose_msg = pose2D_t.decode(data)
        # Write data to CSV
        writer_slam.writerow([float(slam_pose_msg.utime), slam_pose_msg.x, slam_pose_msg.y, slam_pose_msg.theta])

    def particle_handler(channel, data):
        slam_pose_msg = particles_t.decode(data)
        vector = []
        for i in range(len(slam_pose_msg.particles)):
            vector.append([slam_pose_msg.particles[i].pose.x, slam_pose_msg.particles[i].pose.y, slam_pose_msg.particles[i].pose.theta])
        
        # Write data to CSV
        writer_part.writerow([float(slam_pose_msg.utime), vector])

    lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
    lc.subscribe("SLAM_PARTICLES", particle_handler)
    lc.subscribe("SLAM_POSE", slam_pose_handler)

    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:   
        pass

if __name__ == "__main__":
    print("Running")
    main()
