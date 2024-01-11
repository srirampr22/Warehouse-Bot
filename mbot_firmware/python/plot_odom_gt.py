import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file for trajectory data
df = pd.read_csv('/home/mbot/mbot_ws/mbot_firmware/odom_msg_0.2_1.csv')

# Define the ground truth path based on the image provided
# This sequence of (x, y) points represents the corners of the path
ground_truth_path = [
    (0, 0), (0.61, 0), (0.61, -0.61), (1.22, -0.61), (1.22, 0.61),
    (1.83, 0.61), (1.83, -0.61), (2.44, -0.61), (2.44, 0), (3.05, 0)
]

# Unzip the points into X and Y coordinates
gt_x, gt_y = zip(*ground_truth_path)

# Plotting the trajectory
plt.figure(figsize=(10, 6))
plt.plot(df['x'], df['y'], label='Trajectory')

# Plotting the ground truth path
plt.plot(gt_x, gt_y, 'r--', label='Ground Truth Path') # 'r--' makes the line red and dashed

# Adding labels and title
plt.xlabel('X Position (meters)')
plt.ylabel('Y Position (meters)')
plt.title('X, Y Trajectory with Ground Truth Path')
plt.legend()
plt.axis('equal')  # This will ensure the aspect ratio is equal and the plot is not skewed
plt.grid(True)

# Show the plot
plt.show()
