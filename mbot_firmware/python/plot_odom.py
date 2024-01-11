import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
df1 = pd.read_csv('file_slam_pose.csv')

df2 = pd.read_csv('file_solution_pose.csv')

#time, x, y, theta

df1['time'] = df1['time'] - df1['time'][0]

total_time = df1['time'][len(df1['time'])-1]/1000000

avg_time = total_time/len(df1['time'])

print("Total time: ", total_time)
print("Average time: ", avg_time)


# Plotting
plt.figure(figsize=(10, 6))

plt.plot( df1['x'], df1['y'], label='SLAM')
plt.plot( df2['x'], df2['y'], label='Solution')

# Adding labels and title
plt.xlabel('X Position (meters)')
plt.ylabel('Y Position (meters)')
plt.title('X, Y Trajectory')
plt.legend()
plt.axis('equal')  # This will ensure the aspect ratio is equal and the plot is not skewed
plt.grid(True)

# Show the plot
plt.show()