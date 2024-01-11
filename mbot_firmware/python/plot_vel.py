import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
df1 = pd.read_csv('file_enc_vel_right.csv')

df2 = pd.read_csv('file_cmd_vel_right.csv')

#time,vx,vy,wz
left_motor_vel_cmd = (df2['vx'] - (0.0845 * df2['wz'])) / 0.04183
right_motor_vel_cmd = ((-1 * df2['vx']) - (0.0845 * df2['wz'])) / 0.04183

print(len(left_motor_vel_cmd))

# Plotting
fig, axs = plt.subplots(2, 1)


set_point_right = 7
set_point_left = -7

# Plotting left and right motor velocities
axs[0].plot(df1['time'], df1['left_motor_vel'], label='Left Motor Velocity')
axs[0].plot(df2['time'], left_motor_vel_cmd, label='Setpoint Left Motor Velocity')

# axs[0].xlabel('Time')
# axs[0].ylabel('Motor Velocity')
# axs[0].title('Motor Velocities Over Time')
# axs[0].legend()

axs[1].plot(df1['time'], df1['right_motor_vel'], label='Right Motor Velocity')
axs[1].plot(df2['time'], right_motor_vel_cmd, label='Setpoint Right Motor Velocity')

# axs[1].xlabel('Time')
# axs[1].ylabel('Motor Velocity')
# axs[1].title('Motor Velocities Over Time')
# axs[1].legend()

# # Plotting set points
# plt.axhline(y=left_motor_vel_cmd, color='r', linestyle='--', label='Set Point')
# plt.axhline(y=right_motor_vel_cmd, color='r', linestyle='--', label='Set Point')
# Adding labels and title


# Show the plot
plt.show()