import pandas as pd

def calculate_gyro_bias(file_path):
    # Load the CSV data into a DataFrame
    df = pd.read_csv(file_path)

    # Calculate the mean of the gyro_z column
    gyro_bias = df['gyro_z'].mean()

    return gyro_bias

file_path = '~/mbot_ws/mbot_firmware/python/imu_msg.csv'
bias = calculate_gyro_bias(file_path)
print("Gyro_z Bias:", bias)
