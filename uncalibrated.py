# Created By  : Nadia khodaei   
# Created Date: 10/03/2024
# version ='1.0'
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import re

# Serial port settings
port_name = "/dev/ttyACM0"
baud_rate = 115200
timeout = 1

# Initialize stm32 as None
stm32 = None

# Open serial connection
try:
    stm32 = serial.Serial(port_name, baud_rate, timeout=timeout)
    print("Serial connection opened")
except serial.SerialException as e:
    print(f"Failed to open serial connection: {e}")
    exit()

# Commands
GET_GYRO = bytes([0x02])
SET_LED = bytes([0x01])

# Function to send command and receive response
def send_command(command):
    stm32.write(command)
    time.sleep(0.1)  # Wait for response
    return stm32.readline().decode("utf-8").strip()

# Initialize variables for online noise measurement
num_measurements = 10  # Number of baseline measurements
baseline_measurements = []  # List to store baseline measurements
noise_measurements = []  # List to store noise measurements
time_points = []  # List to store time points (received data count)
prev_gyro_values = None  # Previous gyro values
gyro_drifts = []  # List to store gyro drifts over time

# Create a Matplotlib figure and axis for plotting
fig, ax = plt.subplots()
plt.ion()  # Turn on interactive mode for plotting
fig.show()

# Define a threshold for gyro drift (adjust as needed)
gyro_drift_threshold = 0.1  # Example threshold value

try:
    # Counter to keep track of received gyro data
    received_data_count = 0

    while True:
        # Request gyro data
        gyro_response = send_command(GET_GYRO)
        gyro_trip = gyro_response.lstrip("\r\x00").rstrip(" n")
        
        # Split response by comma and filter out invalid parts
        gyro_data = [part.strip() for part in gyro_trip.split(",") if ":" in part]

        if len(gyro_data) >= 3:
            # Extract roll, pitch, and yaw values
            roll = float(gyro_data[0].split(":")[1])
            pitch = float(gyro_data[1].split(":")[1])
            yaw_string = gyro_data[2].split(":")[1]  # Extract the value part of the string
            yaw_numeric = re.search(r'[-+]?\d*\.\d+|\d+', yaw_string).group()  # Extract numerical part using regex
            yaw = float(yaw_numeric)
            print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

            # Calculate angular velocity
            current_time = time.time()  # Get current time
            if prev_gyro_values is not None:
                # Calculate time difference
                delta_time = current_time - prev_time
                
                # Calculate angular displacement
                delta_angle = yaw - prev_angle
                
                # Calculate angular velocity
                angular_velocity = delta_angle / delta_time
                
                print(f"Angular Velocity: {angular_velocity} radians per second")
                
            # Update previous time and angle for next iteration
            prev_time = current_time
            prev_angle = yaw

            # Logic to determine LED states based on roll and pitch
            if abs(roll) > abs(pitch):
                # Roll has higher magnitude
                if roll > 1000:
                    # Roll is positive, set LED for positive roll
                    led_command = SET_LED + bytes([4]) 
                else:
                    # Roll is negative, set LED for negative roll
                    led_command = SET_LED + bytes([0])
            elif abs(pitch) > abs(roll):
                # Pitch has higher magnitude
                if pitch > 1000:
                    # Pitch is positive, set LED for positive pitch
                    led_command = SET_LED + bytes([2])
                else:
                    # Pitch is negative, set LED for negative pitch
                    led_command = SET_LED + bytes([6])
            else:
                # No significant roll or pitch, turn off LED
                led_command = SET_LED + bytes([0])

            # Send LED command
            response = send_command(led_command)
            
            # Increment the counter
            received_data_count += 1
            
            # Calculate noise level online
            baseline_measurements.append(np.linalg.norm([roll, pitch, yaw]))
            if len(baseline_measurements) > num_measurements:
                # Remove oldest measurement if exceeds the number of baseline measurements
                baseline_measurements.pop(0)
            if len(baseline_measurements) == num_measurements:
                # Calculate noise level (standard deviation) using current baseline measurements
                noise_level = np.std(baseline_measurements)

                # Append noise measurement and current time to lists
                noise_measurements.append(noise_level)
                time_points.append(received_data_count)

                # Update plot
                ax.clear()
                ax.plot(time_points, noise_measurements, marker='o')
                ax.set_xlabel('Time')
                ax.set_ylabel('Noise Level')
                ax.set_title('Online Noise Measurements Over Time')
                ax.grid(True)
                fig.canvas.draw()
                plt.pause(0.05)  # Pause to allow the plot to update
        
        if prev_gyro_values is not None:
            drift = np.linalg.norm(np.array([roll, pitch, yaw]) - prev_gyro_values)
            print(f"Gyro Drift: {drift}")
            # Compare gyro drift with threshold
            if drift > gyro_drift_threshold:
                print("Gyro drift exceeds threshold. Take corrective action.")
                # Implement corrective action here, such as recalibration or filtering

        prev_gyro_values = np.array([roll, pitch, yaw])

except KeyboardInterrupt:
    print("Exiting...")
finally:
    if stm32 is not None:
        stm32.close()  # Close serial connection

# Keep the plot window open
plt.ioff()
plt.show()
