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
GET_GYRO = bytes([0x03])
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
        print(gyro_data)
            


except KeyboardInterrupt:
    print("Exiting...")
finally:
    if stm32 is not None:
        stm32.close()  # Close serial connection


