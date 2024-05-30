# Created By  : Nadia khodaei   
 # Created Date: 10/03/2024 ..etc
 # version ='1.3'
import serial
import time
import numpy as np
import re
from filterpy.kalman import KalmanFilter

# Serial port settings
PORT_NAME = "/dev/ttyACM0"
BAUD_RATE = 115200
TIMEOUT = 1

# Commands
GET_GYRO = bytes([0x02])
SET_LED = bytes([0x01])

# Calibration parameters (replace with actual values)
GYRO_OFFSET = [0.0, 0.0, 0.0]
GYRO_SCALE = [1.0, 1.0, 1.0]
GYRO_BIAS = [0.0, 0.0, 0.0]

# Initialize Kalman Filter
kf = KalmanFilter(dim_x=3, dim_z=3)
kf.x = np.array([0, 0, 0])  # Initial state [roll, pitch, yaw]
kf.F = np.eye(3)  # State transition matrix (identity matrix)
kf.H = np.eye(3)  # Measurement matrix (identity matrix)
kf.P *= 1000  # Covariance matrix
kf.R = np.diag([0.1, 0.1, 0.1])  # Measurement noise covariance
kf.Q = np.diag([0.001, 0.001, 0.001])  # Process noise covariance

def open_serial_port(port_name, baud_rate, timeout):
    """Open serial port."""
    try:
        serial_port = serial.Serial(port_name, baud_rate, timeout=timeout)
        print("Serial connection opened")
        return serial_port
    except serial.SerialException as e:
        print(f"Failed to open serial connection: {e}")
        exit()

def send_command(serial_port, command):
    """Send command to STM32 and receive response."""
    serial_port.write(command)
    time.sleep(0.1)  # Wait for response
    return serial_port.readline().decode("utf-8").strip()

def calibrate_gyro(raw_values):
    """Apply calibration to raw gyro data."""
    calibrated_values = []
    for raw, offset, scale, bias in zip(raw_values, GYRO_OFFSET, GYRO_SCALE, GYRO_BIAS):
        calibrated = (raw - offset) * scale + bias
        calibrated_values.append(calibrated)
    return calibrated_values

def extract_sensor_data(response):
    """Extract sensor data from response."""
    data = [part.strip() for part in response.lstrip("\r\x00").rstrip(" n").split(",") if ":" in part]
    return [float(re.search(r'[-+]?\d*\.\d+|\d+', part.split(":")[1]).group()) for part in data]

# Main function
def main():
    # Open serial connection
    serial_port = open_serial_port(PORT_NAME, BAUD_RATE, TIMEOUT)

    try:
        while True:
            # Request gyro data
            gyro_response = send_command(serial_port, GET_GYRO)
            gyro_data = extract_sensor_data(gyro_response)

            if len(gyro_data) == 3:
                # Apply calibration
                calibrated_gyro = calibrate_gyro(gyro_data)

                # Perform Kalman filter prediction step
                kf.predict()

                # Perform Kalman filter correction step with measured gyro data
                kf.update(calibrated_gyro)

                # Get the filtered state estimate
                filtered_state = kf.x
                filtered_roll, filtered_pitch, filtered_yaw = filtered_state

                print(f"Filtered Roll: {filtered_roll}, Filtered Pitch: {filtered_pitch}, Filtered Yaw: {filtered_yaw}")

                # Other parts of your code...
    finally:
        # Close serial connection
        if serial_port is not None:
            serial_port.close()
            print("Serial connection closed")

if __name__ == "__main__":
    main()
