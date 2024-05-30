import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import re

# Serial port settings
PORT_NAME = "/dev/ttyACM0"
BAUD_RATE = 115200
TIMEOUT = 1

# Commands
GET_GYRO = bytes([0x02])
SET_LED = bytes([0x01])

# Function to open serial connection
def open_serial_connection(port_name, baud_rate, timeout):
    try:
        stm32 = serial.Serial(port_name, baud_rate, timeout=timeout)
        print("Serial connection opened")
        return stm32
    except serial.SerialException as e:
        print(f"Failed to open serial connection: {e}")
        return None

# Function to close serial connection
def close_serial_connection(ser):
    if ser is not None:
        ser.close()
        print("Serial connection closed")

# Function to send command and receive response
def send_command(ser, command):
    ser.write(command)
    time.sleep(0.01)  # Wait for response
    return ser.readline().decode("utf-8").strip()

# Gyro calibration function
def calibrate_gyro(ser):
    print("Calibrating gyro...")
    baseline_roll = []
    baseline_pitch = []
    baseline_yaw = []
    num_calibration_samples = 10 # Number of samples for calibration

    for _ in range(num_calibration_samples):
        gyro_response = send_command(ser, GET_GYRO)
        gyro_trip = gyro_response.lstrip("\r\x00").rstrip(" n")

        gyro_data = [part.strip() for part in gyro_trip.split(",") if ":" in part]

        if len(gyro_data) >= 3:
            baseline_roll.append(float(gyro_data[0].split(":")[1]))
            baseline_pitch.append(float(gyro_data[1].split(":")[1]))
            yaw_string = gyro_data[2].split(":")[1]
            yaw_numeric = re.search(r'[-+]?\d*\.\d+|\d+', yaw_string).group()
            baseline_yaw.append(float(yaw_numeric))
            time.sleep(0.2)  # Wait between samples

    avg_roll = np.mean(baseline_roll)
    avg_pitch = np.mean(baseline_pitch)
    avg_yaw = np.mean(baseline_yaw)

    print(f"Baseline Roll: {avg_roll}, Baseline Pitch: {avg_pitch}, Baseline Yaw: {avg_yaw}")
    return avg_roll, avg_pitch, avg_yaw

# Function to apply moving average filter
def moving_average(data, window_size):
    if len(data) < window_size:
        return None
    return np.mean(data[-window_size:])

# Main function
def main():
    # Open serial connection
    stm32 = open_serial_connection(PORT_NAME, BAUD_RATE, TIMEOUT)
    if stm32 is None:
        print ("no connection")
        return

    try:
        # Calibration
        baseline_roll_bias, baseline_pitch_bias, baseline_yaw_bias = calibrate_gyro(stm32)

        # Initialize variables
        window_size = 50  # Size of the moving average window
        roll_buffer = []
        pitch_buffer = []
        yaw_buffer = []

        received_data_count = 0

        # Initialize plot for data visualization
        plt.ion()
        fig, ax = plt.subplots()
        time_points = []

        while True:
            gyro_response = send_command(stm32, GET_GYRO)
            gyro_trip = gyro_response.lstrip("\r\x00").rstrip(" n")

            gyro_data = [part.strip() for part in gyro_trip.split(",") if ":" in part]

            if len(gyro_data) >= 3:
                roll_raw = float(gyro_data[0].split(":")[1]) - baseline_roll_bias
                pitch_raw = float(gyro_data[1].split(":")[1]) - baseline_pitch_bias
                yaw_raw = float(re.search(r'[-+]?\d*\.\d+|\d+', gyro_data[2].split(":")[1]).group()) - baseline_yaw_bias

                # Apply moving average filter
                roll_filtered = moving_average(roll_buffer, window_size)
                pitch_filtered = moving_average(pitch_buffer, window_size)
                yaw_filtered = moving_average(yaw_buffer, window_size)

                # Update buffers
                roll_buffer.append(roll_raw)
                pitch_buffer.append(pitch_raw)
                yaw_buffer.append(yaw_raw)

                # Print roll, pitch, and yaw values
                # print(f"Roll: {roll_filtered}, Pitch: {pitch_filtered}, Yaw: {yaw_filtered}")

                # Data visualization
                # time_points.append(received_data_count)
                # ax.clear()
                # ax.plot(time_points, roll_buffer, label='Roll')
                # ax.plot(time_points, pitch_buffer, label='Pitch')
                # ax.plot(time_points, yaw_buffer, label='Yaw')
                # ax.legend()
                # plt.xlabel('Time')
                # plt.ylabel('Angle')
                # plt.title('Gyro Data Visualization')
                # plt.pause(0.01)

                # Logic to determine LED states based on roll and pitch
                if roll_filtered is not None and pitch_filtered is not None and yaw_filtered is not None:
                    if abs(roll_filtered) > abs(pitch_filtered):
                        # Roll has higher magnitude
                        if roll_filtered > 1000:
                            # Roll is positive, set LED for positive roll
                            led_command = SET_LED + bytes([4]) 
                        else:
                            # Roll is negative, set LED for negative roll
                            led_command = SET_LED + bytes([0])
                    elif abs(pitch_filtered) > abs(roll_filtered):
                        # Pitch has higher magnitude
                        if pitch_filtered > 1000:
                            # Pitch is positive, set LED for positive pitch
                            led_command = SET_LED + bytes([2])
                        else:
                            # Pitch is negative, set LED for negative pitch
                            led_command = SET_LED + bytes([6])
                else:
                    # Handle case where filtered values are None
                    print("Filtered values are None, skipping LED control")
                    continue


                # Send LED command
                send_command(stm32, led_command)

                # Increment the counter
                received_data_count += 1

    except KeyboardInterrupt:
        print("Exiting...")
   
        # close_serial_connection(stm32)

if __name__ == "__main__":
    main()
