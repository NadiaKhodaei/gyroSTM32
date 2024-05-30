#!/usr/bin/env python
import serial
import time
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros

def request_imu():
    try:
        stm32.write(bytearray([0x03]))  # Command to request IMU data
        time.sleep(0.5)
        return stm32.readline().decode("utf-8")
    except serial.SerialException as e:
        rospy.logerr("Error reading from serial port: %s", e)
        return None

def imu_publisher():
    port_name = "/dev/ttyACM0"
    global stm32
    try:
        stm32 = serial.Serial(port_name, 9600, timeout=1)
    except serial.SerialException as e:
        rospy.logerr("Error opening serial port: %s", e)
        return  # Exit if cannot open serial

    pub = rospy.Publisher('imu_data', Imu, queue_size=10)
    rospy.init_node('imu_reader_node', anonymous=True)
    rate = rospy.Rate(2)  # 2Hz
    br = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():
        raw_data = request_imu()
        if raw_data:
            data = raw_data.strip('\x00').strip()
            data_list = data.split(',')

            try:
                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = "imu_link"

                # Assuming the first three values are accelerometer readings and the next three are gyroscope readings
                imu_msg.linear_acceleration.x = float(data_list[0])
                imu_msg.linear_acceleration.y = float(data_list[1])
                imu_msg.linear_acceleration.z = float(data_list[2])
                imu_msg.angular_velocity.x = float(data_list[3])
                imu_msg.angular_velocity.y = float(data_list[4])
                imu_msg.angular_velocity.z = float(data_list[5])

                # Broadcast transform
                t = TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "base_link"
                t.child_frame_id = "imu_link"
                t.transform.translation.x = 0.0  # Change based on actual mounting
                t.transform.translation.y = 0.0
                t.transform.translation.z = 0.0
                t.transform.rotation.x = 0.0
                t.transform.rotation.y = 0.0
                t.transform.rotation.z = 0.0
                t.transform.rotation.w = 1.0  # No rotation relative to base_link

                br.sendTransform(t)
                pub.publish(imu_msg)
            except ValueError as e:
                rospy.logerr("Error converting data to float: %s", e)
        else:
            rospy.logwarn("Failed to read data from STM32.")
        rate.sleep()

if __name__ == '__main__':
    imu_publisher()

