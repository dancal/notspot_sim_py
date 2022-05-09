#!/usr/bin/env python3
from MPU6050 import MPU6050

import time
import rospy
#from notspot_mpu6050.msg import mpu6050msg
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField 

if __name__ == "__main__":
    rospy.init_node('notspot_mpu6050', anonymous=True)
    rate = rospy.Rate(100) # 5hz

    pubs_imu    = rospy.Publisher('notspot_imu/base_link_orientation', Imu, queue_size = 2)
    #pubs_magnetometer = rospy.Publisher('imu/mag', MagneticField, queue_size = 15)
    imu_read    = Imu()
	#magnetometer_read = MagneticField()

    #rospy.init_node("notspot_mpu6050")
    #pub = rospy.Publisher("notspot_imu/base_link_orientation",mpu6050msg, queue_size=10)

    #mpu6050pub = mpu6050msg()
    i2c_bus         = 1
    device_address  = 0x68

    x_accel_offset  = int(-841)
    y_accel_offset  = int(1794)
    z_accel_offset  = int(3206)
    x_gyro_offset   = int(26)
    y_gyro_offset   = int(28)
    z_gyro_offset   = int(26)

    # x_avg_read: 0.08 x_avg_offset: 926.423499999996
    # y_avg_read: -0.72 y_avg_offset: 2136.152999999997
    # z_avg_read: 0.34 z_avg_offset: -856.0713749999996
    # x_avg_read: 0.16 x_avg_offset: 35.570499999999996
    # y_avg_read: -0.07 y_avg_offset: -8.081437499999883
    # z_avg_read: -0.01 z_avg_offset: -28.00306249999999

    enable_debug_output = True
    mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,enable_debug_output)

    mpu.dmp_initialize()
    mpu.set_DMP_enabled(True)
    mpu_int_status = mpu.get_int_status()
    print(hex(mpu_int_status))

    packet_size = mpu.DMP_get_FIFO_packet_size()
    print(packet_size)
    FIFO_count = mpu.get_FIFO_count()
    print(FIFO_count)

    FIFO_buffer = [0]*64
    rate = rospy.Rate(100)
    try:
        while not rospy.is_shutdown():
            try:
                FIFO_count = mpu.get_FIFO_count()
                mpu_int_status = mpu.get_int_status()
            except:
                continue
            # If overflow is detected by status or fifo count we want to reset
            if (FIFO_count == 1024) or (mpu_int_status & 0x10):
                mpu.reset_FIFO()
                print('overflow!')
            # Check if fifo data is ready
            elif (mpu_int_status & 0x02):
                # Wait until packet_size number of bytes are ready for reading, default
                # is 42 bytes
                while FIFO_count < packet_size:
                    FIFO_count = mpu.get_FIFO_count()

                FIFO_buffer     = mpu.get_FIFO_bytes(packet_size)
                accel           = mpu.DMP_get_acceleration_int16(FIFO_buffer)
                quat            = mpu.DMP_get_quaternion_int16(FIFO_buffer)
                grav            = mpu.DMP_get_gravity(quat)
                roll_pitch_yaw  = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
                acc             = mpu.get_acceleration()
                gyro            = mpu.get_rotation()

                imu_read.header.stamp = rospy.Time.now()
                imu_read.header.frame_id = "base_link"

                ### Accelerometer data
                imu_read.linear_acceleration.x = acc[0]/16384.0
                imu_read.linear_acceleration.y = acc[1]/16384.0
                imu_read.linear_acceleration.z = acc[2]/16384.0

                #### Gyrometer data
                gyro_x = gyro[0]/16.384
                gyro_y = gyro[1]/16.384
                gyro_z = gyro[2]/16.384

                if abs(gyro_x) < 0.2:
                    imu_read.angular_velocity.x = 0.0
                else:
                    imu_read.angular_velocity.x = gyro_x

                if abs(gyro_y) < 0.2:
                    imu_read.angular_velocity.y = 0.0
                else:
                    imu_read.angular_velocity.y = gyro_y

                if abs(gyro_z) < 0.2:
                    imu_read.angular_velocity.z = 0.0
                else:
                    imu_read.angular_velocity.z = gyro_z

                #imu_read.orientation.x          = roll_pitch_yaw.x
                #imu_read.orientation.y          = roll_pitch_yaw.y
                #imu_read.orientation.z          = roll_pitch_yaw.z
                #imu_read.orientation.w          = 0.1

                imu_read.orientation.x          = quat.x
                imu_read.orientation.y          = quat.y
                imu_read.orientation.z          = quat.z
                imu_read.orientation.w          = quat.w
                print(imu_read.orientation)

                # str_show = "roll: %.2f  pitch: %.2f  yaw: %.2f        "%(roll_pitch_yaw.x,roll_pitch_yaw.y,roll_pitch_yaw.z)
                # print("\r %s"%(str_show),end='')
                #mpu6050pub.Euler_angles[0] = roll_pitch_yaw.x
                #mpu6050pub.Euler_angles[1] = roll_pitch_yaw.y
                #mpu6050pub.Euler_angles[2] = roll_pitch_yaw.z
                #mpu6050pub.accel[0] = acc[0]/16384.0
                #mpu6050pub.accel[1] = acc[1]/16384.0
                #mpu6050pub.accel[2] = acc[2]/16384.0
                #mpu6050pub.gyro[0] = gyro[0]/16.384
                #mpu6050pub.gyro[1] = gyro[1]/16.384
                #mpu6050pub.gyro[2] = gyro[2]/16.384

                #print("\r %s"%(str_show))
                #print(acc[0]/16384.0)
                #print(acc[1]/16384.0)
                #print(acc[2]/16384.0)
                #print(gyro[0]/16.384)
                #print(gyro[1]/16.384)
                #print(gyro[2]/16.384)
                pubs_imu.publish(imu_read)

                rate.sleep()
                
                rospy.loginfo(roll_pitch_yaw.x)
    except KeyboardInterrupt:
        print('\n Ctrl + C QUIT')
           

    