#! /usr/bin/env python3

from MPU6050 import MPU6050
import time
import rospy
import tf
import numpy as np
from sensor_msgs.msg import Imu
#from tf.transformations import *


class Kalman():


    def __init__(self):
        self.P = np.matrix([[0., 0.],[0., 0.]])
    def setKalmanAngle(self, angle):
        self.State = np.matrix([[angle],[0.   ]])
        #print('0. set initial guess: \n', self.State)

    def getKalmanAngle(self, angle, gyro_rate, dt):
        R = 0.03
        Q = np.matrix([[0.001, 0.   ],[0.,    0.003]])
        H = np.matrix( [1.,    0.   ])

        F = np.matrix([[1., -dt],[0., 1. ]])
        B = np.matrix([[dt],[0.]])
        #print('F= \n', F)
        #print('B= \n', B)
        #print(self.State)
        
        #(I). State prediction
        self.State = F * self.State + B * gyro_rate
        #print('I. State prediction: \n', self.State)

        #(II). Covariance prediction
        self.P = F * self.P * np.transpose(F) + Q
        #print('II. Covariance prediction P: \n', self.P)

        #(III). Innovation
        I = angle - H * self.State
        #print('III. Innovation I: \n', I)

        #(IV). Innovation covariance S
        S = H * self.P * np.transpose(H) + R
        #print('IV. Innovation covariance S: \n', S)

        #(V). Kalman gain KG
        KG = self.P * np.transpose(H) / S
        #print('V. Kalman Gain: \n', KG)

        #(VI). Update state
        self.State = self.State + KG * I
        #print('VI. Update State: \n', self.State)

        #(VII). Update covariance
        self.P = (np.eye(2) - KG * H) * self.P
        #print('VII. Update Covariance P: \n', self.P)

        return self.State.item(0)

#X = Kalman()
#X.setKalmanAngle(30)
#x1 = X.getKalmanAngle(30, 2, 0.01)
#print(x1)

if __name__ == "__main__":
    rospy.init_node("notspot_mpu6050")
    pub         = rospy.Publisher("notspot_imu/base_link_orientation", Imu,queue_size=10)

    i2c_bus = 1
    device_address = 0x68

    x_accel_offset = int(-841)
    y_accel_offset = int(1794)
    z_accel_offset = int(3206)
    x_gyro_offset = int(26)
    y_gyro_offset = int(28)
    z_gyro_offset = int(26)

    # x_avg_read: 0.08 x_avg_offset: 926.423499999996
    # y_avg_read: -0.72 y_avg_offset: 2136.152999999997
    # z_avg_read: 0.34 z_avg_offset: -856.0713749999996
    # x_avg_read: 0.16 x_avg_offset: 35.570499999999996
    # y_avg_read: -0.07 y_avg_offset: -8.081437499999883
    # z_avg_read: -0.01 z_avg_offset: -28.00306249999999

    enable_debug_output = False
    mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,enable_debug_output)

    mpu.dmp_initialize()
    mpu.set_DMP_enabled(True)
    mpu_int_status = mpu.get_int_status()
    print(hex(mpu_int_status))

    packet_size = mpu.DMP_get_FIFO_packet_size()
    print(packet_size)
    FIFO_count = mpu.get_FIFO_count()
    print(FIFO_count)

    # mpu.calibrateMPU6500()
    # print(self.gbias)
    # print(self.abias)
    Roll = Kalman()
    Pitch = Kalman()
    Yaw = Kalman()

    Roll.setKalmanAngle(0)
    Pitch.setKalmanAngle(0)
    Yaw.setKalmanAngle(0.)

    FIFO_buffer = [0]*64
    rate = rospy.Rate(10)
    #make initial guesses
    #time_pre = time.time()

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

                # DMP_get_quaternion

                #  attitude = ahrs.filters.Madgwick(acc=acc_data, gyr=gyro_data)
                accVal          = [acc[0]/16384.0, acc[1]/16384.0, acc[2]/16384.0]
                gryoVal         = [gyro[0]/16.384, gyro[1]/16.384, gyro[2]/16.384]
                #str_show = "roll: %.2f  pitch: %.2f  yaw: %.2f        "%(roll_pitch_yaw.x,roll_pitch_yaw.y,roll_pitch_yaw.z)
                #print("\r %s"%(str_show),end='')

                quaternion      = tf.transformations.quaternion_from_euler(roll_pitch_yaw.x, roll_pitch_yaw.y, roll_pitch_yaw.z)

	            # R = Roll.getKalmanAngle(get_roll(ax, ay, az), gx, dt)
	            # #print('Roll: \t', R)
	            # P = Pitch.getKalmanAngle(get_pitch(ax, ay, az), gy, dt)

                # covariance matrix
                imu_data    = Imu()
                imu_data.header.stamp = rospy.Time.now()

                imu_data.orientation_covariance[0] = -1
                imu_data.angular_velocity_covariance[0] = -1
                imu_data.linear_acceleration_covariance[0] = -1

                imu_data.angular_velocity.x     = gryoVal[0]
                imu_data.angular_velocity.y     = gryoVal[1]
                imu_data.angular_velocity.z     = gryoVal[2]

                imu_data.linear_acceleration.x  = accVal[0]
                imu_data.linear_acceleration.y  = accVal[1]
                imu_data.linear_acceleration.z  = accVal[2]

                imu_data.orientation.x          = quaternion[0]
                imu_data.orientation.y          = quaternion[1]
                imu_data.orientation.z          = quaternion[2]
                imu_data.orientation.w          = quaternion[3]

                pub.publish(imu_data)

                #time_pre = time.time()
                rate.sleep()
                #rospy.loginfo(roll_pitch_yaw.x)

    except KeyboardInterrupt:
        print('\n Ctrl + C QUIT')