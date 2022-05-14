#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

import os
import sys
import time
import smbus
import numpy as np

import MPU9250
import kalman 
#import madgwick
from registers import *

import tf.transformations 
class IMU_MPU9250:
	def __init__(self):
		
		self.address = 0x68
		#self.bus = smbus.SMBus(1)
		#self.imu = MPU9250.MPU9250(self.bus, self.address)
		#self.imu = MPU9250(address_ak, address_mpu_master, address_mpu_slave, bus, gfs, afs, mfs, mode)
		#self.imu = MPU9250.MPU9250(AK8963_ADDRESS, MPU9050_ADDRESS_68, MPU9050_ADDRESS_68, 1, GFS_1000, AFS_8G, AK8963_BIT_16, AK8963_MODE_C100HZ)
		self.imu = MPU9250.MPU9250(AK8963_ADDRESS, MPU9050_ADDRESS_68, MPU9050_ADDRESS_68, 1, GFS_250, AFS_2G, AK8963_BIT_16, AK8963_MODE_C100HZ)
		self.imu_publisher = rospy.Publisher("notspot_imu/base_link_orientation", Imu, queue_size=10)
		self.imu_data = Imu()
			
		rospy.init_node("imu_node")

	def run(self):
		#self.imu.begin()

		#self.imu.abias	= [-0.00594482421875, -0.021722412109375, 0.012518310546874911]
		#self.imu.gbias	= [2.5920867919921875, -1.71966552734375, -0.49095153808593744]
		#self.imu.abias	= [-0.037506103515625, -0.009466552734375, 0.006726074218750044]
		#self.imu.gbias	= [2.474212646484375, -1.726722717285156, -0.458526611328125]
		#[0.040191650390625, 0.039825439453125, -0.015142822265625022]
		#[2.9462814331054688, -1.7131805419921875, -1.4919281005859375]

		#[0.04139404296875, 0.04173583984375, -0.014245605468749978]
		#[2.9500961303710938, -1.7169952392578125, -1.5001296997070312]


		#self.imu.reset()
		#self.imu.calibrate() # Calibrate sensors
		self.imu.calibrateMPU6500()
		self.imu.configure()

		self.sensorfusionKalman 	= kalman.Kalman()

		print(self.imu.abias)
		print(self.imu.gbias)
		print(self.imu.magScale)
		print(self.imu.mbias)
		
		count = 0
		currTime = time.time()
		rate = rospy.Rate(60)
		while not rospy.is_shutdown():

			newTime 	= time.time()
			dt 			= newTime - currTime
			currTime 	= newTime

			N 			= 5
			accX		= 0
			accY		= 0
			accZ		= 0
			gyroX		= 0
			gyroY		= 0
			gyroZ		= 0
			magX		= 0
			magY		= 0
			magZ		= 0

			accArrX		= []
			accArrY		= []
			accArrZ		= []
			gyroArrX	= []
			gyroArrY	= []
			gyroArrZ	= []
			for i in range(N):
				AccelVals	= self.imu.readAccelerometerMaster()
				GyroVals	= self.imu.readGyroscopeMaster()
				MagVals 	= self.imu.readMagnetometerMaster()

				accArrX.append(AccelVals[0])
				accArrY.append(AccelVals[1])
				accArrZ.append(AccelVals[2])

				gyroArrX.append(GyroVals[0])
				gyroArrY.append(GyroVals[1])
				gyroArrZ.append(GyroVals[2])

				accX		= AccelVals[0]
				accY		= AccelVals[1]
				accZ		= AccelVals[2]

				gyroX		= GyroVals[0]
				gyroY		= GyroVals[1]
				gyroZ		= GyroVals[2]

				magX		= MagVals[0]
				magY		= MagVals[1]
				magZ		= MagVals[2]
				time.sleep(0.01)

			magX	= 1
			magY	= 1
			magZ	= 1

			accX	= np.mean(accArrX) / 16380.0
			accY	= np.mean(accArrY) / 16380.0
			accZ	= np.mean(accArrZ) / 16380.0

			gyroX	= np.mean(gyroArrX) / 16.0
			gyroY	= np.mean(gyroArrY) / 16.0
			gyroZ	= np.mean(gyroArrZ) / 16.0

			#print( 'min = ', np.min(accArrX), ', max = ', np.max(accArrX) )
			print(accX, accY, accZ)
			self.sensorfusionKalman.computeAndUpdateRollPitchYaw(accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ, dt)
			#self.sensorfusionMadgwick.updateRollPitchYaw(accX, accY, accZ, gyroX, gyroY, gyroZ, magX, magY, magZ, dt)

			## print ("Accel x: {0} ; Accel y : {1} ; Accel z : {2}".format(self.imu.AccelVals[0], self.imu.AccelVals[1], self.imu.AccelVals[2]))
			## print ("Gyro x: {0} ; Gyro y : {1} ; Gyro z : {2}".format(self.imu.GyroVals[0], self.imu.GyroVals[1], self.imu.GyroVals[2]))
			## print("Kalmanroll:{0} KalmanPitch:{1} KalmanYaw:{2} ".format(self.sensorfusionKalman.roll, self.sensorfusionKalman.pitch, self.sensorfusionKalman.yaw))
			## print("KalmanrollCov:{0} KalmanPitchCov:{1} KalmanYawCov:{2} ".format(self.sensorfusionKalman.rollCovariance, self.sensorfusionKalman.pitchCovariance, self.sensorfusionKalman.yawCovariance))
			##self.sensorfusionKalman.roll 				= self.imu.roll
			##self.sensorfusionKalman.pitch 			= self.imu.pitch
			#self.sensorfusionKalman.yaw 				= 1
			
			#if abs(self.sensorfusionKalman.roll) < 0.8 or abs(self.sensorfusionKalman.pitch) < 0.8:
			#	continue
			#self.sensorfusionKalman.roll	= round(self.sensorfusionKalman.roll / 2, 2)
			#if abs(self.sensorfusionKalman.roll) < 0.5:
			#	self.sensorfusionKalman.roll	= 0.0
			
			#self.sensorfusionKalman.pitch	= round(self.sensorfusionKalman.pitch / 2, 2)
			#if abs(self.sensorfusionKalman.pitch) < 0.5:
			#	self.sensorfusionKalman.pitch	= 0.0
			

			print('K = ', self.sensorfusionKalman.roll, self.sensorfusionKalman.pitch, self.sensorfusionKalman.yaw)
			#print('M = ', self.sensorfusionMadgwick.roll, self.sensorfusionMadgwick.pitch, self.sensorfusionMadgwick.yaw)

			quaternion 							= tf.transformations.quaternion_from_euler(self.sensorfusionKalman.roll, self.sensorfusionKalman.pitch, self.sensorfusionKalman.yaw)

			quaternionXcal						= quaternion[0]
			quaternionYcal						= quaternion[1]
			quaternionZcal						= quaternion[2]
			quaternionWcal						= quaternion[3]

			self.imu_data.header.stamp			= rospy.Time.now()
			self.imu_data.header.frame_id		= 'base_link'
			self.imu_data.angular_velocity.x 	= gyroX
			self.imu_data.angular_velocity.y 	= gyroY
			self.imu_data.angular_velocity.z 	= gyroZ
			self.imu_data.linear_acceleration.x = accX
			self.imu_data.linear_acceleration.y = accY
			self.imu_data.linear_acceleration.z = accZ

			self.imu_data.orientation.x 		= -1 * quaternionXcal
			self.imu_data.orientation.y 		= 1 * quaternionYcal
			self.imu_data.orientation.z 		= 1 * quaternionZcal
			self.imu_data.orientation.w 		= 1 * quaternionWcal

			#print(self.sensorfusionKalman.roll, self.sensorfusionKalman.pitch, self.sensorfusionKalman.yaw)

			#print('quaternionXcal = ', quaternionXcal, ', quaternionYcal = ', quaternionYcal, ', quaternionZcal = ', quaternionZcal, ', quaternionWcal = ', quaternionWcal)
			#print(self.imu_data.orientation)
			self.imu_publisher.publish(self.imu_data)
			rate.sleep()

		rospy.spin()	
		
def main():
    imu_sensor = IMU_MPU9250()
    imu_sensor.run()
    #while not rospy.is_shutdown():
    #    imu_sensor.run()
    #    rospy.spin()
    #try:
    #    rate = rospy.Rate(10)
	#    while not rospy.is_shutdown():
    #        imu_sensor.run()
    #        imu_sensor.imu_publisher.publish(imu_sensor.imu_data)
    #        rate.sleep()
    #except KeyboardInterrupt:
    #    print('\n Ctrl + C QUIT')

# Main loop
if __name__ == '__main__':
	main()	


