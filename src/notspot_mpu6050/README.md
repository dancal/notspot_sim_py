MPU6050_RaspberryPi_ROS
=======

This repository contains a ROS-Python node which reads accelerometer and gyroscope MPU6050 code from Raspberry Pi and publishes it as /imu/data_raw rotopic. 

Credits
------------

This repository was created based on https://github.com/m-rtijn/mpu6050 

Steps for Installation
------------

1. The package depends on ``python-smbus`` or ``python3-smbus`` package. 

    sudo apt install python-smbus
or
    sudo apt install python3-smbus

2. Clone this repository 

    git clone https://github.com/ashwin-kumat/MPU6050_RaspberryPi_ROS.git
    
3. Rosrun raw_data_publisher.py after making it executable    

    rosrun MPU6050_RaspberryPi_ROS raw_data_publisher.py
    

