import smbus
import math
import time
from registers import *

class MPU6050:

    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    
    global t_prev
    t_prev = int(time.time()*1000000.0)

    # Master and Slave Biases
    gbias = [0, 0, 0] # Gyroscope Master Bias
    abias = [0, 0, 0] # Accelerometer Master Bias
    mbias = [0, 0, 0]  # Magnetometer Hard Iron Distortion

    def __init__(self, address, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    # I2C communication methods

    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.

        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    # MPU-6050 Methods

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050.

        Returns the temperature in degrees Celcius.
        """
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)

        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53

        return actual_temp

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range.

        accel_range -- the range to set the accelerometer to. Using a
        pre-defined range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw = False):
        """Reads the range the accelerometer is set to.

        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g = False):
        """Gets and returns the X, Y and Z values from the accelerometer.

        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier = None
        accel_range = self.read_accel_range(True)

        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope to range.

        gyro_range -- the range to set the gyroscope to. Using a pre-defined
        range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def read_gyro_range(self, raw = False):
        """Reads the range the gyroscope is set to.

        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope.

        Returns the read values in a dictionary.
        """
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_scale_modifier = None
        gyro_range = self.read_gyro_range(True)

        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier

        return {'x': x, 'y': y, 'z': z}

    def get_all_data(self):
        """Reads and returns all the available data."""
        temp = self.get_temp()
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()

        return [accel, gyro, temp]   
 
    # Accelerometer Angle Degree ----------------------------
    def dist(self,a,b):
        return math.sqrt((a*a)+(b*b))

    def get_accel_rotation(self):
        accel_data = self.get_accel_data()
        radians_x = math.atan2(accel_data['x'], self.dist(accel_data['y'],accel_data['z']))
        radians_y = math.atan2(accel_data['y'], self.dist(accel_data['x'],accel_data['z']))
        radians_z = math.atan2(self.dist(accel_data['x'],accel_data['y']),accel_data['z'])
        return {'x': -math.degrees(radians_x), 'y': math.degrees(radians_y), 'z': -math.degrees(radians_z)}    
  
    # Gyroscope Angle Degree --------------------------------    
    def get_gyro_rotation(self):
        global t_prev
        gyro_data = self.get_gyro_data()
        t_now = int(time.time()*1000000.0)
        dt_n = t_now - t_prev
        t_prev = t_now
        dt = dt_n /1000000
        
        gyro_anGle_x = gyro_data['x'] * dt
        gyro_anGle_y = gyro_data['y'] * dt
        gyro_anGle_z = gyro_data['z'] * dt
        return {'x': gyro_anGle_x, 'y': gyro_anGle_y, 'z': gyro_anGle_z}        

    def get_sensor_avg(self, samples, softstart=100):
        """Return the average readings from the sensors over the
        given number of samples.  Discard the first softstart
        samples to give things time to settle."""
        sample = self.read_sensors()
        counters = [0] * 7

        for i in range(samples + softstart):
            # the sleep here is to ensure we read a new sample
            # each time
            time.sleep_ms(2)  # type: ignore[attr-defined]

            sample = self.read_sensors()
            if i < softstart:
                continue

            for j, val in enumerate(sample):
                counters[j] += val

        return SensorReadings(*[x // samples for x in counters])

    # Data Convert
    # @param [in] self - The object pointer.
    # @param [in] data1 - LSB
    # @param [in] data2 - MSB
    # @retval Value: MSB+LSB(int 16bit)
    def dataConv(self, data1, data2):

        value = data1 | (data2 << 8)

        if(value & (1 << 16 - 1)):
            value -= (1 << 16)

        return value


    def writeMaster(self, register, value, sleep = 0):
        
        self.bus.write_byte_data(self.address, register, value)

        if sleep > 0:
            time.sleep(sleep)

    def readMaster(self, register, quantity):
        return self.bus.read_i2c_block_data(self.address, register, quantity)

    # This function calibrate MPU6500 and load biases to params in this class.
    # To calibrate, you must correctly position the MPU so that gravity is all along the z axis of the accelerometer.
    # This function accumulates gyro and accelerometer data after device initialization. It calculates the average
    # of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
    # This function reset sensor registers. Configure must be called after.
    #  @param [in] self - The object pointer.
    def calibrateMPU6500(self):

        # get stable time source; Auto select clock source to be PLL gyroscope reference if ready, else use the internal oscillator, bits 2:0 = 001
        self.writeMaster(PWR_MGMT_1, 0x01)
        self.writeMaster(PWR_MGMT_2, 0x00, 0.2)

        # Configure device for bias calculation
        self.writeMaster(INT_ENABLE, 0x00) # Disable all interrupts
        self.writeMaster(FIFO_EN, 0x00) # Disable FIFO
        self.writeMaster(PWR_MGMT_1, 0x00) # Turn on internal clock source
        self.writeMaster(I2C_MST_CTRL, 0x00) # Disable I2C master
        self.writeMaster(USER_CTRL, 0x00) # Disable FIFO and I2C master modes
        self.writeMaster(USER_CTRL, 0x0C, 0.015) # Reset FIFO and DMP

        # Configure MPU6500 gyro and accelerometer for bias calculation
        self.writeMaster(CONFIG, 0x01) # Set low-pass filter to 188 Hz
        self.writeMaster(SMPLRT_DIV, 0x00) # Set sample rate to 1 kHz
        self.writeMaster(GYRO_CONFIG, 0x00) # Set gyro full-scale to 250 degrees per second, maximum sensitivity
        self.writeMaster(ACCEL_CONFIG, 0x00) # Set accelerometer full-scale to 2G, maximum sensitivity

        # Configure FIFO to capture accelerometer and gyro data for bias calculation
        self.writeMaster(USER_CTRL, 0x40) # Enable FIFO
        self.writeMaster(FIFO_EN, 0x78, 0.04) # Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150) # 0.4 - accumulate 40 samples in 40 milliseconds = 480 bytes

        # At end of sample accumulation, turn off FIFO sensor read
        self.writeMaster(FIFO_EN, 0x00) # Disable gyro and accelerometer sensors for FIFO
      
        # read FIFO sample count
        data = self.readMaster(FIFO_COUNTH, 2) 
        fifo_count = self.dataConv(data[1], data[0])
        packet_count = int(fifo_count / 12); # How many sets of full gyro and accelerometer data for averaging

        index = 0
        accel_bias = [0, 0, 0] 
        gyro_bias = [0, 0, 0]

        while index < packet_count:

            # read data for averaging
            data = self.readMaster(FIFO_R_W, 12) 

            # Form signed 16-bit integer for each sample in FIFO
            # Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            accel_bias[0] += self.dataConv(data[1], data[0]) 
            accel_bias[1] += self.dataConv(data[3], data[2])
            accel_bias[2] += self.dataConv(data[5], data[4])
            gyro_bias[0] += self.dataConv(data[7], data[6])
            gyro_bias[1] += self.dataConv(data[9], data[8])
            gyro_bias[2] += self.dataConv(data[11], data[10])

            index += 1

        # Normalize sums to get average count biases
        accel_bias[0] /= packet_count 
        accel_bias[1] /= packet_count
        accel_bias[2] /= packet_count
        gyro_bias[0] /= packet_count
        gyro_bias[1] /= packet_count
        gyro_bias[2] /= packet_count

        # Remove gravity from the z-axis accelerometer bias calculation
        if accel_bias[2] > 0:
            accel_bias[2] -= ACCEL_SCALE_MODIFIER_2G_DIV
        else:
            accel_bias[2] += ACCEL_SCALE_MODIFIER_2G_DIV

        # Output scaled gyro biases for display in the main program
        self.gbias = [
            (gyro_bias[0]),
            (gyro_bias[1]),
            (gyro_bias[2])
        ]

        # Output scaled accelerometer biases for manual subtraction in the main program
        self.abias = [
            (accel_bias[0]),
            (accel_bias[1]),
            (accel_bias[2])
        ]

        ## Output scaled gyro biases for display in the main program
        #self.gbias = [
        #    (gyro_bias[0] / GYRO_SCALE_MODIFIER_250DEG_DIV),
        #    (gyro_bias[1] / GYRO_SCALE_MODIFIER_250DEG_DIV),
        #    (gyro_bias[2] / GYRO_SCALE_MODIFIER_250DEG_DIV)
        #]

        ## Output scaled accelerometer biases for manual subtraction in the main program
        #self.abias = [
        #    (accel_bias[0] / ACCEL_SCALE_MODIFIER_2G_DIV),
        #    (accel_bias[1] / ACCEL_SCALE_MODIFIER_2G_DIV),
        #    (accel_bias[2] / ACCEL_SCALE_MODIFIER_2G_DIV)
        #]


if __name__ == "__main__":
    mpu = mpu6050(0x68)
    
    print(mpu.get_temp())

    accel_data = mpu.get_accel_data()
    print(accel_data['x'])
    print(accel_data['y'])
    print(accel_data['z'])
    
    gyro_data = mpu.get_gyro_data()
    print(gyro_data['x'])
    print(gyro_data['y'])
    print(gyro_data['z'])
