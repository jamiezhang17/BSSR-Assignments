# Name: Jamie Zhang
# Year and program: 2T9 + PEY ECE
# UofT email: jamiejamie.zhang@mail.utoronto.ca

# Please Note! i added comments to a lot of places  
# as a way of self-learning and explaining to myself what everything means

"""
Implement the functions accordingly. You can search TODO in this document. 
Provide explanations to demonstrate your understanding and cite relevant 
section of the datasheet.

Other than that there is 1 mistake in the code. Correct that and add comments
to provide explanations. 
"""

"""This module implements the LIS2HH12 driver."""

from dataclasses import dataclass, field
from enum import IntEnum

from periphery import GPIO, I2C

from iclib.utilities import twos_complement

import time

class Register(IntEnum):
    TEMP_L = 0x0B
    TEMP_H = 0x0C
    WHO_AM_I = 0x0F
    ACT_THS = 0x1E
    ACT_DUR = 0x1F
    CTRL1 = 0x20
    CTRL2 = 0x21
    CTRL3 = 0x22
    CTRL4 = 0x23
    CTRL5 = 0x24
    CTRL6 = 0x25
    CTRL7 = 0x26
    STATUS = 0x27
    OUT_X_L = 0x28
    OUT_X_H = 0x29
    OUT_Y_L = 0x2A
    OUT_Y_H = 0x2B
    OUT_Z_L = 0x2C
    OUT_Z_H = 0x2D
    FIFO_CTRL = 0x2E
    FIFO_SRC = 0x2F
    IG_CFG1 = 0x30
    IG_SRC1 = 0x31
    IG_THS_X1 = 0x32
    IG_THS_Y1 = 0x33
    IG_THS_Z1 = 0x34
    IG_DUR1 = 0x35
    IG_CFG2 = 0x36
    IG_SRC2 = 0x37
    IG_THS2 = 0x38
    IG_DUR2 = 0x39
    XL_REFERENCE = 0x3A
    XH_REFERENCE = 0x3B
    YL_REFERENCE = 0x3C
    YH_REFERENCE = 0x3D
    ZL_REFERENCE = 0x3E
    ZH_REFERENCE = 0x3F

@dataclass
class LIS2HH12:
    """A Python driver for STMicroelectronics LIS2HH12 accelerometer"""
    
    i2c: I2C
    """The I2C bus."""
    sa0_pin: bool
    """The device version."""
    output_data_rate: int = field(init=False, default=0)
    measurement_range: int = field(init=False, default=2)
    address: int = field(init=False)
    """The address on I2C bus."""

    ## is the error here that the address was in binary format instead of hexadecimal???
    def __post_init__(self) -> None: 
        if self.sa0_pin: # SAD[1] (write command)
            self.address = 0x1E # 0b0011110 the 7-bit address when SA0 = 1 (last bit is Read/Write)
        else: # SAD[0] (write command)
            self.address = 0x1D # 0b0011101 the 7-bit address when SA0 = 0
    ## reference: from datasheet 6.1.1 i2c operation, table 13

    @dataclass
    class Vector:
        x: float
        y: float
        z: float

        def __add__(self, other: 'LIS2HH12.Vector') -> 'LIS2HH12.Vector':
            return LIS2HH12.Vector(
                self.x + other.x,
                self.y + other.y,
                self.z + other.z
            )

        def __sub__(self, other: 'LIS2HH12.Vector') -> 'LIS2HH12.Vector':
            return LIS2HH12.Vector(
                self.x - other.x,
                self.y - other.y,
                self.z - other.z
            )

        def __iter__(self):
            return iter((self.x, self.y, self.z))

    # i2c bus writes a byte to a register 
    def write(self, register: Register, data: int) -> None:
        # parameters: 
        # register: the register address to write to 
        # data: a byte storing a decimal 0-255)
        message = I2C.Message([register, data]) 

        self.i2c.transfer(self.address, [message])

    # writing/modifying specific bits back to register (question: why do we need to modify?)
    def write_bits(
        self,
        register: Register,
        bits: dict[int, bool]
    ) -> None:
        # parameters: 
        # register: register address
        # bits: bit positions (0-7) to boolean values
        raw = self.read(register, 1)[0] & 0xFF # reading current bit

        # from datasheet 6.1.1 12c operation 
        # The Slave Address (SAD) associated to the LIS2HH12 is 00111xxb where the xx bits are
        # modified by the SA0/SDO pin in order to modify the device address. If the SA0/SDO pin is
        # connected to the supply voltage, the address is 0011101b, otherwise if the SA0/SDO pin is
        # connected to ground, the address is 0011110b. This solution permits to connect and
        # address two different accelerometers to the same I2C lines.
        for bit, value in bits.items(): 
            if value: # value = boolean (true = 1, false = 0)
                raw |= 1 << bit # sets bit to 1
            else:
                raw &= ~(1 << bit) # sets bit to 0

        self.write(register, raw) # write back modified bits to register

    # master device reads data from slave device register
    def read(self, register: Register, length: int) -> list[int]:
        # parameters:
        # register: starting register address
        # length: number of bytes to read

        if length > 1: # auto-increments the bit for reading consecutive registers (most significant bit MSB of register address)
            register |= 0x80 

        write_message = I2C.Message([register]) # write target register (slave) address to read from
        read_message = I2C.Message([0] * length, read=True) # reads back the data returned 
        self.i2c.transfer(self.address, [write_message, read_message]) # perform the i2c transaction (start, write data to slave, stop)
        
        return list(read_message.data) # returns list of bytes read

    def close(self) -> None: 
        self.i2c.close() # close the i2c bus

    ######### configuring stuff and turning on the sensor 
    def config(
            self,
            odr: int | None = None, 
            measurement_range: int | None = None,
            enable_axes: bool = False,
            enable_auto_inc: bool = False,
    ) -> None:
        # parameters:
        # odr: output data rate in Hz (0, 10, 50, 100, 200, 400, 800)
        # measurement_range: measurement range in g (2, 4, 8)
        # enable_axes: enable all axes: X, Y, Z (1 = enable, 0 = disable)
        # enable_auto_inc: enable auto-increment of the register address for consecutive byte readings

        # **** configure the accelerometer **********
        if odr is not None: # handle output data rate (CTRL1 bits 6-4: ODR2-ODR0)
            if odr not in [0, 10, 50, 100, 200, 400, 800]: ## these values are valid Output Data Rates selections in Hz (collecting samples rate)
                raise ValueError('invalid output data rate')
            
            # from datasheet 8.5 CTRL1 (20h) table 24: 
            # ODR [2:0] is used to set the power mode and ODR selection
            match odr: # matching odr to bit pattern 
                case 0: # power down 
                    odr_bits = {6: 0, 5: 0, 4: 0} # 6:5:4 bits for ODR2-ODR0, odr2 = 0, odr1 = 0, odr0 = 0
                case 10: # odr selection = 10 hz
                    odr_bits = {6: 0, 5: 0, 4: 1} # odr2 = 0, odr1 = 0, odr0 = 1
                case 50: # odr selection = 50 hz
                    odr_bits = {6: 0, 5: 1, 4: 0} # odr2 = 0, odr1 = 1, odr0 = 0
                case 100: # odr selection = 100 hz
                    odr_bits = {6: 0, 5: 1, 4: 1} # odr2 = 0, odr1 = 1, odr0 = 1
                case 200: # odr selection = 200 hz
                    odr_bits = {6: 1, 5: 0, 4: 0} # odr2 = 1, odr1 = 0, odr0 = 0   
                case 400: # odr selection = 400 hz
                    odr_bits = {6: 1, 5: 0, 4: 1} # odr2 = 1, odr1 = 0, odr0 = 1
                case 800: # odr selection = 800 hz
                    odr_bits = {6: 1, 5: 1, 4: 0} # odr2 = 1, odr1 = 1, odr0 = 0
            
            self.output_data_rate = odr
            self.write_bits(Register.CTRL1, odr_bits)

        # TODO: handle measurement_range, enable_axes, enable_auto_inc here
        # should do nothing for enable_axes, enable_auto_inc if false

        # ***** configure measurement range: using CTRL4 for data output from sensor **********
        if measurement_range is not None: # handle measurement range (CTRL4 bits 5:4 for FS1-FS0)
            if measurement_range not in [2, 4, 8]:
                raise ValueError('invalid measurement range') # error message

            match measurement_range: # match range to bit pattern for FS [1:0] = full scale selection
                # what is full scale selection? 
                # determines the min/max values a sensor can measure accurately
                # choosing a range close to the expected maximum signal give better results
                case 2:
                    fs_bits = {5: 0, 4: 0} # ±2g
                case 4:
                    fs_bits = {5: 1, 4: 0} # ±4g
                case 8:
                    fs_bits = {5: 1, 4: 1} # ±8g

                    # note from 8.8 CTRL4 (23h), table 31 row 2 (FS [1:0]):  
                    # 00: ±2g (default)
                    # 01: not available  
                    # 10: ±4g
                    # 11: ±8g

            self.measurement_range = measurement_range
            self.write_bits(Register.CTRL4, fs_bits)
            
        # ****** configure axes-enable using CTRL1 *************
        # use 2:0 bits of CTRL1 to enable 
        # bit 2: Z-axis enable. Default value: 1 
        # bit 1: Y-axis enable. Default value: 1 
        # bit 0: X-axis enable. Default value: 1 
        # axes_bits = {0: 1, 1: 1, 2: 1} where X,Y,Z axes default state (already enabled)
        if enable_axes: # if true, set bits 0,1,2 to 1
            axes_bits = {0: 1, 1: 1, 2: 1} # leave in enabled state
        else: # if false, set bits 0,1,2 to 0
            axes_bits = {0: 0, 1: 0, 2: 0} # do nothing: disable all axes
        
        self.write_bits(Register.CTRL1, axes_bits) # write the axes configuration back to CTRL1 register

        # ************ Configure auto-increment (CTRL4) ***********
        # IF_ADD_INC: Register address is automatically incremented during multiple byte access 
        # with a serial interface (I2C or SPI)
        # (0: disabled, 1: enabled) uses 2 bits
        # from datasheet 6.2: 
        # In multiple read/write commands additional blocks of 8 clock periods will be added. When
        # the CTRL4 (23h) (IF_ADD_INC) bit is ‘0’, the address used to read/write data remains the
        # same for every block. When the CTRL4 (23h) (IF_ADD_INC) bit is ‘1’, the address used to
        # read/write data is increased at every block.
        if enable_auto_inc: # if true, set bit 2 to 1
            self.write_bits(Register.CTRL4, {2: 1}) # enables auto-increment and writes back the modified bits to CTRL4 register
        else: # if false, set bit 2 to 0
            self.write_bits(Register.CTRL4, {2: 0}) # do nothing: disable auto-increment

    # read temperature from sensor 
    def read_temperature(self):
        raw = self.read(Register.TEMP_L, 2) # read temperature low and high bytes

        # combine bytes, convert from two's complement to signed integer
        # TEMP_L is LSB, TEMP_H is MSB
        raw_temp = twos_complement((raw[1] << 8 | raw[0]), 16) 
        temp = 25.0 + raw_temp / 8.0 # temperature sensor output has 8 LSB per °C sensitivity
        return temp # returns float: temperature in degrees celsius
        # 2.3 Temperature sensor characteristics, page 11

    # ********* reads raw acceleration data from sensor in LSB units **********
    def read_acceleration(self) -> 'LIS2HH12.Vector':
        # TODO: read acceleration and return a 'LIS2HH12.Vector'
        
        # read all 6 acceleration registers in one transaction
        # OUT_X_L (0x28) through OUT_Z_H (0x2D)
        # auto-increment will move through X, Y, Z registers
        # from datasheet 5.3.7: 
        # FIFO (first in, first out) data is read through the OUT_X_L (28h) - OUT_X_H (29h), OUT_Y_L (2Ah) -
        # OUT_Y_H (2Bh), OUT_Z_L (2Ch) - OUT_Z_H (2Dh) registers... 
        # ... Each time data is read from the FIFO, the oldest X, Y and Z data are placed
        # into the OUT_X, OUT_Y and OUT_Z registers and both single read and read_burst
        # operations can be used
        data = self.read(Register.OUT_X_L, 6)
        
        # convert from two's compliment to signed 16-bit integers 
        # little-endian format: low byte first, high byte second
        # shift the high byte 8 bits left, combine it with the low byte, result is a signed value
        # negative values mean negative acceleration along that axis
        x_convert = twos_complement((data[1] << 8) | data[0], 16) # OUT_X_H:OUT_X_L
        y_convert = twos_complement((data[3] << 8) | data[2], 16) # OUT_Y_H:OUT_Y_L
        z_convert = twos_complement((data[5] << 8) | data[4], 16) # OUT_Z_H:OUT_Z_L
    
        # sensitivity from datasheet 2.1 table 3 row 2:
        # convert mg to g units (gravitational acceleration / digit))
        # ±2g: 0.061 mg/LSB = 0.000061 g/LSB 
        # ±4g: 0.122 mg/LSB = 0.000122 g/LSB  
        # ±8g: 0.244 mg/LSB = 0.000244 g/LSB
        # sensitivity (converted to g/digit) 
        sensitivity_map = {
            2: 0.000061,  # @ FS ±2g
            4: 0.000122,  # @ FS ±4g
            8: 0.000244   # @ FS ±8g
        }
        
        sensitivity = sensitivity_map.get(self.measurement_range, 0.000061)
        
        # Convert to g and return as Vector: raw signed 16-bit values (x, y, z) 
        return self.Vector(
            x = x_convert * sensitivity,
            y = y_convert * sensitivity, 
            z = z_convert * sensitivity
        )
    

    def self_test(self) -> bool: # ST = self-test
        # TODO: implement the self-test function
        # from datasheet 2.6.3: 
        # When the self-test is activated, the
        # device output level is given by the algebraic sum of the signals produced by the acceleration
        # acting on the sensor and by the electrostatic test-force. If the output signals change within
        # the amplitude specified in Table 3, then the sensor is working properly and the parameters
        # of the interface chip are within the defined specifications.

        # 1. read initial acceleration values (averaged over 10 samples)
        initial_readings = [] # initalize array
        for _ in range(10): # take 10 readings 
            vec = self.read_acceleration() # read acceleration
            initial_readings.append((abs(vec.x), abs(vec.y), abs(vec.z))) # turn vector readings into doubles then store in array
            time.sleep(0.001)  # small delay between reads
        
        # average the initial readings
        init_x = sum(r[0] for r in initial_readings) / len(initial_readings)
        init_y = sum(r[1] for r in initial_readings) / len(initial_readings)
        init_z = sum(r[2] for r in initial_readings) / len(initial_readings)
        
        # 2. enable self-test mode 
        # from datasheet 8.9 CTRL5 (24h) table 33 row 4
        # ST[2:1] Self-test enable. Default value: 00 
        # 00: Self-test disabled (normal mode)
        # 01: positive sign self test
        # 10: negative sign self test
        # 11: not allowed
        self.write_bits(Register.CTRL5, {2: 0, 1: 1}) # enable positive sign self-test
        
        # 3. read acceleration with self-test enabled
        test_readings = [] # array 
        for _ in range(10): # take 10 readings
            vec = self.read_acceleration() # read acceleration
            test_readings.append((abs(vec.x), abs(vec.y), abs(vec.z))) # turn vector readings into doubles then store in array
            time.sleep(0.001) # small delay between reads
        
        # average the test readings
        test_x = sum(r[0] for r in test_readings) / len(test_readings)
        test_y = sum(r[1] for r in test_readings) / len(test_readings)
        test_z = sum(r[2] for r in test_readings) / len(test_readings)
        
        # 4. disable self-test and return to normal mode
        self.write_bits(Register.CTRL5, {2: 0, 1: 0})
        
        # 5. calculate the absolute difference btw final and initial readings (the change in odr)
        delta_x = abs(test_x - init_x)
        delta_y = abs(test_y - init_y)
        delta_z = abs(test_z - init_z)
        
        # 6. check if changes are within expected range
        # from datasheet 2.1 table 3 row 7 (self-test positive difference)
        # min threshold: 70 mg = 0.070 g, 
        # max threshold: 1500 mg = 1.500 g
        min_thresh = 0.070  # min threshold in g
        max_thresh = 1.500  # max threshold in g
        axes_pass = [
            min_thresh <= delta_x <= max_thresh, # condition for x-axis
            min_thresh <= delta_y <= max_thresh, # condition for y-axis
            min_thresh <= delta_z <= max_thresh # condition for z-axis
        ]
        
        # return true if all axes pass (output change is within expected range)
        if all(axes_pass):
            print("  Self-test: PASS")
            return True
        else:
            print("  Self-test: FAIL")
            return False
        

# below is an example of how to use the driver

i2c_bus: I2C = I2C('/dev/apalis-i2c1')
accelerometer: LIS2HH12 = LIS2HH12(i2c_bus, True)
accelerometer.config(
    odr=100,
    measurement_range=8,
    enable_axes=True,
    enable_auto_inc=True,
)
test = accelerometer.self_test()
temperature = accelerometer.read_temperature()
accel = accelerometer.read_acceleration()

print(f'test: {test}, temperature: {temperature}, acceleration: {accel}')

"""
TODO: (Bonus) store values on the FIFO queue once it detects amplitude of 
acceleration in x direction is greater than 2g, then store values from fifo 
queue in an array. Use latched interrupt. Use Interrupt generator 1 on INT1 
pin. The interrupt pin GPIO is already setup. 
"""

int1: GPIO = GPIO('/dev/gpiochip0', 1, 'in', edge="rising")

accel_array = []

int1.poll(timeout=-1)
int1.read_event()
