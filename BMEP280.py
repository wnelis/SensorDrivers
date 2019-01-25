#
# Class definitions for Bosch BMP280 and Bosch BME280 sensors.
# These class definitions are specific for a Raspberry Pi, as it is assumed that
# the I2C channel number is one. The class definitions only support the forced
# mode of measurements. Using Python module smbus2, an all Python driver for
# these sensores is realised.
#
# Written by W.J.M. Nelis, wim.nelis@ziggo.nl, 2018.02
#
# To do:
# - Improve error handling, especially in method _read_raw(). Define the return
#   status in case of an I/O error. (rawtemp == None in case of failure?)
#
import math
import smbus2 as smbus
import struct
import time

#
# Constant definitions for the i2c bus.
#
I2C_CHANNEL= 1				# The i2c channel on an RPi

#
# Constant definitions for sensor BMP280.
# Define the chip register addresses and address blocks.
#
BMP280_REG_CALIBRTN= 0x88		# Start of calibration data
BMP280_LNG_CALIBRTN=   24		# Length of calibration data
BMP280_REG_CHIP_ID = 0xd0		# Identification code
BMP280_REG_RESET   = 0xe0		# Soft reset
BMP280_REG_STATUS  = 0xf3		# Chip status
BMP280_REG_CONTROL = 0xf4		# Control data acquisition
BMP280_REG_CONFIG  = 0xf5		# Configuration options
BMP280_REG_RAW_DATA= 0xf7		# Start of raw measurement data
BMP280_LNG_RAW_DATA=    6		# Length

BMP280_VAL_CHIP_ID = 0x58		# Chip identifier of BMP280
BMP280_VAL_RESET   = 0xb6		# Perform soft reset
BMP280_VAL_CONTROL = 0xb5		# to=16, po=16, mode is forced
BMP280_VAL_CONFIG  = 0xe0		# ts=4000, iir is off, spi disabled

#
# Define class BMP280, which handles a Bosch BMP280, which contains a pressure
# sensor and a temperature sensor. This class assumes that all measurements are
# performed in forced mode. Thus the normal (repetitive) mode is not supported.
#
class BMP280( smbus.SMBus ):

  def __init__( self, device ):
    super().__init__( I2C_CHANNEL )	# Parent initialisation

  # Preset instance variables.
    self.device  = device		# i2c slave device address
    self.rawtemp = None			# Last temperature measurement
    self.rawpres = None			# Last pressure measurement
    self.t_fine  = None			# Compensation temperature value
    self.livetime= 2			# Live time of one measurement [s]
    self.tom= time.time() - self.livetime - 1	# Time of last measurement

    self._check_chip()			# Check device address and chip
    self._config_chip()			# Set configuration register
    self._load_calibration()		# Fetch calibration data

 #
 # Method _check_chip checks if the chip identifier matches the expected value.
 # Finding the chip identifier also implies that the chip is operational.
 #
  def _check_chip( self ):
    chip_id= self.read_byte_data( self.device, BMP280_REG_CHIP_ID )
    if chip_id != BMP280_VAL_CHIP_ID:
      raise ValueError( "Unexpected chip identifier: %04x" % chip_id )

  def _config_chip( self ):
    self.write_byte_data( self.device, BMP280_REG_CONFIG, BMP280_VAL_CONFIG )

 #
 # Method _load_calibration loads the calibration data, also called the
 # compensation parameters, from the chip and saves them in this instance
 # itself. The algorithm is taken from URL
 # https://github.com/adafruit/Adafruit_CircuitPython_BMP280/blob/master/adafruit_bmp280.py
 #
  def _load_calibration( self ):
    self.cpt= [0]*3			# Preset calibration parameters
    self.cpp= [0]*9

    c= self.read_i2c_block_data( self.device, BMP280_REG_CALIBRTN, BMP280_LNG_CALIBRTN )
    c= list(struct.unpack( '<HhhHhhhhhhhh', bytes(c) ))	# Extract (un)signed shorts
    c= [ float(i)  for i in c ]		# Convert to floating point format

    self.cpt= c[:3]			# Move to appropiate lists
    self.cpp= c[3:]

 #
 # Method _read_raw retrieves the current (raw) measurements of the pressure and
 # the temperature. The mode of operation is forced mode, thus the sensor must
 # be triggered to perform a measurement. At most one measurement per livetime
 # seconds is performed.
 #
  def _read_raw( self ):
    now= time.time()
    if self.tom < now-self.livetime:
  #
  # Select force mode and wait for the measurement to be complete. The time
  # typically needed is 1.5 + 2*(osr_t + osr_p) [ms]. Check every 10 [ms] if the
  # measurement is completed.
  #
      self.write_byte_data( self.device, BMP280_REG_CONTROL, BMP280_VAL_CONTROL )
      m= 1				# A random integer non-zero value
      while m != 0:
        time.sleep( 0.010 )
        try:
          m= self.read_register( BMP280_REG_STATUS ) & 0x08
        except OSError as err:
          pass				# Ignore I/O error

      try:
        d= self.read_i2c_block_data( self.device, BMP280_REG_RAW_DATA, BMP280_LNG_RAW_DATA )
      except OSError as err:
        pass

      self.rawpres= ((d[0]<<16) + (d[1]<<8) + d[2])>>4
      self.rawtemp= ((d[3]<<16) + (d[4]<<8) + d[5])>>4
      self.t_fine = None
      self.tom    = now

 #
 # Return the calibrated pressure. The formulas are taken from the Bosch BMP280
 # datasheet, section 8.1.
 #
  def read_pressure( self ):
    self._read_raw()
    if self.t_fine is None:  self.read_temperature()

    var1= float(self.t_fine)/2.0 - 64000.0
    var2= var1*var1*self.cpp[5]/32768.0
    var2= var2 + var1*self.cpp[4]*2.0
    var2= var2/4.0 + self.cpp[3]*65536.0
    var1= (var1*self.cpp[2]/524288.0 + self.cpp[1])*var1/524288.0
    var1= (1.0 + var1/32768.0)*self.cpp[0]
    if var1 == 0.0:  return var1
    pres= 1048576.0 - self.rawpres
    pres= (pres - var2/4096.0)*6250.0/var1
    var1= pres*pres*self.cpp[8]/2147483648.0
    var2= pres*self.cpp[7]/32768.0
    pres= pres + (var1 + var2 + self.cpp[6])/16.0
    return pres

 #
 # Return the content of one register, a byte value. The register address can be
 # in the range [0x80,0xff].
 #
  def read_register( self, register ):
    if register < 0x80  or  register > 0xff:
      raise ValueError( 'Register address out of range: %d' % register )
    return self.read_byte_data( self.device, register )

 #
 # Return the calibrated temperature. The formulas are taken from the Bosch
 # BMP280 datasheet, section 8.1. Note that a temperature value (t_fine) is
 # saved as an instance variable: it is needed to calculate the calibrated
 # pressure.
 #
  def read_temperature( self ):
    self._read_raw()
    var1= (self.rawtemp/ 16384.0 - self.cpt[0]/1024.0)*self.cpt[1]
    var2=  self.rawtemp/131072.0 - self.cpt[0]/8192.0
    var2= var2*var2*self.cpt[2]
    self.t_fine= int( var1 + var2 )
    return (var1 + var2)/5120.0

  def reset( self ):
    self.write_byte_data( self.device, BMP280_REG_RESET, BMP280_VAL_RESET )
    time.sleep( 0.002 )			# Startup time


#
# Constant definitions for sensor BME280.
# Define the chip register addresses and address blocks.
#
BME280_REG_CALIBRTN= BMP280_REG_CALIBRTN	# Start of calibration data
BME280_LNG_CALIBRTN= BMP280_LNG_CALIBRTN	# Length of calibration data
BME280_REG_DIGH1   = 0xa1			# Humidity calibration H1
BME280_REG_CHIP_ID = BMP280_REG_CHIP_ID		# Identification code
BME280_REG_RESET   = BMP280_REG_RESET		# Soft reset
BME280_REG_DIGH2   = 0xe1			# Humidity calibration H2..H6
BME280_LNG_DIGH2   =    7			# Length
BME280_REG_STATUS  = BMP280_REG_STATUS		# Chip status
BME280_REG_CONTROL0= BMP280_REG_CONTROL		# Control data acquisition
BME280_REG_CONTROL1= 0xf2			# Control humidity data acquisition
BME280_REG_CONFIG  = BMP280_REG_CONFIG		# Configuration options
BME280_REG_RAW_DATA= BMP280_REG_RAW_DATA	# Start of raw measurement data
BME280_LNG_RAW_DATA=    8			# Length

BME280_VAL_CHIP_ID = 0x60			# Chip identifier of BME280
BME280_VAL_RESET   = BMP280_VAL_RESET		# Perform soft reset
BME280_VAL_CONTROL0= BMP280_VAL_CONTROL		# to=16, po=16, mode is forced
BME280_VAL_CONTROL1= 0x05			# ho=16
BME280_VAL_CONFIG  = BMP280_VAL_CONFIG		# ts=4000, iir is off, spi disabled

#
# Define class BME280, which handles a Bosch BME280, which contains a pressure
# sensor, a temperature sensor and a humidity sensor. This class assumes that
# all measurements are performed in forced mode. Thus the normal (repetitive)
# mode is not supported.
#
class BME280( BMP280 ):

  def __init__( self, device ):
    super().__init__( device )		# Parent initialisation

  # Presets which are specific to a BME280.
    self.write_byte_data( self.device, BME280_REG_CONTROL1, BME280_VAL_CONTROL1 )
    self.rawhumi = None			# Last humidity measurement

 #
 # Method _check_chip checks if the chip identifier matches the expected value.
 # Finding the chip identifier also implies that the chip is operational.
 #
  def _check_chip( self ):
    chip_id= self.read_byte_data( self.device, BME280_REG_CHIP_ID )
    if chip_id != BME280_VAL_CHIP_ID:
      raise ValueError( "Unexpected chip identifier: %04x" % chip_id )

 #
 # Method _load_calibration loads the calibration data, also called the
 # compensation parameters, from the chip and saves them in this instance
 # itself.
 #
  def _load_calibration( self ):
    super()._load_calibration()		# Do the BMP280 compatible part

    self.cph   = [0]*6			# Preset calibration parameters
    self.cph[0]= float(self.read_byte_data( self.device, BME280_REG_DIGH1 ))

    c= self.read_i2c_block_data( self.device, BME280_REG_DIGH2, BME280_LNG_DIGH2 )
    c= list(struct.unpack( '<hBBBBb', bytes(c) ))
    self.cph[1]= float(c[0])
    self.cph[2]= float(c[1])
    self.cph[3]= float((c[2]<<4) | (c[3] & 0x0F))
    self.cph[4]= float(((c[3] & 0xF0)<<4) | c[4])
    self.cph[5]= float(c[5])

 #
 # Method _read_raw retrieves the current (raw) measurements of the pressure,
 # the temperature and the humidity. The mode of operation is forced mode, thus
 # the sensor must be triggered to perform a measurement. At most one
 # measurement per livetime seconds is performed.
 #
  def _read_raw( self ):
    now= time.time()
    if self.tom < now-self.livetime:
  #
  # Select force mode and wait for the measurement to be complete. Note that the
  # humidity control register MUST be set prior to invoking this method! The
  # time typically needed is 2 + 2*(osr_t + osr_p + osr_h) [ms]. Check every 10
  # [ms] if the measurement is completed.
  #
      self.write_byte_data( self.device, BME280_REG_CONTROL0, BME280_VAL_CONTROL0 )
      m= 1				# A random integer non-zero value
      while m != 0:
        time.sleep( 0.010 )
        m= self.read_register( BME280_REG_STATUS ) & 0x08

      d= self.read_i2c_block_data( self.device, BME280_REG_RAW_DATA, BME280_LNG_RAW_DATA )
      self.rawpres= ((d[0]<<16) + (d[1]<<8) + d[2])>>4
      self.rawtemp= ((d[3]<<16) + (d[4]<<8) + d[5])>>4
      self.rawhumi=  (d[6]<< 8) +  d[7]
      self.t_fine = None
      self.tom    = now

 #
 # Return the calibrated humidity. The formulas were initially taken from the
 # Bosch BME280 datasheet, section 8.1. However, the sample code published by
 # the manufacturer contains a slightly different formula (see
 # https://github.com/BoschSensortec/BME280_driver/blob/master/bme280.c). As
 # this source is taken to be authorative, the sample code is used.
 #
 # A test was performed in which the results of both formulas were compared when
 # computing the calibrated temperature in an office room. Never a relative
 # difference exceeding 1E-6 was found.
 #
 # Note that a temperature value (t_fine) is saved as an instance variable: it
 # is needed to calculate the calibrated humidity.
 #
  def read_humidity( self ):
    self._read_raw()
    if self.t_fine is None:  self.read_temperature()

    var1= float(self.t_fine) - 76800.0
    var2= self.cph[3] * 64.0 + self.cph[4] / 16384.0 * var1
    var3= float(self.rawhumi) - var2
    var4= self.cph[1] / 65536.0
    var5= 1.0 + (self.cph[2] / 67108864.0) * var1
    var6= 1.0 + (self.cph[5] / 67108864.0) * var1 * var5
    var6= var3 * var4 * var5 * var6
    humi= var6 * (1.0 - self.cph[0] * var6 / 524288.0)

    if humi > 100.0:  humi= 100.0
    if humi <   0.0:  humi=   0.0
    return humi

#
# Method read_dew_point calculates the dew point temperature, given the
# temperature and the relative humidity. The formula and the constants are
# taken from URL https://nl.wikipedia.org/wiki/Dauwpunt. See also URL
# https://en.wikipedia.org/wiki/Vapour_pressure_of_water
#
# The computation is using the Tetens approximation formula between temperature
# T and the maximal vapor pressure P twice. The formula is:
#  P= c*exp( a*T/(b+T) )			(0)
# The inverse of this formula is:
#  T= b*ln(P/c)/( a - ln(P/c) )			(1)
# Multiplying P for the current temperature T with the relative humidity and
# substituting this product in formula (1) results in the formula's used in
# this method.
#
  def read_dew_point( self ):
    temp= self.read_temperature()
    humi= self.read_humidity() / 100.0
    a=  17.27				# []
    b= 237.7				# [C]
    gamma= math.log(humi) + a*temp/(b + temp)
    return b*gamma/(a - gamma)

