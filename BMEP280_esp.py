#
# Class definitions for Bosch BMP280 and Bosch BME280 sensors.
# These class definitions are specific for an ESP8266 device. As a consequence,
# these definitions are written for MicroPython. These classes only support the
# forced mode of measurements.
# These definitions are a rewrite of the definitions for a Raspberry Pi, in
# which module smbus2 is used to access the I2C bus.
#
# Written   by W.J.M. Nelis, wim.nelis@ziggo.nl, 2018.02
# - for a Raspberry Pi
#
# Rewritten by W.J.M. Nelis, wim.nelis@ziggo.nl, 2020.06
# - for a ESP8266 running microPython
#
from machine import Pin, I2C            # I2C driver
from micropython import const           # Memory minimisation
#
import math                             # Logarithm
import micropython                      # Optimisation
import struct                           # Decoding of calibration data
import time

#
# Define the optimisation level for the code generation:
#  0 - no optimisation
#  1 - do not compile assert
#
micropython.opt_level( 0 )

#
# Constant definitions for sensor BMP280.
# Define the chip register addresses and address blocks. Note that the values to
# be written to a register need to be byte strings.
#
BMP280_REG_CALIBRTN= const( 0x88 )      # Start of calibration data
BMP280_LNG_CALIBRTN= const(   24 )      # Length of calibration data
BMP280_REG_CHIP_ID = const( 0xd0 )      # Identification code
BMP280_REG_RESET   = const( 0xe0 )      # Soft reset
BMP280_REG_STATUS  = const( 0xf3 )      # Chip status
BMP280_REG_CONTROL = const( 0xf4 )      # Control data acquisition
BMP280_REG_CONFIG  = const( 0xf5 )      # Configuration options
BMP280_REG_RAW_DATA= const( 0xf7 )      # Start of raw measurement data
BMP280_LNG_RAW_DATA= const(    6 )      # Length

BMP280_VAL_CHIP_ID = const( 0x58 )      # Chip identifier of BMP280
BMP280_VAL_RESET   = b'\xb6'            # Perform soft reset
BMP280_VAL_CONTROL = b'\xb5'            # to=16, po=16, mode is forced
BMP280_VAL_CONFIG  = b'\xe0'            # ts=4000, iir is off, spi disabled

#
# Define class BMP280, which handles a Bosch BMP280, which contains a pressure
# sensor and a temperature sensor. This class assumes that all measurements are
# performed in forced mode. Thus the normal (repetitive) mode is not supported.
#
class BMP280():

  def __init__( self, i2c, device ):
  # Preset instance variables.
    self.i2c     = i2c                  # I2C object
    self.device  = device               # I2C slave device address
    self.cpt     = [0.0]*3              # Temperature calibration parameters
    self.cpp     = [0.0]*9              # Pressure calibration parameters
    self.rawtemp = None                 # Last temperature measurement
    self.rawpres = None                 # Last pressure measurement
    self.t_fine  = None                 # Compensation temperature value
    self.livetime= 2                    # Live time of one measurement [s]
    self.tom= time.time() - self.livetime - 1   # Time of last measurement

    self._check_chip()                  # Check device address and chip
    self._config_chip()                 # Set configuration register
    self._load_calibration()            # Fetch calibration data

 #
 # Private method _check_chip checks if the chip identifier matches the expected
 # value. Finding the chip identifier also implies that the chip is operational.
 #
  def _check_chip( self ):
    chip_id= self._read_register( BMP280_REG_CHIP_ID )
    assert chip_id == BMP280_VAL_CHIP_ID, \
      "Unexpected chip identifier: {:04x}".format(chip_id)

  def _config_chip( self ):
    self._write_register( BMP280_REG_CONFIG, BMP280_VAL_CONFIG )

 #
 # Private method _load_calibration loads the calibration data, also called the
 # compensation parameters, from the chip and saves them in this instance
 # itself. The algorithm is taken from URL
 # https://github.com/adafruit/Adafruit_CircuitPython_BMP280/blob/master/adafruit_bmp280.py
 #
  def _load_calibration( self ):
    c= self._read_registers( BMP280_REG_CALIBRTN, BMP280_LNG_CALIBRTN )
    c= list(struct.unpack( '<HhhHhhhhhhhh', bytes(c) )) # Extract (un)signed shorts
    c= [ float(i)  for i in c ]         # Convert to floating point format
    self.cpt= c[:3]                     # Move to appropiate lists
    self.cpp= c[3:]

 #
 # Private method _read_raw retrieves the current (raw) measurements of the
 # pressure and the temperature. The mode of operation is forced mode, thus the
 # sensor must be triggered to perform a measurement. At most one measurement
 # per livetime seconds is performed.
 #
  def _read_raw( self ):
    now= time.time()
    if self.tom < now-self.livetime:
  #
  # Select force mode and wait for the measurement to be complete. The time
  # typically needed is 1.5 + 2*(osr_t + osr_p) [ms]. Check every 10 [ms] if the
  # measurement is completed.
  #
      self._write_register( BMP280_REG_CONTROL, BMP280_VAL_CONTROL )
      m= 1                              # A random integer non-zero value
      while m != 0:
        time.sleep( 0.010 )
        try:
          m= self._read_register( BMP280_REG_STATUS ) & 0x08
        except OSError as err:
          pass                          # Ignore I/O error

      try:
        d= self._read_registers( BMP280_REG_RAW_DATA, BMP280_LNG_RAW_DATA )
      except OSError as err:
        pass

      self.rawpres= ((d[0]<<16) + (d[1]<<8) + d[2])>>4
      self.rawtemp= ((d[3]<<16) + (d[4]<<8) + d[5])>>4
      self.t_fine = None
      self.tom    = now

 #
 # Private method _read_register returns the content of one register, a byte
 # value. The register address can be in the range [0x80,0xff].
 #
  def _read_register( self, register ):
    assert 0x7f < register < 0x100, \
      'Register address out of range: {:02x}'.format(register)
    return self.i2c.readfrom_mem( self.device, register, 1 )[0]

  def _read_registers( self, register, count ):
    assert 0x7f < register  and  register+count < 0x101, \
      'Register address out of range: {:02x}'.format(register)
    return self.i2c.readfrom_mem( self.device, register, count )

 #
 # Private method _write_register writes one byte into one register. The
 # register address can be in the range [0x80,0xff].
 #
  def _write_register( self, register, value ):
    assert 0x7f < register < 0x100, \
      'Register address out of range: {:02x}'.format(register)
    return self.i2c.writeto_mem( self.device, register, value )

 #
 # Property pressure returns the calibrated pressure expressed in [Pa]. The
 # formulas are taken from the Bosch BMP280 datasheet, section 8.1.
 #
  @property
  def pressure( self ):
    self._read_raw()
    if self.t_fine is None:  self.temperature

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
 # Method reset performs a complete power-on reset.
 #
  def reset( self ):
    self._write_register( BMP280_REG_RESET, BMP280_VAL_RESET )
    time.sleep( 0.002 )                 # Startup time

 #
 # Property temperature returns the calibrated temperature expressed in [C]. The
 # formulas are taken from the Bosch BMP280 datasheet, section 8.1. Note that a
 # temperature value (t_fine) is saved as an instance variable: it is needed to
 # calculate the calibrated pressure.
 #
 # Note: the formulas below are effectively a second degree polynomial. The
 # variable t_fine = int( a*raw^2 + b*raw + c ), with:
 #   a = cpt[2]/17179869184
 #   b = cpt[1]/16384 - cpt[0]*cpt[2]/536870912
 #   c = cpt[0]*(cpt[0]*cpt[2]/67108864 - cpt[1]/1024)
 # These numbers must be computed with high precision. If this is possible in
 # advance, the temperature will become simpler and faster to compute.
 #
  @property
  def temperature( self ):
    self._read_raw()
    var0= self.rawtemp/16384.0 - self.cpt[0]/1024.0
    var1= var0*self.cpt[1]
    var0= var0*var0*self.cpt[2]/64.0
    self.t_fine= int( var0 + var1 )
    return (var0 + var1)/5120.0


#
# Constant definitions for sensor BME280.
# Define the chip register addresses and address blocks.
#
BME280_REG_CALIBRTN= BMP280_REG_CALIBRTN        # Start of calibration data
BME280_LNG_CALIBRTN= BMP280_LNG_CALIBRTN        # Length of calibration data
BME280_REG_DIGH1   = const( 0xa1 )              # Humidity calibration H1
BME280_REG_CHIP_ID = BMP280_REG_CHIP_ID         # Identification code
BME280_REG_RESET   = BMP280_REG_RESET           # Soft reset
BME280_REG_DIGH2   = const( 0xe1 )              # Humidity calibration H2..H6
BME280_LNG_DIGH2   = const(    7 )              # Length
BME280_REG_STATUS  = BMP280_REG_STATUS          # Chip status
BME280_REG_CONTROL0= BMP280_REG_CONTROL         # Control data acquisition
BME280_REG_CONTROL1= const( 0xf2 )              # Control humidity data acquisition
BME280_REG_CONFIG  = BMP280_REG_CONFIG          # Configuration options
BME280_REG_RAW_DATA= BMP280_REG_RAW_DATA        # Start of raw measurement data
BME280_LNG_RAW_DATA= const(    8 )              # Length

BME280_VAL_CHIP_ID = const( 0x60 )              # Chip identifier of BME280
BME280_VAL_RESET   = BMP280_VAL_RESET           # Perform soft reset
BME280_VAL_CONTROL0= BMP280_VAL_CONTROL         # to=16, po=16, mode is forced
BME280_VAL_CONTROL1= b'\x05'                    # ho=16
BME280_VAL_CONFIG  = BMP280_VAL_CONFIG          # ts=4000, iir is off, spi disabled

#
# Define class BME280, which handles a Bosch BME280, which contains a pressure
# sensor, a temperature sensor and a humidity sensor. This class assumes that
# all measurements are performed in forced mode. Thus the normal (repetitive)
# mode is not supported.
#
class BME280( BMP280 ):

  def __init__( self, i2c, device ):
    super().__init__( i2c, device )     # Parent initialisation

  # Presets which are specific to a BME280.
    self.rawhumi = None                 # Last humidity measurement
    self._write_register( BME280_REG_CONTROL1, BME280_VAL_CONTROL1 )

 #
 # Private method _check_chip checks if the chip identifier matches the expected
 # value. Finding the chip identifier also implies that the chip is operational.
 #
  def _check_chip( self ):
    chip_id= self._read_register( BME280_REG_CHIP_ID )
    assert chip_id == BME280_VAL_CHIP_ID, \
      "Unexpected chip identifier: {:04x}".format(chip_id)

 #
 # Private method _load_calibration loads the calibration data, also called the
 # compensation parameters, from the chip and saves them in this instance
 # itself.
 #
  def _load_calibration( self ):
    super()._load_calibration()         # Do the BMP280 compatible part

    self.cph   = [0.0]*6                # Humidity calibration parameters
    self.cph[0]= float(self._read_register( BME280_REG_DIGH1 ))

    c= self._read_registers( BME280_REG_DIGH2, BME280_LNG_DIGH2 )
    c= list(struct.unpack( '<hBBBBb', bytes(c) ))
    self.cph[1]= float(c[0])
    self.cph[2]= float(c[1])
    self.cph[3]= float((c[2]<<4) | (c[3] & 0x0F))
    self.cph[4]= float(((c[3] & 0xF0)<<4) | c[4])
    self.cph[5]= float(c[5])

 #
 # Private method _read_raw retrieves the current (raw) measurements of the
 # pressure, the temperature and the humidity. The mode of operation is forced
 # mode, thus the sensor must be triggered to perform a measurement. At most one
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
      self._write_register( BME280_REG_CONTROL0, BME280_VAL_CONTROL0 )
      m= 1                              # A random integer non-zero value
      while m != 0:
        time.sleep( 0.010 )
        m= self._read_register( BME280_REG_STATUS ) & 0x08

      d= self._read_registers( BME280_REG_RAW_DATA, BME280_LNG_RAW_DATA )
      self.rawpres= ((d[0]<<16) + (d[1]<<8) + d[2])>>4
      self.rawtemp= ((d[3]<<16) + (d[4]<<8) + d[5])>>4
      self.rawhumi=  (d[6]<< 8) +  d[7]
      self.t_fine = None
      self.tom    = now

 #
 # Property humidity returns the calibrated humidity expressed as a fraction,
 # thus a value in the range [0,1]. The formulas were initially taken from the
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
  @property
  def humidity( self ):
    self._read_raw()
    if self.t_fine is None:  self.temperature

    var0= float(self.t_fine) - 76800.0
    var1= float(self.rawhumi) - self.cph[3]*64.0 - self.cph[4]/16384.0*var0
    var2= self.cph[1]/65536.0
    var3= 1.0 + (self.cph[2]/67108864.0)*var0
    var4= 1.0 + (self.cph[5]/67108864.0)*var0*var3
    var5= var1*var2*var4
    humi= var5*(1.0 - self.cph[0]*var5/524288.0)

    humi= max( 0.0, min(100.0,humi) )
    return humi / 100.0

 #
 # Property dew_point calculates the dew point temperature expressed in [C],
 # given the temperature and the relative humidity. The formula and the
 # constants are taken from URL https://nl.wikipedia.org/wiki/Dauwpunt. See also
 # URL https://en.wikipedia.org/wiki/Vapour_pressure_of_water
 #
 # The computation is using the Tetens approximation formula between temperature
 # T and the maximal vapor pressure P twice. The formula is:
 #  P= c*exp( a*T/(b+T) )                        (0)
 # The inverse of this formula is:
 #  T= b*ln(P/c)/( a - ln(P/c) )                 (1)
 # Multiplying P for the current temperature T with the relative humidity and
 # substituting this product in formula (1) results in the formula's used in
 # this method.
 #
  @property
  def dew_point( self ):
    temp= self.temperature
    humi= self.humidity
    a=  17.27                           # []
    b= 237.7                            # [C]
    gamma= math.log(humi) + a*temp/(b + temp)
    return b*gamma/(a - gamma)
