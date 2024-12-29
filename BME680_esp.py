#
# bme680f: A BME680 MicroPython driver, using floating point arithmetic to
# calculate the temperature, air pressure and relative humidity in SI units.
#
# The BME680 sensor is used as a replacement for a BME280 sensor, thus there is
# no need to measure the air quality. Those gas related measurements are
# switched off in this driver and there are no methods included to control or
# perform those measurements. The smaller footprint of this module is well
# appreciated when using it on an ESP8266 microcontroller.
#
# This driver is a rewrite of the the driver written by Salvatore Sanfilippo,
# published at URL https://github.com/antirez/bme680-pure-mp.
#
# Characteristics of this implementation are:
# - Only I2c communication is supported.
# - No air quality measurements. Note that using these gas measurements implies
#   that a surface is heated for some time, followed by a measurement of the
#   change in resistance. The heating has a severe impact on the temperature
#   sensor, which is located within 3 [mm] of the heated surface.
# - Skip a measurement if it's raw value is 0x8000.
# - Floating point arithmetic is used for the calculations of the calibrated
#   sensor readings. It is found that the FP formulas are slightly less complex
#   compared with those using integer arithmetic and require the same time to
#   execute on an ESP8266 microcontroller (which has no FPU). Both versions need
#   about 7 [ms] to do the calculations.
# - Assert is chosen for value checks, as those asserts can be effectively
#   removed by selecting the appropriate optimization.
#
# Note:
# The formulas for the calculation of the calibrated temperature are taken from
# the Bosch BMP680 data-sheet, section 3.3.1. A temperature value named t_fine
# is important as it is needed to calculate both the calibrated humidity and
# pressure too.
#
# It is found that the published formulas are effectively a second degree
# polynomial, that is variable t_fine = a*raw^2 + b*raw + c, with:
#   a = cp_t3/1073741824
#   b = cp_t2/16384 - cp_t1*cp_t3/33554432
#   c = cp_t1*(cp_t1*cp_t3/4194304 - cp_t2/1024)
# in which cp_tX are the calibration parameters for the temperature. The
# coefficients of the polynomial are calculated at object initialisation, thus
# only a simple one-line expression is needed to calculate t_fine after each
# measurement.
#
# Written by W.J.M. Nelis, wim.nelis@ziggo.nl, 2024.12.
#
# To do:
# - Extend method _check_chip to check for non-BME688, that is register 0xF0
#   should be 0x00.
#
import math				# Calculation of dew point temperature
import struct				# Unpack calibration data
import time				# For milliseconds sleep

#
# Installation constants.
# -----------------------
#
_BME680_DEVICE_ADDRESS = const(0x77)	# Default I2C device address

# Register addresses and sizes of register blocks
_BME680_REG_LOW        = const(0x00)	# Lowest register address in use
_BME680_REG_HIGH       = const(0xEF)	# Highest register address in use
#
_BME680_REG_MEAS_STATUS= const(0x1D)
_BME680_REG_RAW_DATA   = const(0x1F)	# Start of raw measurement data
_BME680_LNG_RAW_DATA   = const(   8)	# Length
_BME680_REG_RES_HEAT_0 = const(0x5A)
_BME680_REG_GAS_WAIT_0 = const(0x64)
_BME680_REG_CTRL_GAS_0 = const(0x70)
_BME680_REG_CTRL_GAS_1 = const(0x71)
_BME680_REG_CTRL_HUM   = const(0x72)
_BME680_REG_CTRL_MEAS  = const(0x74)
_BME680_REG_CONFIG     = const(0x75)
_BME680_REG_CALIB_RB0  = const(0x8A)	# Calibration parameters, block #0
_BME680_LNG_CALIB_RB0  = const(  23)
_BME680_REG_CHIP_ID    = const(0xD0)
_BME680_REG_SOFTRESET  = const(0xE0)
_BME680_REG_CALIB_RB1  = const(0xE1)	# Calibration parameters, block #1
_BME680_LNG_CALIB_RB1  = const(  10)

# Register value constants
_BME680_VAL_SOFTRESET  = const(0xB6)	# Value to start a reset
_BME680_VAL_CHIP_ID    = const(0x61)	# BME680 chip identifier
_BME680_VAL_FORCE_MODE = const(0x01)	# Force one measurement
# Allowed values and mapping to selector values
BME680_OSR= (0, 1, 2, 4, 8, 16)		# Allowed oversampling rate values
BME680_IIR= (0, 1, 3, 7, 15, 31, 63, 127) # Allowed exponential moving average values


#
# Define class BME680, which handles a Bosch BME680, which contains a pressure
# sensor, a temperature sensor and a humidity sensor. This class only supports
# I2C communications and disables the gas related measurements.
#
class BME680:
  def __init__( self, i2c, device=_BME680_DEVICE_ADDRESS ):
    self.i2c     = i2c
    self.i2c_da  = device
    self.rawtemp = None			# Last temperature measurement
    self.rawpres = None			# Last pressure measurement
    self.rawhumi = None			# Last humidity measurement
    self.t_fine  = None			# Compensation temperature value
    self.lifetime= 2000			# Life time of one measurement [ms]
    self.tom= time.ticks_diff(time.ticks_ms(), self.lifetime+1)	# Expired timer
  #
    assert device in i2c.scan(), 'BME680 sensor not found'
    self._check_chip()			# Make sure it's a BME680
    self.reset()
    self._load_calibration_parameters()
    self.set_mode()			# Set default operational mode
  #
    self.tc_a= self.cp_t3/1073741824	# t_fine polynomial coefficients
    self.tc_b= self.cp_t2/16384 - self.cp_t1*self.cp_t3/33554432
    self.tc_c= self.cp_t1*(self.cp_t1*self.cp_t3/4194304 - self.cp_t2/1024)

 #
 # Private method _read_register returns the content of one register, a byte
 # value.
 #
  def _read_register( self, register ):
    assert _BME680_REG_LOW <= register <= _BME680_REG_HIGH, \
      f'Register address out of range: {register:02x}'
    return self.i2c.readfrom_mem( self.i2c_da, register, 1 )[0]

 #
 # Private method _read_registers returns the content of a set of consecutive
 # registers, a tuple of bytes.
 #
  def _read_registers( self, register, count ):
    assert _BME680_REG_LOW <= register  and \
           register+count <= _BME680_REG_HIGH+1, \
      f'Register address out of range: {register:02x}'
    assert count > 0, \
      f'Transfer size out of range: {count}'
    return self.i2c.readfrom_mem( self.i2c_da, register, count )

 #
 # Private method _write_register writes one byte into one register.
 #
  def _write_register( self, register, value ):
    assert _BME680_REG_LOW <= register <= _BME680_REG_HIGH, \
      f'Register address out of range: {register:02x}'
    if isinstance( value, int ):
      value= value.to_bytes( 1, 'big' )
    return self.i2c.writeto_mem( self.i2c_da, register, value )

  def _write_smart( self, register,value ):
    curval= self._read_register( register )
    if curval != value:  return self._write_register( register, value )

 #
 # Private method _check_chip checks if the chip identifier matches the expected
 # value. Finding the chip identifier also implies that the chip and the
 # communication path are operational.
 #
  def _check_chip( self ):
    chip_id= self._read_register( _BME680_REG_CHIP_ID )
    assert chip_id == _BME680_VAL_CHIP_ID, \
      f'Failed to find BME680, chip identifier: {chip_id:04x}'

 #
 # Private method _load_calibration_parameters retrieves the calibration data
 # from the sensor, converts them to floating point numbers and saves them.
 #
  def _load_calibration_parameters(self):
    cp = self._read_registers(_BME680_REG_CALIB_RB0, _BME680_LNG_CALIB_RB0) + \
         self._read_registers(_BME680_REG_CALIB_RB1, _BME680_LNG_CALIB_RB1)
    cp = list( struct.unpack('<hbBHhbBhhbbHhhBBBBbbbBbH',bytes(cp)) )
    cp[15]= (cp[15] << 4) | (cp[16] >> 4)	# Convert BBB to HH
    cp[17]= (cp[17] << 4) | (cp[16] & 0x0f)
    cp= [ float(i)  for i in cp ]	# Convert to floating point format

    # Temperature calibration parameters.
    self.cp_t1 = cp[23]
    self.cp_t2 = cp[0]
    self.cp_t3 = cp[1]

    # Pressure calibration parameters.
    self.cp_p1 = cp[3]
    self.cp_p2 = cp[4]
    self.cp_p3 = cp[5]
    self.cp_p4 = cp[7]
    self.cp_p5 = cp[8]
    self.cp_p6 = cp[10]
    self.cp_p7 = cp[9]
    self.cp_p8 = cp[12]
    self.cp_p9 = cp[13]
    self.cp_p10= cp[14]

    # Humidity calibration parameters.
    self.cp_h1 = cp[17]
    self.cp_h2 = cp[15]
    self.cp_h3 = cp[18]
    self.cp_h4 = cp[19]
    self.cp_h5 = cp[20]
    self.cp_h6 = cp[21]
    self.cp_h7 = cp[22]

 #
 # Private method _map_iirc maps an IIR (EMA) coefficient onto its corresponding
 # selector value. If an unsupported coefficient is specified, an exception is
 # thrown.
 #
  def _map_iirc( self, aniirc ):
    assert aniirc in BME680_IIR, \
      f'Unsupported IIR filter coefficient: {aniirc}'
    return BME680_IIR.index( aniirc )

 #
 # Private method _map_osr maps an oversampling rate to its corresponding
 # oversampling rate selector. If an unsupported oversampling rate is specified,
 # an exception is generated.
 #
  def _map_osr( self, anosr ):
    assert anosr in BME680_OSR, \
      f'Unsupported oversampling rate: {anosr}'
    return BME680_OSR.index( anosr )

 #
 # Private method _read_raw retrieves the current (raw) measurements of the
 # pressure, the temperature and the humidity. The mode of operation is forced
 # mode, thus the sensor must be triggered to perform a measurement. At most one
 # measurement per lifetime [ms] is performed.
 #
  def _read_raw( self ):
    now= time.ticks_ms()
    if time.ticks_diff(now, self.tom) > self.lifetime:
  #
  # Select force mode and wait for the measurement to be complete. Note that the
  # (other) control registers MUST be set prior to invoking this method! On a
  # BME280 sensor, the time typically needed is 2 + 2*(osr_t + osr_p + osr_h)
  # [ms], but in the data-sheet of the BME680 the time needed for one
  # measurement is not specified. At the highest oversampling rate the formula
  # above seems to be valid for a BME680 as well. Check every 5 [ms] to see if
  # the measurement is completed.
  #
      register = self._read_register( _BME680_REG_CTRL_MEAS ) & 0xFC
      register|= _BME680_VAL_FORCE_MODE
      self._write_register( _BME680_REG_CTRL_MEAS, register )
      wdt= time.ticks_add( time.ticks_ms(), 3000 )	# Watchdog timer
      while time.ticks_diff( wdt, time.ticks_ms() ) > 0:
        time.sleep_ms( 5 )
        if self._read_register( _BME680_REG_MEAS_STATUS ) & 0x80:
          break
      else:
        raise ValueError( 'Timed out waiting for measurement from BME680' )
  #
  # Retrieve the raw measurements and save them.
  #
      d= self._read_registers( _BME680_REG_RAW_DATA, _BME680_LNG_RAW_DATA )
      self.rawpres= (d[0]<<12) + (d[1]<<4) + (d[2]>>4)
      self.rawtemp= (d[3]<<12) + (d[4]<<4) + (d[5]>>4)
      self.rawhumi= (d[6]<< 8) +  d[7]
  #
  # Compute a derivate of the raw temperature, called t_fine, which is needed in
  # the computation of the calibrated temperature, relative humidity and air
  # pressure.
  #
      self.t_fine= (self.tc_a*self.rawtemp + self.tc_b)*self.rawtemp + self.tc_c
      self.tom   = now

 #
 # Method dew_point calculates the dew point temperature expressed in [C], given
 # the temperature and the relative humidity. The formula and the constants are
 # taken from URL https://en.wikipedia.org/wiki/Vapour_pressure_of_water.
 #
 # The computation is using the Tetens approximation formula between temperature
 # T and the maximal vapor pressure P twice. The formula is:
 #  P= c*exp( a*T/(b+T) )                        (0)
 # The inverse of this formula is:
 #  T= b*ln(P/c)/( a - ln(P/c) )                 (1)
 # Multiplying P/c for the current temperature T with the relative humidity and
 # substituting this product in formula (1) results in the formula's used in
 # this method.
 #
  def get_dew_point( self ):
    temp= self.get_temperature()
    humi= self.get_humidity()
    if temp is None  or  humi is None:  return None
  #
    a=  17.27				# []
    b= 237.7				# [C]
    gamma= math.log(humi) + a*temp/(b + temp)
    return b*gamma/(a - gamma)

  @property
  def dew_point( self ):
    return self.get_dew_point()

 #
 # Method humidity returns the calibrated humidity expressed as a fraction, thus
 # a value in the range [0,1]. The formulas are taken from the Bosch BME680
 # data-sheet, section 3.3.3, which match the floating point formulas in the
 # reference API.
 #
  def get_humidity( self ):
    self._read_raw()
    if self.rawhumi == 0x8000:  return None	# Exit if not measured
  #
    temp= self.t_fine/5120.0		# Calibrated temperature
    var1= self.rawhumi - (self.cp_h1*16.0 + self.cp_h3*temp/2.0)
    var2= var1*( (self.cp_h2/262144.0) * (1.0 + self.cp_h4*temp/16384.0 +
          self.cp_h5*temp*temp/1048576.0) );
    var3= self.cp_h6/  16384.0
    var4= self.cp_h7/2097152.0
    humi= (var2 + (var3 + var4*temp)*var2*var2)/100.0
    humi= min( max(humi,0.0), 1.0 )
    return humi

  @property
  def humidity( self ):
    return self.get_humidity()

 #
 # Method pressure returns the calibrated air pressure expressed in [Pa]. The
 # formulas are taken from the Bosch BMP680 data-sheet, section 3.3.2, which
 # match the formulas used in the reference API.
 # This method will return None if the pressure is not measured (oversampling
 # rate is zero) or to avoid dividing by (almost) zero.
 #
  def get_pressure( self ):
    self._read_raw()
    if self.rawpres == 0x8000:  return None	# Exit if not measured
  #
    var1= self.t_fine/2.0 - 64000.0
    var2= var1*var1*self.cp_p6/131072.0
    var2= var2 + var1*self.cp_p5*2.0
    var2= var2/4.0 + self.cp_p4*65536.0
    var1= (self.cp_p3*var1*var1/16384.0 + self.cp_p2*var1)/524288.0
    var1= (1.0 + var1/32768.0)*self.cp_p1
    if int(var1) == 0:
      return None			# Do not divide by (almost) zero
    else:
      pres= 1048576.0 - self.rawpres
      pres= (pres - var2/4096.0)*6250.0/var1
      var1= self.cp_p9*pres*pres/2147483648.0;
      var2= pres*self.cp_p8/32768.0
      var3= pres/256.0
      var3= var3*var3*var3*self.cp_p10/131072.0
      pres= pres + (var1 + var2 + var3 + self.cp_p7*128.0)/16.0
      return pres			# Finally, the air pressure in [Pa]

  @property
  def pressure( self ):
    return self.get_pressure()

 #
 # Method reset will perform a soft reset. According to the reference API, the
 # reset takes up to 5 [ms].
 #
  def reset( self ):
    self._write_register( _BME680_REG_SOFTRESET, b'\xB6' )
    time.sleep_ms( 5 )

 #
 # Method set_mode is used to set the operational parameters for the
 # measurements.
 #
 # Parameters osrh, osrp and osrt control the oversampling of humidity, pressure
 # and temperature outputs. More oversampling means more delay and more
 # precision. Special value zero causes this measurement to be skipped.
 # Parameter iirfc controls the working of the EMA (low pass) filter, named IIR.
 # Special value zero disables this filter.
 #
  def set_mode( self, osrh=16, osrp=16, osrt=16, iirfc=0,
                gas_run=False, gas_temp=320, gas_ms=150 ):

  # Handle the oversampling rates. Map the rates onto selector values and save
  # those values in the registers of the BME680, if the value has changed.
    osrsh= self._map_osr( osrh )
    osrsp= self._map_osr( osrp )
    osrst= self._map_osr( osrt )
    assert osrst > 0, "Don't disable temperature measurement"
    regval= (osrst << 5) + (osrsp << 2)		# Set mode to sleep
    self._write_smart( _BME680_REG_CTRL_MEAS, regval )
    self._write_smart( _BME680_REG_CTRL_HUM , osrsh  )

  # Handle the IIR filter, better known as the exponential moving average (EMA).
  # Note that writing the associated register will clear the moving average.
    iirfs= self._map_iirc( iirfc )
    self._write_smart( _BME680_REG_CONFIG, iirfs << 2 )

  # Disable the measurement of the gas(ses).
    assert not gas_run, 'Gas measurement is not supported'
    regval= self._read_register( _BME680_REG_CTRL_GAS_1 )
    if regval & 0b00010000:
      regval&= 0b11101111		# Disable gas test
      self._write_register( _BME680_REG_CTRL_GAS_1, regval )

 #
 # Method temperature returns the calibrated temperature expressed in [C].
 #
  def get_temperature( self ):
    self._read_raw()
    return self.t_fine/5120.0

  @property
  def temperature( self ):
    return self.get_temperature()
