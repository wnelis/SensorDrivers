#
# Class definition for the Sensirion temperature / humidity sensor SHT31.
#
# This definition is based on the Adafruit_SHT.py driver, written by Tony
# DiCola, Adafruit Industries. It uses the smbus2 driver for the i2c-layer.
# This driver is specific for the Raspberry Pi. It only supports the "Single
# Shot Data Acquisition Mode".
#
# Written by W.J.M. Nelis, wim.nelis@ziggo.nl, 2018.07
#
import smbus2 as smbus
import time

#
# Constant definitions for the i2c bus.
#
I2C_CHANNEL= 1				# The i2c channel on an RPi

#
# Constant definitions for sensor SHT31.
# Define the command codes.
#
SHT31_MEAS_HIGHREP_STRETCH= 0x2C06
SHT31_MEAS_MEDREP_STRETCH = 0x2C0D
SHT31_MEAS_LOWREP_STRETCH = 0x2C10
SHT31_MEAS_HIGHREP        = 0x2400
SHT31_MEAS_MEDREP         = 0x240B
SHT31_MEAS_LOWREP         = 0x2416
SHT31_MEAS_LNG            =      6
SHT31_READSTATUS          = 0xF32D
SHT31_READSTATUS_LNG      =      3
SHT31_CLEARSTATUS         = 0x3041
SHT31_SOFTRESET           = 0x30A2
SHT31_HEATER_ON           = 0x306D
SHT31_HEATER_OFF          = 0x3066
#
# Define the flags in the (16-bit) status register.
#
SHT31_STATUS_DATA_CRC_ERROR   = 0x0001
SHT31_STATUS_COMMAND_ERROR    = 0x0002
SHT31_STATUS_RESET_DETECTED   = 0x0010
SHT31_STATUS_TEMPERATURE_ALERT= 0x0400
SHT31_STATUS_HUMIDITY_ALERT   = 0x0800
SHT31_STATUS_HEATER_ACTIVE    = 0x2000
SHT31_STATUS_ALERT_PENDING    = 0x8000

#
# Define class SHT31, which handles a Senserion SHT31-DIS temperature and
# humidity sensor.
#
class SHT31( smbus.SMBus ):
  def __init__( self, device ):
    super().__init__( I2C_CHANNEL )	# Parent initialisation

  # Preset instance variables.
    self.device  = device		# i2c slave device address
    self.rawtemp = None			# Last temperature measurement
    self.rawrhum = None			# Last relative humidity measurement
    self.livetime= 2			# Live time of one measurement [s]
    self.tolm    = 0			# Time of last measurement
    self.crcerr  = 0			# Total number of CRC-errors

  #
  # Private method _calc_crc8 computes the 8-bit CRC of a given set of bytes.
  # The algorithm is taken from module Adafruit_SHT31.py. Note that the CRC of
  # a list of bytes followed by its CRC is by definition zero.
  #
  def _calc_crc8( self, bfr ):
    pol= 0x31				# CRC polynomial
    crc= 0xff				# CRC preset
    for i in range( 0, len(bfr) ):
      crc^= bfr[i]
      for j in range( 8, 0, -1 ):
        if crc & 0x80:
          crc= (crc << 1) ^ pol
        else:
          crc= (crc << 1)
    return crc & 0xff

  #
  # Private method _read_raw retrieves the current (raw) measurements of the
  # relative humidity and the temperature. The single shot data acquisition mode
  # is used, thus the sensor must be triggered to perform a measurement. At most
  # one measurement per livetime seconds is performed.
  #
  def _read_raw( self ):
    now= time.time()
    if self.tolm < now-self.livetime:
   #
   # Select force mode and wait for the measurement to be complete. The time
   # needed for one measurement in high repeatability mode is at most 15 [ms].
   #
      self.rawtemp= None		# Flag: no measurement yet
      self._write_command( SHT31_MEAS_HIGHREP )	# Start measurement
      time.sleep( 0.015 )
      for n in range(3):
   #
   # Method read_i2c_block_data will write a single byte, the register address,
   # prior to start reading. It is found the aforementioned write operation is
   # ignored by the SHT31 sensor.
   #
        d= self.read_i2c_block_data( self.device, 0x00, SHT31_MEAS_LNG )
        if self._calc_crc8( d[0:3] ) == 0  and  self._calc_crc8( d[3:6] ) == 0:
          self.rawtemp= (d[0]<<8) + d[1]
          self.rawrhum= (d[3]<<8) + d[4]
          self.tolm   = now
          break
        else:
          self.rawtemp= None
          self.rawrhum= None
          self.crcerr+= 1
          time.sleep( 0.005 )

  #
  # Private method _write_command writes a 16-bit command to the sensor. Method
  # SMBus.write_byte_data, which writes an 8-bit value to an 8-bit register in
  # the sensor, is misused to write a single 16-bit value.
  #
  def _write_command( self, cmd):
    cmd_msb= cmd >> 8			# Pseudo register value
    cmd_lsb= cmd &  0xFF		# Pseudo byte register content
    self.write_byte_data( self.device, cmd_msb, cmd_lsb )

  def clear_status( self ):
    self._write_command( SHT31_CLEARSTATUS )

  #
  # Method read_dew_point calculates the dew point temperature, expressed in
  # degrees Celsius, given the temperature and the relative humidity. The
  # formula and the constants are taken from URL
  # https://nl.wikipedia.org/wiki/Dauwpunt. See also URL
  # https://en.wikipedia.org/wiki/Vapour_pressure_of_water
  #
  # The computation is using the Tetens approximation formula between
  # temperature T and the maximal vapor pressure P twice. The formula is:
  #  P= c*exp( a*T/(b+T) )			(0)
  # The inverse of this formula is:
  #  T= b*ln(P/c)/( a - ln(P/c) )		(1)
  # Multiplying P for the current temperature T with the relative humidity and
  # substituting this product in formula (1) results in the formula's used in
  # this method.
  #
  def read_dew_point():
    self._read_raw()
    if self.rawrhum == None  or  self.rawtemp == None:
      return float( 'nan' )
    else:
      a=  17.27				# []
      b= 237.7				# [C]
      humi= self.rawrhum/65535.0
      temp= 175*self.rawtemp/65535.0 - 45
      gamma= math.log(humi) + a*temp/(b + temp)
      return b*gamma/(a - gamma)

  #
  # Method read_errocount returns the accumulated error count, and resets the
  # internal error counter.
  #
  def read_errorcount( self ):
    errcnt= self.crcerr
    self.crcerr= 0
    return errcnt

  #
  # Method read_humidity returns the current relative humidity, a floating point
  # number in the range 0 .. 100. If reading of the value fails, it returns the
  # value NaN.
  #
  def read_humidity( self ):
    self._read_raw()
    if self.rawrhum == None:
      return float( 'nan' )
    else:
      return 100*self.rawrhum/65535.0

  #
  # Method read_status returns the current status of the sensor. If reading of
  # the status fails, it returns None.
  #
  def read_status( self ):
    self._write_command( SHT31_READSTATUS )
    sts= self.read_i2c_block_data( self.device, 0x00, SHT31_READSTATUS_LNG )
    if self._calc_crc8( sts[0:3] ) == 0:
      return (sts[0]<<8) + sts[1]
    else:
      self.crcerr+= 1
      return None

  #
  # Method read_temperature returns the current temperature, expressed in
  # degrees Celsius. If reading of the temperature fails, it returns value NaN.
  #
  def read_temperature( self ):
    self._read_raw()
    if self.rawtemp == None:
      return float( 'nan' )
    else:
      return 175*self.rawtemp/65535.0 - 45

  def reset( self ):
    self._write_command( SHT31_SOFTRESET )
    time.sleep( 0.002 ) 			# Wait the required time

  def set_heater( self, HtrOn= True ):
    if HtrOn:
      self._write_command( SHT31_HEATER_ON  )
    else:
      self._write_command( SHT31_HEATER_OFF )
