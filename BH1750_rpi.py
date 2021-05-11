#
# Class definitions for the ROHM BH1750 digital ambient light sensor.
#
# These class definitions only support the 'one time' measurement modes. As a
# result, the sensor is most of the time in the power-down mode, thus minimising
# it's power usage. Note that module smbus2 is used, as this module allows for
# access to I2C-devices which do not have explicitly addressable registers.
#
# These class definitions are specific to the Raspberry Pi, as it is assumed
# that the I2C channel number is one.
#
# Written by W.J.M. Nelis, wim.nelis@ziggo.nl, 2021.05
#
import smbus2 as smbus
import time

#
# Constant definitions for the i2c bus.
#
I2C_CHANNEL= 1                          # The i2c channel on an RPi

#
# BH1750 constants.
#
BH1750_DEFAULT_ADDRESS  = 0x23		# Address with ADDR pin low
BH1750_ALTERNATE_ADDRESS= 0x5c		# Address with ADDR pin high
BH1750_READ_LENGTH      =    2
#
BH1750_CMD_POWER_DOWN   = 0x00
BH1750_CMD_POWER_ON     = 0x01
BH1750_CMD_RESET        = 0x07
BH1750_CMD_ONE_TIME_MODE= 0x20		# Excluding the resolution mode
BH1750_CMD_SENS_HIGH_BIT= 0x40
BH1750_CMD_SENS_LOW_BIT = 0x60
# Define the resolution modes
BH1750_MEDIUM_RESOLUTION= 0x00
BH1750_HIGH_RESOLUTION  = 0x01
BH1750_LOW_RESOLUTION   = 0x02
# Define the sensor sensitivity range, that is the integration time in an
# unknown unit.
BH1750_MIN_MEASURE_TIME =   31
BH1750_DEF_MEASURE_TIME =   69
BH1750_MAX_MEASURE_TIME =  254
#
# Define the supported resolution modes and the maximum measurement time,
# expressed in [ms], for each.
#
BH1750_RESOLUTION_MODES= (
  ( BH1750_MEDIUM_RESOLUTION, 180 ),
  ( BH1750_HIGH_RESOLUTION  , 180 ),
  ( BH1750_LOW_RESOLUTION   ,  24 ) )

#
# Class BH1750 defines methods to read the light intensity and to set the
# measurement mode and the integration time. Within this class only method
# smbus2.i2c_rdwr() is used to perform I/O. Many i2c.+ methods use a register
# address, but the BH1750 does not want to receive a register address.
#
class BH1750():

 #
 # Method __init__ initialises the object instance. For the commands to start a
 # measurement and to read the result, the needed i2c message is created as an
 # instance variable.
 #
  def __init__( self, i2c=None, *, address=BH1750_DEFAULT_ADDRESS ):
    if i2c is None:
      self._i2c= smbus.SMBus( I2C_CHANNEL )
    else:
      self._i2c= i2c
    self._address= address		# Device address on I2C bus
    self._meascmd= smbus.i2c_msg.write( self._address, [0x00] )
    self._readcmd= smbus.i2c_msg.read ( address,  BH1750_READ_LENGTH )
    self._somecmd= smbus.i2c_msg.write( self._address, [0x00] )
    self._mode   = None			# Resolution mode
    self._timer  = BH1750_DEF_MEASURE_TIME
    self._timemax= None			# Max time at default sensitivity
    self._timeout= None			# Max time to complete measurement
    self._scale  = None			# Factor to map count onto [lx]

    self.resolution = BH1750_MEDIUM_RESOLUTION
    self.sensitivity= BH1750_DEF_MEASURE_TIME
    self.power_off()

 #
 # Private method _set_read_pars sets two parameters needed when reading a value
 # from the sensor: the maximal time it should take to read a value and the
 # scale factor to convert to digital reading into the lux count.
 #
  def _set_read_pars( self ):
    assert self._timer   is not None
    assert self._mode    is not None
    assert self._timemax is not None
    self._timeout= int( self._timer * self._timemax / BH1750_DEF_MEASURE_TIME )
    self._scale  = BH1750_DEF_MEASURE_TIME / (1.2 * self._timer)
    if self._mode == BH1750_HIGH_RESOLUTION:
      self._scale /= 2.0

 #
 # Property resolution retrieves or sets the resolution of the sensor. There are
 # three resolutions, which are 0.5 [lx], 1 [lx] or 4 [lx]. They are selected by
 # resolution modes 1, 0 and 2 respectively.
 #
  @property
  def resolution( self ):
    return self._mode

  @resolution.setter
  def resolution( self, val ):
    assert isinstance( val, int )
    newmode= None
    for mode,time in BH1750_RESOLUTION_MODES:
      if val == mode:
        newmode= mode
        self._meascmd.buf[0]= BH1750_CMD_ONE_TIME_MODE + mode
        self._mode   = mode
        self._timemax= time
        self._set_read_pars()
        break
    assert newmode is not None

 #
 # Property sensitivity retrieves or sets the sensitivity of the sensor, that is
 # the measurement time. The setter will write the timer register of the sensor.
 #
  @property
  def sensitivity( self ):
    return self._timer

  @sensitivity.setter
  def sensitivity( self, val ):
    assert isinstance( val, int )
    newtimer= max( min(val,BH1750_MAX_MEASURE_TIME),  BH1750_MIN_MEASURE_TIME )
    if self._timer is None  or  self._timer != newtimer:
      self._timer= newtimer
      self._set_read_pars()

      self.power_on()
      nibble= (self._timer >> 5) & 0x07	# Set higher 3 bits of timer
      self._somecmd.buf[0]= BH1750_CMD_SENS_HIGH_BIT + nibble
      self._i2c.i2c_rdwr( self._somecmd )
      nibble=  self._timer & 0x1f	# Set lower 5 bits of timer
      self._somecmd.buf[0]= BH1750_CMD_SENS_LOW_BIT + nibble
      self._i2c.i2c_rdwr( self._somecmd )
      time.sleep( self._timemax*0.001 )	# Waiting seems to be necessary
      self.power_off()

 #
 # Property range returns the range, that is both the lowest and the highest
 # light intensity that can be measured with the current settings of the
 # resolution and the sensitivity.
 #
  @property
  def range( self ):
    return ( 0, int(65535*self._scale) )

 #
 # Method power_off switches the sensor to the standby mode.
 #
  def power_off( self ):
    self._somecmd.buf[0]= BH1750_CMD_POWER_DOWN
    self._i2c.i2c_rdwr( self._somecmd )

 #
 # Method power_on switches the sensor to the operational mode.
 #
  def power_on( self ):
    self._somecmd.buf[0]= BH1750_CMD_POWER_ON
    self._i2c.i2c_rdwr( self._somecmd )

 #
 # Method reset clears the data register.
 #
  def reset( self ):
    self._somecmd.buf[0]= BH1750_CMD_RESET
    self._i2c.i2c_rdwr( self._somecmd )

 #
 # Method read returns the current light intensity. Both the resolution and the
 # sensitivity can be changed prior to starting the measurement. This method
 # returns a float value containing the light intensity expressed in [lx].
 # The time spent in this method is minimised by setting the data register in
 # the sensor to zero and check regularly for a non-zero value. The checks stop
 # once the maximum time needed for a measurement has elapsed.
 #
  def read( self, resol=None, sens=None ):
    if resol is not None:  self.resolution = resol
    if sens  is not None:  self.sensitivity= sens

    self.power_on()			# Switch on sensor
    self.reset()			# Reset data register to zero

    self._i2c.i2c_rdwr( self._meascmd )	# Send command to measure
    t0= time.time() + self._timeout	# End of measuring period
    while True:
      time.sleep( 0.01 )		# Wait for 10 [ms]
      self._i2c.i2c_rdwr( self._readcmd )
      t1= time.time()
      rv= int.from_bytes( list(self._readcmd), 'big' )
      if rv >  0:  break
      if t1 > t0:  break
    return rv*self._scale

