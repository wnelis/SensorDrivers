#
# Module scd4x defines class SCD4X, which can be used to interface with a
# Sensirion SCD40 or SCD41 sensor.
#
# Class SCD4X is adapted for asyncio, as the waiting time imposed by the sensor
# might be large. A number of methods are therefore coroutines. The delay to
# wait for the completion of a command can be realised in either a synchronous
# way (time.sleep(), if the delay is short) or in an asynchronous way
# (asyncio.sleep(), if the delay is long). The context switching time on an
# ESP8266 is in the order of 1 [ms]. Thus the performance might be (slightly)
# better if delays shorter than a few milliseconds are realised in the
# synchronous way.
#
# This module started out as a copy op the Adafruit SCD4X class, see URL
# https://github.com/adafruit/Adafruit_CircuitPython_SCD4X, but it has been
# rewritten for the major part.
#
# Rewritten by Wim Nelis, 2024.12
#
# Notes:
#  . A value passed to the sensor is always a 16-bit unsigned integer. Thus only
#    a non-negative value can be passed. This implies that an altitude or a
#    temperature offset, in general the value in a set_.+ method invocation, is
#    zero or greater.
#  . The value returned by command _SCD4X_READMEASUREMENT seems to contain an
#    undocumented flag. If the MSB is set, a periodic measurement is in
#    progress. However, if the MSB is not set, the sensor is in idle mode.
#  . The SCD4x command list as described in the data-sheet contains command
#    get_ambient_pressure, chapter 3.7.6. This command does not seem to exist.
#
import asyncio				# Multi tasking
import struct				# (Un)pack data (from) to sensor
import time				# Synchronous wait
from   machine     import I2C		# Access to sensor
from   micropython import const, opt_level

opt_level(0)				# No optimisations
#
# Symbol _SCD4X_ALL_METHODS controls the size of the compiled module. If set to
# 1, all methods will be included. However, if set to 0, those methods within an
# "if _SCD4X_ALL_METHODS:" won't be compiled and won't be available. This will
# make the compiled size of the module smaller.
#
_SCD4X_ALL_METHODS = const(1)		# Go for a large module
# In a similar fashion does symbol _SCD4X_DEBUG control the generation of
# additional print statements, which might be used for debugging purposes.
_SCD4X_DEBUG       = const(0)		# Generate debug output

SCD4X_DEFAULT_ADDR = 0x62
_SCD4X_GETVARIANT                      = const(0x202F)
_SCD4X_MEASURESINGLESHOTRHT            = const(0x2196)
_SCD4X_MEASURESINGLESHOT               = const(0x219D)
_SCD4X_STARTLOWPOWERPERIODICMEASUREMENT= const(0x21AC)
_SCD4X_STARTPERIODICMEASUREMENT        = const(0x21B1)
_SCD4X_GETASCE                         = const(0x2313)
_SCD4X_GETTEMPOFFSET                   = const(0x2318)
_SCD4X_GETALTITUDE                     = const(0x2322)
_SCD4X_GETASCT                         = const(0x233F)
_SCD4X_GETASCINITIALPERIOD             = const(0x2340)
_SCD4X_GETASCSTANDARDPERIOD            = const(0x234B)
_SCD4X_SETASCE                         = const(0x2416)
_SCD4X_SETTEMPOFFSET                   = const(0x241D)
_SCD4X_SETALTITUDE                     = const(0x2427)
_SCD4X_SETASCT                         = const(0x243A)
_SCD4X_SETASCINITIALPERIOD             = const(0x2445)
_SCD4X_SETASCSTANDARDPERIOD            = const(0x244E)
_SCD4X_PERSISTSETTINGS                 = const(0x3615)
_SCD4X_FORCEDRECAL                     = const(0x362F)
_SCD4X_FACTORYRESET                    = const(0x3632)
_SCD4X_SELFTEST                        = const(0x3639)
_SCD4X_REINIT                          = const(0x3646)
_SCD4X_SERIALNUMBER                    = const(0x3682)
_SCD4X_POWERDOWN                       = const(0x36E0)
_SCD4X_WAKEUP                          = const(0x36F6)
_SCD4X_STOPPERIODICMEASUREMENT         = const(0x3F86)
_SCD4X_SETPRESSURE                     = const(0xE000)
_SCD4X_DATAREADY                       = const(0xE4B8)
_SCD4X_READMEASUREMENT                 = const(0xEC05)


class SCD4X:
 #
 # Class variable crcn contains the CRC's, with polynomial 0x31 and start value
 # 0x00, computed for each possible value of a nibble. By using this table, the
 # computation of a CRC becomes about 2.5 times faster on an ESP8266.
 #
  crcn= (0x00, 0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97,
         0xb9, 0x88, 0xdb, 0xea, 0x7d, 0x4c, 0x1f, 0x2e )

  def __init__(self, i2c_bus:I2C, address:int=SCD4X_DEFAULT_ADDR) -> None:
    self.i2c_dev = i2c_bus		# I2c access
    self.address = address		# Device address on i2c bus
    self.is_scd41= False		# Sensor model indication
    self.is_idle = True 		# Sensor state
    # Cached readings
    self._temperature = None
    self._humidity    = None
    self._co2         = None
    # Conversion constant and buffers
    self._cct0  = 175 / 65535		# Temperature conversion constant
    self._buffer= bytearray(9)		# Buffer for i2c I/O
    self._bview = memoryview(self._buffer)	# Allow efficient slicing
    self._cmd   = bytearray(2)		# Buffer for a single command

  # Stop any periodic measurement and clear the history in a synchronous way,
  # avoiding this routine to become a coroutine as well.
    self._send_command(_SCD4X_STOPPERIODICMEASUREMENT)	# Just to be sure
#   self.is_idle = True
    time.sleep_ms(500)
    self._send_command(_SCD4X_REINIT)	# Clear history
    time.sleep_ms( 50)
    self.is_scd41 = self.get_sensor_variant() == 1

 #
 # Define the low level hardware access methods.
 # ---------------------------------------------
 #

 #
 # Private method _read_reply reads the expected amount of octets from the
 # sensor into the instance buffer.
 # This method implements the second (reading) half of both the "read sequence"
 # and the "send command and fetch result", as defined in the data-sheet,
 # chapter 3.4. Either method _send_command() or method _set_value() is used
 # prior to invoking this method.
 #
  def _read_reply(self, count:int) -> None:
    assert 0 < count <= len(self._buffer), f'Request exceeds buffer size: {count}'
    assert count % 3 == 0, f'Unsupported request size: {count}'
    try:
      self.i2c_dev.readfrom_into(self.address, self._bview[:count])
    except OSError:
      raise RuntimeError('I2C data read error')
  # Check the CRC of each 16-bit value received.
    for i in range(0, count, 3):
      if self._crc8(self._bview[i:i+2]) != self._buffer[i+2]:
        raise RuntimeError('CRC check failed while reading data')

 #
 # Private method _send_command sends a command to the sensor. This method
 # implements (a part of) the "send command sequence" as described in the
 # data-sheet, chapter 3.4.
 #
  def _send_command(self, cmd:int) -> None:
    if _SCD4X_DEBUG:  print( f'SenC {cmd:04x}' )
    self._cmd= struct.pack('>H', cmd)
    try:
      self.i2c_dev.writeto(self.address, self._cmd)
    except OSError:
      raise RuntimeError('Could not communicate via I2C')

 #
 # Private method _set_value sends a command together with a single 16-bit value
 # to the sensor to set a specific parameter in the sensor.The value is
 # protected by a CRC. This method implements (a part of) the "write sequence"
 # as defined in the data-sheet, chapter 3.4.
 #
  def _set_value(self, cmd:int, value:int) -> None:
    assert 0 <= value <= 65535, f'Illegal value {int}'
    if _SCD4X_DEBUG:  print( f'SetV {cmd:04x} {value:04x}' )
    self._buffer[:4]= struct.pack('>HH', cmd, value)
    self._buffer[ 4]= self._crc8(self._bview[2:4])
    self.i2c_dev.writeto(self.address, self._bview[:5])

  if _SCD4X_DEBUG:
    def _PriB(self, fie:str) -> None:
      print(f'{fie} {self._buffer[0]:02x}{self._buffer[1]:02x}')

 #
 # Define the methods implementing the commands supported by the sensor.
 # ---------------------------------------------------------------------
 #
 # The methods are defined in the order in which the commands are described in
 # the data-sheet.
 #
 # A - Basic commands
 #
  def start_periodic_measurement(self) -> None:
    """Put sensor into working mode, about 5 [s] per measurement."""
    assert self.is_idle, 'Sensor is not in idle mode'
    self._send_command(_SCD4X_STARTPERIODICMEASUREMENT)
    self.is_idle = False

  def _read_data(self) -> None:
    """Reads the temp/hum/co2 from the sensor and caches it"""
    self._send_command(_SCD4X_READMEASUREMENT)
    time.sleep_ms(1)
    self._read_reply(9)
    raw = struct.unpack_from('>HxHxH', self._buffer)
    self._co2         = raw[0]
    self._temperature = -45 + self._cct0*raw[1]
    self._humidity    = raw[2] / 65535

  async def stop_periodic_measurement(self) -> None:
    """Stop periodic measurement mode."""
    self._send_command(_SCD4X_STOPPERIODICMEASUREMENT)
    await asyncio.sleep_ms(500)
    self.is_idle = True

#
# B - On-chip output signal compensation
#
  def set_temperature_offset(self, offset: Union[int, float]) -> None:
    """Set the temperature offset."""
    assert self.is_idle, 'Sensor is not in idle mode'
    assert 0 <= offset < 175, 'Temperature offset out of range'
#   raw = int( min(max(offset,0),20) / self._cct0 )
    raw = int( offset / self._cct0)
    self._set_value(_SCD4X_SETTEMPOFFSET, raw)
    time.sleep_ms(1)

  def get_temperature_offset(self) -> float:
    """Retrieve the current temperature offset. It is needed to calculate an
       updated value of the temperature offset."""
    assert self.is_idle, 'Sensor is not in idle mode'
    self._send_command(_SCD4X_GETTEMPOFFSET)
    time.sleep_ms(1)
    self._read_reply(3)
    if _SCD4X_DEBUG:  self._PriB('GetTO')
    raw = (self._buffer[0] << 8) | self._buffer[1]
    return raw * self._cct0

  if _SCD4X_ALL_METHODS:
    def set_altitude(self, height: int) -> None:
      """Set an offset for the air pressure by specifying the altitude."""
      assert self.is_idle, 'Sensor is not in idle mode'
      assert 0 <= height < 65535, 'Height out of range'
      self._set_value(_SCD4X_SETALTITUDE, height)
      time.sleep_ms(1)

    def get_altitude(self) -> int:
      """Retrieve the offset for the air pressure, expressed as an altitude."""
      assert self.is_idle, 'Sensor is not in idle mode'
      self._send_command(_SCD4X_GETALTITUDE)
      time.sleep_ms(1)
      self._read_reply(3)
      if _SCD4X_DEBUG:  self._PriB('GetA')
      return (self._buffer[0] << 8) | self._buffer[1]

  def set_ambient_pressure(self, pressure: int) -> None:
    """Set the ambient pressure to adjust the CO2 calculations."""
    assert 700 <= pressure <= 1200, 'Ambient pressure out of range'
    self._set_value(_SCD4X_SETPRESSURE, pressure)
    time.sleep_ms(1)

#
# C - Field calibration
#
  async def force_calibration(self, target_co2: int) -> int:
    """Perform a forced recalibration of the sensor."""
    assert self.is_idle, 'Sensor is not in idle mode'
    self._set_value(_SCD4X_FORCEDRECAL, target_co2)
    await asyncio.sleep_ms(400)
    self._read_reply(3)
    raw = (self._buffer[0] << 8) | self._buffer[1]
    if _SCD4X_DEBUG:  self._PriB('ForC')
    if raw == 0xFFFF:
      raise RuntimeError("Forced recalibration failed.")
    return raw-0x8000

  if _SCD4X_ALL_METHODS:
    def set_asc_enabled(self, enable: bool) -> None:
      """Enable or disable the automatic self calibration (ASC)."""
      assert self.is_idle, 'Sensor is not in idle mode'
      self._set_value(_SCD4X_SETASCE, int(enable))
      time.sleep_ms(1)

    def get_asc_enabled(self) -> bool:
      """Retrieve the state of the automatic self calibration (ASC), enabled or
         disabled."""
      assert self.is_idle, 'Sensor is not in idle mode'
      self._send_command(_SCD4X_GETASCE)
      time.sleep_ms(1)
      self._read_reply(3)
      if _SCD4X_DEBUG:  self._PriB('GetASCE')
      return self._buffer[1] == 1

    def set_asc_target(self, target: int) -> None:
      """Set the value of the automatic self calibration (ASC) baseline target."""
      assert self.is_idle, 'Sensor is not in idle mode'
      self._set_value(_SCD4X_SETASCT, target)
      time.sleep_ms(1)

    def get_asc_target(self) -> int:
      """Retrieve the baseline target of the automatic self calibration (ASC)."""
      assert self.is_idle, 'Sensor is not in idle mode'
      self._send_command(_SCD4X_GETASCT)
      time.sleep_ms(1)
      self._read_reply(3)
      if _SCD4X_DEBUG:  self._PriB('GetASCT')
      return (self._buffer[0] << 8) | self._buffer[1]

#
# D - Low power periodic measurement mode
#
  def start_low_periodic_measurement(self) -> None:
    """Put sensor into a low power working mode, about 30s per measurement."""
    assert self.is_idle, 'Sensor is not in idle mode'
    self._send_command(_SCD4X_STARTLOWPOWERPERIODICMEASUREMENT)
    self.is_idle = False

  def get_data_ready(self) -> bool:
    """Check the sensor to see if new data is available."""
    self._send_command(_SCD4X_DATAREADY)
    time.sleep_ms(1)
    self._read_reply(3)
    if _SCD4X_DEBUG:  self._PriB('GetDR')
    return not ((self._buffer[0] & 0x07 == 0) and (self._buffer[1] == 0))

  def is_idle(self) -> bool:
    """Check the sensor to see if is in the idle mode (True) or is performing
       a periodic measurement (False)."""
    self._send_command(_SCD4X_DATAREADY)
    time.sleep_ms(1)
    self._read_reply(3)
    if _SCD4X_DEBUG:  self._PriB('IsI')
    return (self._buffer[0] & 0x80 == 0x00)

#
# E - Advanced features
#
  if _SCD4X_ALL_METHODS:
    async def persist_settings(self) -> None:
      """Save various configuration settings to EEPROM."""
      assert self.is_idle, 'Sensor is not in idle mode'
      self._send_command(_SCD4X_PERSISTSETTINGS)
      await asyncio.sleep_ms(800)

  def get_serial_number(self) -> int:
    """Request the unique serial number of this sensor."""
    assert self.is_idle, 'Sensor is not in idle mode'
    self._send_command(_SCD4X_SERIALNUMBER)
    time.sleep_ms(1)
    self._read_reply(9)
    sn= struct.unpack_from('>HxHxH', self._buffer)
    return sn[0]<<32 | sn[1]<<16 | sn[2]

  async def perform_self_test(self) -> None:
    """Perform a self test."""
    assert self.is_idle, 'Sensor is not in idle mode'
    self._send_command(_SCD4X_SELFTEST)
    await asyncio.sleep_ms(10000)
    self._read_reply(3)
    if _SCD4X_DEBUG:  self._PriB('PerST')
    return (self._buffer[0] | self._buffer[1]) != 0

  async def reset(self) -> None:
    """Resets all configuration settings stored in the EEPROM and erases the FRC
    and ASC algorithm history."""
    assert self.is_idle, 'Sensor is not in idle mode'
    self._send_command(_SCD4X_FACTORYRESET)
    await asyncio.sleep_ms(1200)

 #
 # It was found that with a waiting time of 30 [ms], the returned value of the
 # immediately proceeding get_sensor_variant command might be utterly wrong,
 # typically 0xbf87. However with a waiting time of 50 [ms] this behaviour is
 # not observed.
 #
  async def reinit(self) -> None:
    """Reinitializes the sensor by reloading user settings from EEPROM."""
    assert self.is_idle, 'Sensor is not in idle mode'
    self._send_command(_SCD4X_REINIT)
    await asyncio.sleep_ms(50)

  def get_sensor_variant(self):
    """Retrieve the variant number, 0 for SCD40, 1 for SCD41."""
    assert self.is_idle, 'Sensor is not in idle mode'
    self._send_command(_SCD4X_GETVARIANT)
    time.sleep_ms(1)
    self._read_reply(3)
    if _SCD4X_DEBUG:  self._PriB('GetSV')
    return self._buffer[0] >> 4

#
# F - Single shot measurement mode (SCD41 only)
#
  async def measure_once_all(self) -> None:
    """Measure once CO2, humidity and temperature."""
    assert self.is_idle, 'Sensor is not in idle mode'
    assert self.is_scd41, 'Unsupported command for SCD40'
    self._send_command(_SCD4X_MEASURESINGLESHOT)
    await asyncio.sleep_ms(5000)

  async def measure_once_ht(self) -> None:
    """Measure once humidity and temperature."""
    assert self.is_idle, 'Sensor is not in idle mode'
    assert self.is_scd41, 'Unsupported command for SCD40'
    self._send_command(_SCD4X_MEASURESINGLESHOTRHT)
    await syncio.sleep_ms(50)

  def power_down(self) -> None:
    """Put the sensor from idle mode into sleep mode."""
    assert self.is_idle, 'Sensor is not in idle mode'
    assert self.is_scd41, 'Unsupported command for SCD40'
    self._send_command(_SCD4X_POWERDOWN)
    time.sleep_ms(1)

  async def wake_up(self) -> None:
    """Wake up the sensor from sleep mode into idle mode."""
    assert self.is_idle, 'Sensor is not in idle mode'
    assert self.is_scd41, 'Unsupported command for SCD40'
    self._send_command(_SCD4X_POWERDOWN)
    await asyncio.sleep_ms(30)

  if _SCD4X_ALL_METHODS:
    def set_asc_initial_period(self, period: int) -> None:
      """Set the duration of the initial period for automatic self calibration
         (ASC)."""
      assert self.is_idle, 'Sensor is not in idle mode'
      assert self.is_scd41, 'Unsupported command for SCD40'
      assert period % 4 == 0, 'Illegal initial ASC period'
      self._set_value(_SCD4X_SETASCINITIALPERIOD, period)
      time.sleep_ms(1)

    def get_asc_initial_period(self) -> int:
      """Get the current duration of the initial period for automatic self
         calibration (ASC)."""
      assert self.is_idle, 'Sensor is not in idle mode'
      assert self.is_scd41, 'Unsupported command for SCD40'
      self._send_command(_SCD4X_GETASCINITIALPERIOD)
      time.sleep_ms(1)
      self._read_reply(3)
      if _SCD4X_DEBUG:  self._PriB('GetASCIP')
      return (self._buffer[0] << 8) | self._buffer[1]

    def set_asc_standard_period(self, period: int) -> None:
      """Set the duration of the standard period for automatic self calibration
         (ASC)."""
      assert self.is_idle, 'Sensor is not in idle mode'
      assert self.is_scd41, 'Unsupported command for SCD40'
      assert period % 4 == 0, 'Illegal stan dard ASC period'
      self._set_value(_SCD4X_SETASCSTANDARDPERIOD, period)
      time.sleep_ms(1)

    def get_asc_standard_period(self) -> int:
      """Get the current duration of the standard period for automatic self
         calibration (ASC)."""
      assert self.is_idle, 'Sensor is not in idle mode'
      assert self.is_scd41, 'Unsupported command for SCD40'
      self._send_command(_SCD4X_GETASCSTANDARDPERIOD)
      time.sleep_ms(1)
      self._read_reply(3)
      if _SCD4X_DEBUG:  self._PriB('GetASCSP')
      return (self._buffer[0] << 8) | self._buffer[1]

#
# Private static method _crc8 calculates the CRC of the octets in the supplied
# buffer.
#
  @staticmethod
  def _crc8( buffer:bytearray ) -> int:
    crc0 = 0xFF				# CRC initialisation
    for byte in buffer:
      crc0 ^= byte
      crc1  = ((crc0 << 4) ^ SCD4X.crcn[crc0 >> 4]) & 0xFF
      crc0  = ((crc1 << 4) ^ SCD4X.crcn[crc1 >> 4]) & 0xFF
    return crc0

#
# The next properties return the most recently measured value of either the CO2
# concentration, the relative humidity or the temperature.
#
  @property
  def CO2(self) -> int:  # pylint:disable=invalid-name
    """Returns the CO2 concentration, expressed in PPM."""
    if self.get_data_ready():
      self._read_data()
    return self._co2

  @property
  def humidity(self) -> float:
    """Returns the relative humidity, expressed as a fraction in 0~1."""
    if self.get_data_ready():
      self._read_data()
    return self._humidity

  @property
  def temperature(self) -> float:
    """Returns the current temperature, expressed in degrees Celsius."""
    if self.get_data_ready():
      self._read_data()
    return self._temperature
