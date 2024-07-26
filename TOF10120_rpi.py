#
# A Python driver for the distance sensor, time-of-flight tof10120
#
# Sensor TOF10120 is equipped with two communication interfaces, an asynchronous
# serial interface and an I2C bus interface. Via both interfaces the same
# information is accessible, but the numbering and the formatting are quite
# different. This driver hides the differences between the two and delivers a
# uniform programmers interface which is independent of the physical interface
# being used.
#
# In order to create a uniform interface, both the driver for the asynchronous
# serial interface (UART) as well as the driver for the I2C interface do use the
# notion of registers, which can be read and written. The value retrieved as
# well as the value written into a register is an integer number.
#
# The registers as recognised by this driver are defined in the table below.
#
#   -Register address-   Content
#   Driver  UART   I2C
#        0     1     6   Offset [mm]
#        1     2    --   Serial port send interval [ms]
#        2     3     8   Distance mode, 0=filtered, 1=real-time
#        3     4    12   Maximum distance [mm]
#        4     5     9   Medium mode, 0=active send via UART, 1=passive read
#        5     6     0   Distance [mm]
#        6     7    15   I2C address * 2
#        7    --     4   Distance, filtered [mm]
#
# Written by W.J.M. Nelis, wim.nelis@ziggo.nl, 2024.07
#
# Note: this version of the driver is used on a Raspberry Pi, both with OS 11
#       (bullseye) and 12 (bookworm).
# Note: As the sensor does do clock stretching, up to 10**-5 [s], the frequency
#       of the I2C bus on the Raspberry Pi should be less than 25 [kHz] to mask
#       the stretching.
#
import re
import serial
from   smbus2 import SMBus
import time


#
# ----- Class tof_uart -----
#
# Class tof_uart implements a simple driver for the asynchronous serial (UART)
# interface of the TOF10120 sensor. At the creation of an instance, the medium
# mode is checked. If the sensor is pushing measurements periodically via it's
# serial interface, (register 4 == 0), it is disabled by setting this register
# to 1.
#
class tof_uart():
  def __init__( self, port ):
    self.port= port
    self.tof = serial.Serial( port=port, baudrate=9600,
       parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
       bytesize=serial.EIGHTBITS, timeout=1 )		# UART device object
 # Define the mapping of the driver registers to the addressing and formats
 # needed via the serial interface. The read table specifies the command to
 # retrieve a specific register and the regular expression to recognise and
 # decode the response from the sensor.
    self.rcrt= (			# Read command-response table
      ( 'r1#', r'^D=(\d\d?)mm$'        ),	# Register 0
      ( 'r2#', r'^T=(\d{2,4})mS$'      ),	# Register 1
      ( 'r3#', r'^M=([01])$'           ),	# Register 2
      ( 'r4#', r'^Max[=>](\d{2,4})mm$' ),	# Register 3
      ( 'r5#', r'^S=([01])$'           ),	# Register 4
      ( 'r6#', r'^L=(\d{,4}mm)$'       ),	# Register 5
      ( 'r7#', r'^I=(\d{,3})$'         ) )	# Register 6
  # The write table specifies the format to build the command to set the value
  # of a specific register. Note the format of driver register 0: this value can
  # be negative.
    self.wct = (			# Write command table
      ( 's1{:+}#', ),			# Register 0
      ( 's2-{}#' , ),			# Register 1
      ( 's3-{}#' , ),			# Register 2
      ( 's4-{}#' , ),			# Register 3
      ( 's5-{}#' , ),			# Register 4
      ( None       ),			# Register 5
      ( 's7-{}#' , ) )			# Register 6
    self.wr  = r'^(ok!|fail)$'		# RE to recognise a response on write

  # See if the automatic sending of measurements is enabled. If so, disable it.
  # As the measurements being pushed and the commands sent by this script may
  # collide, the process of checking and disabling is repeated multiple times
  # until the automatic sending is disabled.
    while True:
      val= self.read_register( 4 )
      if   val is None:
        pass
      elif val == 1:
        break
      elif val == 0:
        if self.write_register( 4, 1 ):
          break
      time.sleep( 0.1 )

 #
 # Private method _read_line performs a (timed) readline, convert the byte
 # string into an actual string and removes the trailing blank spaces, including
 # line terminators.
 #
  def _read_line( self ):
    val= self.tof.read_until().decode()
#   dbg= val.replace( '\r', '<Cr>' )		# TEST
#   dbg= dbg.replace( '\n', '<Lf>' )		# TEST
#   print( f'Debug: _read_line: "{dbg}"' )	# TEST
    return val.rstrip()

 #
 # Private method _write_line converts the given string into a byte string and
 # writes the latter to the device.
 #
  def _write_line( self, val ):
#   print( f'Debug: _write_line: "{val}"' )	# TEST
    self.tof.write( val.encode() )

  def open( self ):
    if not self.tof.is_open:
      self.tof.port= self.port
      self.tof.open()

  def close( self ):
    if self.tof.is_open:
      self.tof.close()

 #
 # Method read_register retrieves the content of one register. The returned
 # value either the current value of the register or None if the register
 # address is out of range or retrieval of the register failed.
 #
 # After sending an r-command, a line termination sequence is received, prior to
 # the response on the command. The line termination sequence is “\r\r\n”. The
 # line which is send as a response on the command also ends with the same line
 # terminator sequence. There is however one exception: after command “r6#”, no
 # line termination sequence is send.
 #
  def read_register( self, adr ):
    try:
      crte= self.rcrt[adr]		# Retrieve command-response table entry
    except IndexError:
      return None			# Register address not supported
  #
    self._write_line( crte[0] )		# Send the read command
    for i in range(3):
      aline= self._read_line()		# Try to read the response
      if aline != '':  break		# Leave loop if successful
    else:
      return None			# Reading the response failed
  #
    mo= re.match( crte[1], aline )	# Check format / content
    if mo:
      return int( mo.group(1) )
    else:
#     print( f'Debug: read_register: "{aline}"' )	# TEST
      return None

 #
 # Method write_register changes the content of a register. It returns a False
 # value if the register address is out of range, the register is read-only, the
 # response is not recognised or if writing the value failed. It returns a True
 # value if writing the register succeeded.
 #
  def write_register( self, adr, val ):
    try:
      cte= self.wct[adr]		# Retrieve command table entry
    except IndexError:
      return False			# Register address not supported
    if cte is None:  return False	# Read only register
  #
    aline= cte[0].format( val )
    self._write_line( aline )		# Send the write command
    for i in range(3):
      aline= self._read_line()		# Try to read the response
      if aline != '':  break		# Leave loop if successful
    else:
      return False			# Reading the response failed
  #
    mo= re.match( self.wr, aline )	# Check for 'ok!' or 'fail'
    if mo:
      return mo.group(1) == 'ok!'
    else:
#     print( f'Debug: write_register: "{aline}"' )	# TEST
      return False			# Unrecognised response


#
# ----- Class tof_i2c -----
#
# Class tof_i2c implements a simple driver for the I2C interface of the distance
# sensor TOF10120.
#
class tof_i2c():
  def __init__( self, addr ):
    self.addr= addr
    self.tof = SMBus( 1 )		# I2C device object, bus # 1
  #
  # Table rft specifies for each (high level) register the associated address at
  # the I2c level, it's length in octets, the struct.pack format string and the
  # writable flag. Note that the format string includes a specification of the
  # byte order.
  #
    self.rft= (				# Register formatting table
      (  6, 2, '>h', True  ),		# Register 0
      None,				# Register 1
      (  8, 1,  'B', True  ),		# Register 2
      ( 12, 2, '>H', False ),		# Register 3
      (  9, 1,  'B', True  ),		# Register 4
      (  0, 2, '>H', False ),		# Register 5
      ( 15, 1,  'B', True  ),		# Register 6
      (  0, 4, '>H', False ) )		# Register 7

  # See if the automatic sending of measurements is enabled. If so, disable it.
    if self.read_register( 4 ) == 0:
      self.write_register( 4, 1 )

  def open( self ):
    pass

  def close( self ):
    self.tof.close()

 #
 # Method read_register retrieves the content of a register. The returned value
 # either the current value of the register or None if the register address is
 # out of range or retrieval of the register failed.
 #
  def read_register( self, adr ):
    try:
      fte= self.rft[adr]		# Retrieve formatting table entry
    except IndexError:
      return None			# Register address not supported
    if fte is None:  return None	# Register address not supported
  #
    if 1<= fte[1] <= 2:
      try:
        bfr= bytes( self.tof.read_i2c_block_data( self.addr, fte[0], fte[1] ))
        val= struct.unpack( fte[2], bfr )
      except OSError:
        return None
    else:
      return None
  #
    if len(val) == 1:
      return val[0]			# Unpack tuple
    else:
      return None			# Something wrong

 #
 # Method write_register changes the content of a register. It returns a False
 # value if the register address is out of range, the register is read-only or
 # if writing the value failed. It returns a True value if writing the register
 # succeeded.
 #
  def write_register( self, adr, val ):
    try:
      fte= self.rft[adr]		# Retrieve formatting table entry
    except IndexError:
      return False			# Register address not supported
    if fte is None:  return False	# Register address not supported
    if not fte[3]:   return False	# Register is not writable
  #
    if 1 <= fte[1] <= 2:
      bfr= struct.pack( fte[2], val )
      try:
        tof.write_i2c_block_data( sensor, fte[0], list(bfr) )
      except Exception:
        return False
    else:
      return False
  #
    return True
