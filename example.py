#!/usr/bin/python3
#
# This script retrieves the temperature and the air pressure as measured by a
# Bosch BMP280 sensor, as well as the temperature and the relative humidity as
# measured by a Sensiron SHT31 sensor. Those two sensors are located very close
# to one another. The results are reported to the xymon monitoring server, which
# will also display the difference in the two measured temperatures.
#
import datetime
import re
import socket
import BMEP280				# Bosch BMP280 API
import SHT31				# Sensiron SHT31-DIS API

XyServ= '127.0.0.1'			# Xymon server address
XyPort= 1984				# Xymon server TCP port
XyHost= 'AtHome'			# Source of this test
XyTest= 'lw-pt'				# Weather in living room
XyClr = 'green'				# Test status

def InformXymon( Hst, Tst, Clr, Msg ):	# Send status message to the xymon server
  global XyServ, XyPort
  XyTime= datetime.datetime.today().isoformat( sep=' ' )
  XyTime= re.sub( '\.\d+$', '', XyTime )	# Chop off fractional second
  XyPars= { 'host': Hst, 'test': Tst, 'colour': Clr, 'time': XyTime,
           'message': Msg }
  XyMsg = '''status {host}.{test} {colour} {time}
{message}'''.format( **XyPars )
  try:
    s= socket.socket( socket.AF_INET, socket.SOCK_STREAM )
  except socket.error as msg:
    return False
  if not re.search( '^[\d\.]+$', XyServ ):
    try:
      XyServ= socket.gethostbyname( XyServ )
    except socket.gaierror:
      return False
  try:
    s.connect( (XyServ, XyPort) )
    s.sendall( XyMsg.encode() )
    return True
  except socket.error as msg:
    return False
  finally:
    s.close()

#
# MAIN PROGRAM.
#
sensor= BMEP280.BMP280( 0x76 )		# BMP280 sensor
temp= sensor.read_temperature()		# Temperature [C]
pres= sensor.read_pressure()		# Air pressure [P]

Table = "%-12s : %7.2f [%s]\n"
XyMsg = "<b>Bosch sensor BMP280 #0</b>\n\n"
XyMsg+= Table % ( 'Temperature', temp, 'C'  )
XyMsg+= Table % ( 'Pressure'   , pres/100, 'hP' )

XyMsg+= "<!-- linecount=1 -->\n"
XyMsg+= "<!--DEVMON RRD: lw-pt 0 0\n"
XyMsg+= "DS:temp:GAUGE:600:-100:100 DS:pres:GAUGE:600:0:200000\n"
XyMsg+= "0 %.3f:%d\n" % (temp,pres)
XyMsg+= "-->"

sensor= SHT31.SHT31( 0x44 )
temp= sensor.read_temperature()
humi= sensor.read_humidity() / 100.0
dewp= sensor.read_dew_point()
errr= sensor.read_errorcount()

XyMsg+= "<b>Sensirion sensor SHT31-DIS #1</b>\n\n"
XyMsg+= Table % ( 'Temperature', temp, 'C' )
XyMsg+= Table % ( 'Humidity'   , humi, ''  )
XyMsg+= Table % ( 'Dew point'  , dewp, 'C' )
XyMsg+= Table % ( 'CRC errors' , errr, ''  )

if errr > 0:
  XyClr= 'yellow'

XyMsg+= "<!--DEVMON RRD: lw-ht 0 0\n"
XyMsg+= "DS:temp:GAUGE:600:-100:100 DS:humi:GAUGE:600:0:1\n"
XyMsg+= "1 %.3f:%.4f\n" % (temp,humi)
XyMsg+= "-->"

InformXymon( XyHost, XyTest, XyClr, XyMsg )

