# SensorDrivers

In this repository a few drivers for sensors are collected. The drivers are written in either Python3, and are developed, tested and used on various models of a Raspberry Pi, or in microPython, in which case they are developed and tested on an ESP8266 or an ESP32.

### BH1750.py

File BH1750_rpi.py contains a class definition for the ROHM BH1750 sensor. This sensor measures the light intensity. The resolution can be set to 0.5, 1 or 4 [lx], while the sensitivity (that is the measuring time) can be set to a value in the range 31..254. Property range returns the minimal and the maximal intensity which can be measured. The maximum is a function of both the resolution and the sensitivity. This version of the sensor driver is used on a Raspberry Pi.

### BMEP280.py

File BMEP280_rpi.py contains a class definition for the Bosch BMP280 sensor, as well as a class definition for the Bosch BME280 sensor. The latter driver also includes a method to retrieve the current dew point temperature, which is calculated from the current temperature and the current relative humidity.

File BMEP280_esp.py also contains class definitions for the Bosch BMP280 and BME280 sensors. This version of the driver is adapted for an ESP8266 microcontroller and thus for use in microPython.

### BME680.py

File BME680_esp.py contains a class definition for the BOSCH BME680 sensor. It does NOT support retrieval of an air quality measure, it is effectively used as a successor of the BME280. It does include an optimised calculation of the calibrated temperature and a calculation of the current dew point temperature. Floating point arithmetic is used, as it seems to be slightly faster, even on an ESP8266 which does not have an FPU.

### SCD4X.py

File SCD4X_esp.py contains a driver for the Sensirion SCD40 or SCD41 sensor, which can measure the CO2 concentration. As some of the commands to this sensor require quite a long time to complete, asyncio is used to allow for other tasks to run in the meantime. Thus some of the methods are coroutines. The number of coroutines is minimised by using `time.sleep()`, in stead of `await asyncio.sleep()`, if the delay is shorter than a few milliseconds.

The performance of this driver as well as it's size are optimised in various ways. (A) the CRC's of a nibble are precomputed, at the expense of a tuple with 16 bytes. (B) A memoryview is used to slice the buffer with the data read or the data to be written. (C) The names of constants start with an underscore. (D) A temperature conversion constant is precomputed and saved in the instance. (E) A part of the methods can be omitted by setting installation parameter _SCD4X_ALL_MODULES to 0 (False). (F) Methods struct.pack and struct.unpack are used to convert 16-bit unsigned numbers into two 8-bit octets and vice versa. (G) A number of run-time checks can be disabled, making the compiled version smaller, by setting the micropython optimisation level to 1 of higher.

### SHT31.py

File SHT31.py contains a class definition for the Sensir(i)on temperature and humidity sensor SHT31. Like the driver for the Bosch BME280, it contains a method to retrieve the current dew point temperature.

### TOF10120.py

The time-of-flicht sensor TOF10120 contains two communication interfaces, asynchronous serial (UART) and I2C. File TOF10120_rpi.py contains a class definition for each interface, delivering a programmers interface which hides the differences between the two communication interfaces. This version is tested on Raspberry Pi computers. File TOF10120_esp.py contains similar class definitions, meant to be used in a microPython environment. This version is used on an ESP32 microcontroller.

The data sheet of this sensor is partly written in Chinese and partly in English. The API part of the data sheet is translated to English, edited and extended with personal observations. This version of the API can be found in file TOF10120.API.english.pdf.
