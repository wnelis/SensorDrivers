# SensorDrivers

In this repository a few drivers for sensors are collected. The drivers are written in Python3, and are developed, tested and used on various models of a Raspberry Pi.

### BMEP280.py

File BMEP280_rpi.py contains a class definition for the Bosch BMP280 sensor, as well as a class definition for the Bosch BME280 sensor. The latter driver also includes a method to retrieve the current dew point temperature, which is calculated from the current temperature and the current relative humidity.

File BMEP280_esp.py also contains class definitions for the Bosch BMP280 and BME280 sensors. This version of the driver is adapted for an ESP8266 microcontroller and thus for use in microPython.

### SHT31.py

File SHT31.py contains a class definition for the Sensir(i)on temperature and humidity sensor SHT31. Like the driver for the Bosch BME280, it contains a method to retrieve the current dew point temperature.

