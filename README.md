# SensorDrivers

In this repository a few drivers for sensors are collected. The drivers are written in Python3, and are developed, tested and used on various models of a Raspberry Pi.

File BMEP280.py contains a class definition for the Bosch BMP280 sensor, as well as a class definition for the Bosch BME280 sensor. The latter driver also includes a method to retrieve the current dew point temperature, which is calculated from the current temperature and the current relative humidity.

File SHT31.py contains a class definition for the Sensir(i)on temperature and humidity sensor SHT31. Like the driver for the Bosch BME280, it contains a method to retrieve the current dew point temperature.

