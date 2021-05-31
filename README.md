# Arduino_Nano_Sense_Data_Logger
Log accelerometer data from an Arduino Nano 33 Sense BLE to an SD Card

The initial version started from the [Adafruit_LSM9DS1 sample](https://github.com/adafruit/Adafruit_LSM9DS1/blob/master/examples/lsm9ds1/lsm9ds1.ino) but then added in the writing to the SD card.

There are a few changes made to the stock [Adafruit_LSM9DS1](https://github.com/adafruit/Adafruit_LSM9DS1) library to increase the read rate to about 3-4ms per read.  Mainly:
 * Adafruit_LSM9DS1::getEvent - Was enhanced to only read the Accel, Gyro and Mag registers if the caller wanted that data
 * The SENSORS_GRAVITY_STANDARD define was changed to 1.0f so that accelerometer data was returned in Earth Gs (instead of m/s/s)