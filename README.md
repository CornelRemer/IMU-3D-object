# Inertial measurement unit (IMU)
This IMU was part of my master thesis. It combines a gyro, an accelerometer, a magnetometer  and an arduino Uno to an IMU. Its purpose is the measurement of the pitch, roll and yaw angle of a system.
The use of the [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) prevents the pitch and roll values from drifting. The filter combines the yaw angle and the values of the magnetometer (x,y and z) to a compass which is less vulnerable to external magnetic fields.

## Sensor
Accelerometer and magnetometer: LSM303 DLHC
Gyro: L3GD20

## Connection
Adafruit 9dof Accel & Gyro
* VIN  -->   5 V
* GND  -->   GND
* SCL  -->   A5
* SDA  -->   A4

## Calibration
To calibrate the magnetometer I've used code snippets from Chris Holm and the calibration software Magneto.
* Calibration Software [Magneto v1.2](https://sites.google.com/site/sailboatinstruments1/home)
* Code by [Chris Holm](https://forums.adafruit.com/viewtopic.php?f=8&t=136357&p=685932) Jul 25, 2018

# Objectloader
This modul (objectloader.py) is used to load a Wavefront object created by [Blender](https://www.blender.org/).
It requires a *.obj format:
> vertices: v 1.000000 -1.000000 -1.000000
> faces: f 2//1 3//1 4//1

# Live plot
The 'imu_opengl_live_plot.py' script is used to rotate a 3D Wavefront object. The rotation dependes on the pitch, roll and yaw angle which will be measured by the IMU. 
* Data coming from the IMU has to be in the following format: pitch, roll, yaw (Example: -1.89,0.18,210.63)
* Port COM4 and baudrate 115200 are used for the serial connection.
* To maintain a good performance it could be necessary to comment out edges in function 'drawMyObject()' (line 73-78) or use an object with less vertices.