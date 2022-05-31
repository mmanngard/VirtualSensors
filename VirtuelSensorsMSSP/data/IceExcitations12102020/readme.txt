Each measurement has two files. One file contains torques measured from the motors (the filename ends with “_motor”) 
and one file contains the sensor data. As an example files from ice excitation test are named as follows:
- IceExcitation.csv (sensor data) 
- IceExcitation_motor.csv (motor torque data)

The measurements does not start at the same time because of how the measurement process works. 
There is approximately one second delay before the sensor start to measure. However, the times of each 
measurement file are taken from the same clock, hence it is possible to use the times to compare the signals.

Sensor-data-files has eight columns:
- Time as seconds (s)
	- column name: time
	- the time starts when the CompactRIO is turned on; hence it does not start from zero
- Five columns which has the degree (o) of the five encoders (check provided figure from OneDrive for sensor locations):
	- column names are: first encoder, second encoder, third encoder, fourth encoder, fifth encoder
	- the angle measurement starts when the system starts to rotate. This way the constant twist can be measured as well. 
- Upper torque sensor data as newton-meters (Nm) (check provided figure from OneDrive for sensor locations):
	- column name: upper torque sensors
- Lower torque sensor data as newton-meters (Nm) (check provided figure from OneDrive for sensor locations):
	- column name: lower torque sensor

Motor-torque-files has five columns:
- Time as seconds (s)
	- column name: time
	- the time starts when the CompactRIO is turned on; hence it does not start from zero
- Driving motor torque setpoint as newton-metres (Nm)
	- column name: motor setpoint
- Driving motor measured torque as newton-metres (Nm)
	- column name: motor real
- “Propeller” (load motor) motor torque setpoint as newton-metres (Nm)
	- column name: propeller setpoint
- “Propeller” (load motor) motor measured torque as newton-metres (Nm)
	- column name: propeller real

Motor nominal torque is 11.9 Nm
Propeller motor gear ratio 1:8

Below is the list of the tests run with different excitations and the measurement filenames. 
These tests are made with provided excitations. In some of these measurements, the “propeller” (load) motor has an offset. 
This means that the “propeller” motor has constant torque. With this constant torque, 
we try keep the load torque positive and void backlash from gears. 


Ice Excitations:

	IceExcitation1.csv / IceExcitation1_motor.csv
	- offset zero, excitation gain 10 % of the motor nominal torque

	IceExcitation2.csv / IceExcitation2_motor.csv
	- offset 1%, excitation gain 10 % of the motor nominal torque

	IceExcitation3.csv / IceExcitation3_motor.csv
	- offset 5%, excitation gain 10 % of the motor nominal torque
