// 
// 
// 

#include "ParkingSensorApp.h"

#include <Wire.h>

#define MAXMARGIN 500

void ParkingSensorAppClass::init(ATTDevice* device)
{
	AppBase::init(device);						//call base method
	//_device = device;
	//need to init the accelerometer: make certain it is turned on and set up correctly.
	Wire.begin();
	compass.init();
	compass.enableDefault();
	calibrate();
	_isFull = false;
	if (_device)
		_device->Send(false, BINARY_TILT_SENSOR);				//let the cloud know the current status.
}


void ParkingSensorAppClass::calibrate()
{
	//read the compass for a nr of seconds, first allow it to stabelize (user might want to remove a ladder first)
	//after stabelized, take an average of the values and store as base line. 
	//let the user know the state through the led:
	//     - blue: waiting to start calibration, time to remove the ladder
	//     - red: calibrating
	//	   - green: calibrated.  after 1 sec, the light goes out.

	
	informStartOfCalibration();
	float x, y, z = 0;
	for (int i = 1; i <= 50; i++)
	{
		compass.read();
		SerialUSB.print("x: "); SerialUSB.print(compass.m.x); SerialUSB.print("y: "); SerialUSB.print(compass.m.y);  SerialUSB.print("z: "); SerialUSB.println(compass.m.z);
		x = x - (x / i) + ((float)compass.m.x / i);
		y = y - (y / i) + ((float)compass.m.y / i);
		z = z - (z / i) + ((float)compass.m.z / i);
		SerialUSB.print("avg x: "); SerialUSB.print(x); SerialUSB.print("avg y: "); SerialUSB.print(y);  SerialUSB.print("avg z: "); SerialUSB.println(z);
		delay(100);
	}
	_baseX = (int16_t)x;
	_baseY = (int16_t)y;
	_baseZ = (int16_t)z;
	informEndOfCalibration();
}

void ParkingSensorAppClass::loop()
{
	int dif;
	compass.read();
	SerialUSB.print("x: "); SerialUSB.print(compass.m.x); SerialUSB.print("y: "); SerialUSB.print(compass.m.y);  SerialUSB.print("z: "); SerialUSB.println(compass.m.z);
	dif = abs(compass.m.x - _baseX) + abs(compass.m.y - _baseY) + abs(compass.m.z - _baseZ);
	SerialUSB.print("dif: "); SerialUSB.println(dif);
	if (dif > MAXMARGIN && _isFull == false) {
		SerialUSB.println("newly parked car");
		_isFull = true;
		if (_device)
			_device->Send(true, BINARY_TILT_SENSOR);
	}
	else if (dif < MAXMARGIN && _isFull == true) {
		SerialUSB.println("car left parking");
		_isFull = false;
		if (_device)
			_device->Send(false, BINARY_TILT_SENSOR);
	}
	delay(1000);
}

ParkingSensorAppClass PSApp;

