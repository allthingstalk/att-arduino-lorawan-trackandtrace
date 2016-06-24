// 
// 
// 

#include "DoorSensorApp.h"
#include <Wire.h>

#define MAXMARGIN 5

void DoorSensorAppClass::init(ATTDevice* device)
{
	AppBase::init(device);						//call base method
	//need to init the accelerometer: make certain it is turned on and set up correctly.
	Wire.begin();
	compass.init();
	compass.enableDefault();
	calibrate();
	_isOpen = false;
	if (_device)
		_device->Send(false, BINARY_SENSOR);				//let the cloud know the current status.

}

void DoorSensorAppClass::calibrate()
{
	//read the compass for a nr of seconds, first allow it to stabelize (user might want to remove a ladder first)
	//after stabelized, take an average of the values and store as base line. 
	//let the user know the state through the led:
	//     - blue: waiting to start calibration, time to remove the ladder
	//     - red: calibrating
	//	   - green: calibrated.  after 1 sec, the light goes out.


	informStartOfCalibration();
	_baseLine = 0;
	for (int i = 1; i <= 50; i++)
	{
		compass.read();
		float curHeading = compass.heading();
		SerialUSB.print("heading: "); SerialUSB.println(curHeading);
		_baseLine = _baseLine - (_baseLine / i) + (curHeading / i);
		SerialUSB.print("avg heading: "); SerialUSB.println(_baseLine);
		delay(100);
	}
	informEndOfCalibration();
}


void DoorSensorAppClass::loop()
{
	compass.read();
	float curHeading = compass.heading();
	SerialUSB.print("heading: "); SerialUSB.println(curHeading);
	float dif = curHeading - _baseLine;
	dif = abs(dif);
	SerialUSB.print("dif: "); SerialUSB.println(dif);
	if (dif > MAXMARGIN && _isOpen == false) {
		SerialUSB.println("gate open");
		_isOpen = true;
		if (_device)
			_device->Send(true, BINARY_SENSOR);
	}
	else if (dif < MAXMARGIN && _isOpen == true) {
		SerialUSB.println("gate closed");
		_isOpen = false;
		if (_device)
			_device->Send(false, BINARY_SENSOR);
	}
	delay(1000);
}

DoorSensorAppClass DSApp;

