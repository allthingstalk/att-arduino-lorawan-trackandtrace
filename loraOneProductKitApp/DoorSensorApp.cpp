// 
// 
// 

#include "DoorSensorApp.h"
#include <Wire.h>

#define MAXMARGIN 5
#define LENGTHOFSAMPLINGCYCLCE 10000

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

void DoorSensorAppClass::initInterupt(ATTDevice* device)
{
	init(device);
	startReportingBattery();
	initAcceleroInterupts(compass);
	startReportingMovement(compass);
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
	AppBase::loop();
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


bool inMovingCycle = false;				//when accelero triggered, sample accelerometer for a couple of seconds before putting it back to sleep. acceleration is only at the beginning and end of the action,
unsigned long startOfMovemenetCycle = 0;

void DoorSensorAppClass::loopInterupt()
{
	if (hasMoved() || inMovingCycle)
	{
		if (inMovingCycle == false)
		{
			SerialUSB.println("has moved");
			inMovingCycle = true;
			startOfMovemenetCycle = millis();
		}
		else if (startOfMovemenetCycle + LENGTHOFSAMPLINGCYCLCE <= millis()) {
			inMovingCycle = false;
			SerialUSB.println("stop moving cycle");
		}
		else
			SerialUSB.println("in moving cycle");
		loop();
	}
	else 
		AppBase::loop();
}

DoorSensorAppClass DSApp;

