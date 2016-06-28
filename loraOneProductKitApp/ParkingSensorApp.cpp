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
	startReportingBattery();
}

void ParkingSensorAppClass::initInterupt(ATTDevice* device)
{
	init(device);
	initMagnetoInterupts(compass);
	startReportingMagnetoChange(compass);

	//specific for magneto
	compass.writeReg(0x12, 0b11100000);  //enable interupt gecongition on all axis for magneto data.
	compass.writeReg(0x13, 0b11111101);

	compass.writeReg(0x14, MAXMARGIN & 0x00FF);
	compass.writeReg(0x15, MAXMARGIN & 0xFF00);

	compass.writeReg(0x16, _baseX & 0x00FF);
	compass.writeReg(0x17, _baseX & 0xFF00);

	compass.writeReg(0x18, _baseY & 0x00FF);
	compass.writeReg(0x19, _baseY & 0xFF00);

	compass.writeReg(0x1A, _baseZ & 0x00FF);
	compass.writeReg(0x1B, _baseZ & 0xFF00);
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
	AppBase::loop();
	sample();
	delay(1000);
}

void ParkingSensorAppClass::sample()
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
}

void ParkingSensorAppClass::loopInterupt()
{
	AppBase::loop();
	if (magnetoChanged())
		sample();
}

ParkingSensorAppClass PSApp;

