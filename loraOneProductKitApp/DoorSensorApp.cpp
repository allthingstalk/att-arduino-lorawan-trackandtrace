// 
// 
// 

#include "DoorSensorApp.h"
#include <Wire.h>

#define MAXMARGIN 0.6
#define LENGTHOFSAMPLINGCYCLCE 1500

#define ACCEL_ADR 0b0011110

#define ACCELERO_COMPENSATION 0.061

uint8_t writeReg(uint8_t reg, uint8_t val)
{
	Wire.beginTransmission(ACCEL_ADR);
	Wire.write(reg);
	Wire.write(val);
	Wire.endTransmission();
	delayMicroseconds(10000);
}

uint8_t readReg(uint8_t reg)
{
	Wire.beginTransmission(ACCEL_ADR);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(ACCEL_ADR, 0x01);

	uint8_t val = Wire.read();
	Wire.endTransmission();

	return val;
}


void DoorSensorAppClass::init(ATTDevice* device)
{
	AppBase::init(device);						//call base method
	//need to init the accelerometer: make certain it is turned on and set up correctly.
	Wire.begin();
	compass.init(LSM303::device_D);
	compass.enableDefault();
	calibrate();
	_isOpen = false;
	if (_device)
		_device->Send(false, BINARY_SENSOR);				//let the cloud know the current status.

}

void DoorSensorAppClass::prepareInterrupts(int16_t x, int16_t z)
{
	x = (x * ACCELERO_COMPENSATION) / 16;					//16 mg step size at 2g accuratie
	z = (z * ACCELERO_COMPENSATION) / 16;
	SerialUSB.print("x: "); SerialUSB.print(x, 10); SerialUSB.print("; "); SerialUSB.println(x, 2);
	SerialUSB.print("z: "); SerialUSB.println(z, 2);
	writeReg(0x1F, 0b10000000); //reboot
	writeReg(0x20, 0b01010111); //ctrl1
	writeReg(0x30, 0b10000010); // Axes mask
	writeReg(0x32, x & 0x7F); // Threshold
	writeReg(0x33, 0b00000000); // Duration

	writeReg(0x34, 0b10100000); // Axes mask
	writeReg(0x36, z & 0x7F); // Threshold
	writeReg(0x37, 0b00000000);
	writeReg(0x22, 0b00100000);
	writeReg(0x23, 0b00100000);
}

void DoorSensorAppClass::initInterupt(ATTDevice* device)
{
	AppBase::init(device);						//call base method
												//need to init the accelerometer: make certain it is turned on and set up correctly.
	Wire.begin();
	compass.init(LSM303::device_D);
	compass.enableDefault();
	initAcceleroInterupts(compass);
	int16_t x, y, z;										//the current x, y, z acceleration values (for gravity compensation)
	calibrate(x, y, z);
	prepareInterrupts(x, y, z);
	_isOpen = false;
	if (_device)
		_device->Send(false, BINARY_SENSOR);				//let the cloud know the current status.


	//startReportingMovement(compass);

	startReportingBattery();
}

void DoorSensorAppClass::calibrate(int16_t &x, int16_t &z)
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
		_baseLine = _baseLine - (_baseLine / i) + (curHeading / i);
		SerialUSB.print("heading: "); SerialUSB.print(curHeading);
		SerialUSB.print(",  avg heading: "); SerialUSB.println(_baseLine);

		if (abs(compass.a.x) > x) {
			x = abs(compass.a.x);
			SerialUSB.print("max x: "); SerialUSB.println(x * ACCELERO_COMPENSATION);
		}
		if (abs(compass.a.z) > z) {
			z = abs(compass.a.z);
			SerialUSB.print("max z: "); SerialUSB.println(z * ACCELERO_COMPENSATION);
		}

		delay(100);
	}
	informEndOfCalibration();
}


void DoorSensorAppClass::loop()
{
	AppBase::loop();
	checkGatePosition();
	delay(1000);
}

void DoorSensorAppClass::checkGatePosition()
{
	compass.read();

	//see page 10 in http://www.st.com/content/ccc/resource/technical/document/datasheet/1c/9e/71/05/4e/b7/4d/d1/DM00057547.pdf/files/DM00057547.pdf/jcr:content/translations/en.DM00057547.pdf
	//for convertion numbers (0.061), which depends on the sampling frequency
	SerialUSB.print("x: "); SerialUSB.print(compass.a.x * ACCELERO_COMPENSATION);
	SerialUSB.print(", y: "); SerialUSB.print(compass.a.y * ACCELERO_COMPENSATION);
	SerialUSB.print(", z: "); SerialUSB.println(compass.a.z * ACCELERO_COMPENSATION);

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
}


bool inMovingCycle = false;				//when accelero triggered, sample accelerometer for a couple of seconds before putting it back to sleep. acceleration is only at the beginning and end of the action,
unsigned long startOfMovemenetCycle = 0;

void DoorSensorAppClass::loopInterupt()
{
	bool moved = hasMoved();
	if(moved)										//check this at every run, so that every movement triggers a full cycle, even if there is overlaps:the lat interrupt triggers a full cycle.
		startOfMovemenetCycle = millis();
	if (moved || inMovingCycle)
	{
		if (inMovingCycle == false)
		{
			SerialUSB.println("has moved, starting movement sampling cycle");
			inMovingCycle = true;
		}
		else if (startOfMovemenetCycle + LENGTHOFSAMPLINGCYCLCE <= millis()) {
			inMovingCycle = false;
			SerialUSB.println("stop movement sampling cycle");
		}
		else
			SerialUSB.println("in movement sampling cycle");
		checkGatePosition();
	}
	else 
		AppBase::loop();
}

DoorSensorAppClass DSApp;

