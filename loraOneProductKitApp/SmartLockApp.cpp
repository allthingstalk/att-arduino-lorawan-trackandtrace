// 
// 
// 

#include "SmartLockApp.h"
#include "Sodaq_UBlox_GPS.h"
#include <Wire.h>

#define MAX_MOVEMENT_TOLERANCE 400

void SmartLockAppClass::init(ATTDevice* device)
{
	AppBase::init(device);						//call base method
												//need to init the accelerometer: make certain it is turned on and set up correctly.
	Wire.begin();
	compass.init();
	compass.enableDefault();
	sodaq_gps.init();
	setState(false);

	StartReportingBattery();

	compass.read();
	_prevX = compass.a.x;		//need to establish a base line for detecting movement.
	_prevY = compass.a.y;
	_prevZ = compass.a.z;

	//char report[80];
	//snprintf(report, sizeof(report), "X: %6d Y:%6d Z:%6d", compass.a.x, compass.a.y, compass.a.z);
	//SerialUSB.println(report);
}

bool SmartLockAppClass::isMoving()
{
	compass.read();

	//char report[80];
	//snprintf(report, sizeof(report), "X: %6d Y:%6d Z:%6d", compass.a.x, compass.a.y, compass.a.z);
	//SerialUSB.println(report);

	double div = abs(_prevX - compass.a.x) + abs(_prevY - compass.a.y) + abs(_prevZ - compass.a.z);
	_prevX = compass.a.x;		//need to establish a base line for detecting movement.
	_prevY = compass.a.y;
	_prevZ = compass.a.z;
	SerialUSB.print("accelero div: "); SerialUSB.println(div);
	return div > MAX_MOVEMENT_TOLERANCE;
}

void SmartLockAppClass::sendGPSFix(bool blocking)
{
	sodaq_gps.setDiag(SerialUSB);

	bool sent = false;
	do
	{
		uint32_t start = millis();
		uint32_t timeout = 900L * 1000;
		SerialUSB.println(String("waiting for fix ..., timeout=") + timeout + String("ms"));
		//scan turns on the gps, 'false' turns it off again.
		if (sodaq_gps.scan(false, timeout)) {
			SerialUSB.println(String(" time to find fix: ") + (millis() - start) + String("ms"));
			SerialUSB.println(String(" datetime = ") + sodaq_gps.getDateTimeString());
			SerialUSB.println(String(" lat = ") + String(sodaq_gps.getLat(), 7));
			SerialUSB.println(String(" lon = ") + String(sodaq_gps.getLon(), 7));
			SerialUSB.println(String(" num sats = ") + String(sodaq_gps.getNumberOfSatellites()));

			_device->Queue((float)sodaq_gps.getLat());
			_device->Queue((float)sodaq_gps.getLon());
			_device->Queue((float)10.);
			_device->Queue((float)10.);
			sent = _device->Send(GPS);
		}
		else {
			SerialUSB.println("No Fix");
			sent = true;								//couldn't get a gps fix, don't block, the app will try again later.
		}
	} while (blocking && sent == false);
}

//stores and sends the 'movement value to the cloud + also sends the gps coordinates.
void SmartLockAppClass::setState(bool value)
{
	if (_device) {
		while (!_device->Send(value, PUSH_BUTTON))		//it's crucial that we report movement. Could be the sign of a stolen object. so block until we managed to send it out.
			delay(15000);
		//sendGPSFix();
	}
	_wasMoving = value;
	SerialUSB.print("curr moving state: "); SerialUSB.println(_wasMoving);

}

void SmartLockAppClass::loop()
{
	AppBase::loop();
	bool curMoving = isMoving();
	if (_wasMoving == false){
		if (curMoving) {
			setState(true);
			delay(15000);			//simulate sleep+wake up by timer
		}
		else
			delay(2000);
	}
	else{
		if (curMoving == false) {
			setState(false);
		}
		else {
			//sendGPSFix();
			delay(35000);			//simulate sleep+wake up by timer
		}
	}
}

SmartLockAppClass SLApp;

