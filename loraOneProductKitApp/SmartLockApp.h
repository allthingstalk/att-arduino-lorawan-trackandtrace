// SmartLockApp.h

#ifndef _SMARTLOCKAPP_h
#define _SMARTLOCKAPP_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "AppBase.h"
#include "LSM303.h"
#include <ATT_LoRa_IOT.h>

class SmartLockAppClass : public AppBase
{
 protected:
	 LSM303 compass;
	 bool _wasMoving;
	 double _prevX;
	 double _prevY;
	 double _prevZ;

	 //sample the accelerometer, return if there is movement
	 bool isMoving();
	 //stores and sends the 'movement value to the cloud + also sends the gps coordinates.
	 void setState(bool value);
	 //send current location to cloud. if blocking is true, the function will block untill the message was succesfully sent.
	 void sendGPSFix(bool blocking);
 public:
	 virtual void init(ATTDevice* device) override;
	 virtual void loop() override;
};

extern SmartLockAppClass SLApp;

#endif

