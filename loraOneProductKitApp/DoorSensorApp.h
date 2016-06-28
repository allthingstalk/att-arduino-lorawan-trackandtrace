// DoorSensorApp.h

#ifndef _DOORSENSORAPP_h
#define _DOORSENSORAPP_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "AppBase.h"
#include "LSM303.h"
#include <ATT_LoRa_IOT.h>

class DoorSensorAppClass : public AppBase
{
 protected:
	 LSM303 compass;
	 bool _isOpen;
	 float _baseLine;				//the base line for a closed door.

	 //samples the heading for a couple of seconds and uses this as a base line so that 
	 //it can determine when the gate/door has moved/comes back into position.
	 void calibrate();
 public:
	 virtual void init(ATTDevice* device) override;
	 void initInterupt(ATTDevice* device);
	// Inherited via App
	virtual void loop() override;
	void loopInterupt();
};

extern DoorSensorAppClass DSApp;

#endif

