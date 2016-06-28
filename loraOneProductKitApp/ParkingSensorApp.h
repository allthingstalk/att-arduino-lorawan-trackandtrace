// ParkingSensorApp.h

#ifndef _PARKINGSENSORAPP_h
#define _PARKINGSENSORAPP_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "AppBase.h"
#include "LSM303.h"
#include <ATT_LoRa_IOT.h>

class ParkingSensorAppClass: public AppBase
{
 protected:
	 LSM303 compass;
	 //ATTDevice* _device;

	 int16_t _baseX;			//base line for no car
	 int16_t _baseY;			
	 int16_t _baseZ;

	 bool _isFull;				//true when a car is detected in the parking space

	 //samples the magnetormeter for a couple of seconds and uses this as a base line so that 
	 //it can determine when something comes underneath/above it.
	 void calibrate();
	 void sample();
 public:
	virtual void init(ATTDevice* device) override;
	void initInterupt(ATTDevice* device);

	// Inherited via App
	virtual void loop() override;
	void loopInterupt();
};
extern ParkingSensorAppClass PSApp;

#endif

