// App.h

#ifndef _APP_h
#define _APP_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <ATT_LoRa_IOT.h>
#include "RTCZero.h"
#include "LSM303.h"

class AppBase
{
 protected:
	 ATTDevice* _device;
	 RTCZero rtc;

	 void informStartOfCalibration();
	 void informEndOfCalibration();
	
 public:
	 virtual void init(ATTDevice* device) { _device = device; };
	 virtual void loop();
	 void startReportingBattery();
	 void initAcceleroInterupts(LSM303& compass);
	 void initMagnetoInterupts(LSM303& compass);
	 void startReportingMovement(LSM303& compass);
	 void startReportingMagnetoChange(LSM303& compass);
	 //returns true if the accelerometer reported movement
	 //resets the switch so that new movement can be detected on the next run.
	 bool hasMoved();
	 //returns true if the accelerometer reported a change in the magnetic field
	 //resets the switch so that a new change can be detected on the next run.
	 bool magnetoChanged();

	 void stopAcceleroInterupt(LSM303& compass);
	 void stopMagnetoInterupt(LSM303& compass);
};


#endif

