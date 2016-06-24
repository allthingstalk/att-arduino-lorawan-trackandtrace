// BoardFeatures.h

#ifndef _BOARDFEATURES_h
#define _BOARDFEATURES_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Arduino.h"

class BoardFeaturesClass
{
 protected:


 public:
	void init();
	void RedLedOn();
	void RedLedOff();

	inline void GreenLedOn() { digitalWrite(LED_GREEN, LOW); };
	inline void GreenLedOff() { digitalWrite(LED_GREEN, HIGH); };

	inline void BlueLedOn() { digitalWrite(LED_BLUE, LOW); };
	inline void BlueLedOff() { digitalWrite(LED_BLUE, HIGH); };
	
	// Start delay of 5s to allow for new upload after reset
	void WaitForUpload();

};

extern BoardFeaturesClass BoardFeatures;

#endif

