// PowerManager.h

#ifndef _POWERMANAGER_h
#define _POWERMANAGER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class PowerManagerClass
{
 protected:


 public:
	void init();
	//sets the power mode of the board
	void setPower(byte value);
	//set the power of the GPS
	void setGPSPower(byte value);
	void disableUSB();
	void enableUSB();
	// Resets the default digital pins to not connectd
	void resetAllDigitalPins();
	void resetPin(uint8_t pin);

	//put the device in deep sleep (disables USB during process)
	void sleep();
};

extern PowerManagerClass PowerManager;

#endif

