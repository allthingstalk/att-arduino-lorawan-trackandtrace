// Configs.h

#ifndef _CONFIGS_h
#define _CONFIGS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class ConfigsClass
{
 protected:


 public:
	void init();
};

extern ConfigsClass Configs;

#endif

