// 
// 
// 

#include "BoardFeatures.h"
	

void BoardFeaturesClass::init()
{
	pinMode(LED_RED, OUTPUT);

}

void BoardFeaturesClass::RedLedOff()
{
	digitalWrite(LED_RED, HIGH);
}


void BoardFeaturesClass::RedLedOn()
{
	digitalWrite(LED_RED, LOW);
}


void BoardFeaturesClass::WaitForUpload()
{
	BoardFeatures.RedLedOn();				// LED On to indicate we are waiting.
	delay(5000);
	BoardFeatures.RedLedOff();				//and we are done
}



BoardFeaturesClass BoardFeatures;

