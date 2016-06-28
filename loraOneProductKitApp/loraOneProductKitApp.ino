#include "SmartLockApp.h"
#include <MicrochipLoRaModem.h>
#include <ATT_LoRa_IOT.h>
#include "keys.h"

#include "BoardFeatures.h"
#include "PowerManager.h"
#include "Configs.h"


//#include "ParkingSensorApp.h"
#include "DoorSensorApp.h"
//#include "SmartLockApp.h"

// Test sleep current

MicrochipLoRaModem Modem(&Serial1, &SerialUSB);
ATTDevice Device(&Modem, &SerialUSB);

void setup()
{
	while(!SerialUSB){}
	SerialUSB.println("start");
	PowerManager.init();
	BoardFeatures.init();
	PowerManager.setPower(HIGH);			//turn board on
	//PowerManager.resetAllDigitalPins();
	//PowerManager.setGPSPower(LOW);			// Disable GPS
	//BoardFeatures.WaitForUpload();			// Do not remove as there is no wake up trigger, so we need to wait a little bit to allow for uploading new scripts.
	PowerManager.resetPin(LED_RED);			// Reset the LED pin that was used for the 'waitforUpload'.

	// Note: It is more power efficient to leave Serial1 running
	Serial1.begin(57600);

	// Put LoRa to sleep of 30s
	//Serial1.println("sys sleep 30000");
	//delay(100);
	//attachInterrupt(BUTTON, buttonPressed, LOW);

	Serial1.begin(Modem.getDefaultBaudRate());					// init the baud rate of the serial connection so that it's ok for the modem

	while (!Device.Connect(DEV_ADDR, APPSKEY, NWKSKEY));
	SerialUSB.println("Ready to send data");

	//SLApp.init(&Device);
	DSApp.initInterupt(&Device);
  //PSApp.init(&Device);
}

//void buttonPressed()
//{
	//Blink the LED for 5s
//	digitalWrite(13, HIGH);
//	delayMicroseconds(500000);
//	digitalWrite(13, LOW);
//}

void loop()
{
	//PowerManager.sleep();
	//Stay awake 10s for new upload
	//delay(10000);
	//SLApp.loop();
    DSApp.loopInterupt();
    //PSApp.loop();
}
