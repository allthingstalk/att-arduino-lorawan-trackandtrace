// 
// 
// 

#include "PowerManager.h"

void PowerManagerClass::init()
{
	pinMode(ENABLE_PIN_IO, OUTPUT);
	//pinMode(GPS_ENABLE, OUTPUT);  -> done from ublox lib

}

void PowerManagerClass::setPower(byte value)
{
	digitalWrite(ENABLE_PIN_IO, value);
}

void PowerManagerClass::setGPSPower(byte value)
{
	digitalWrite(GPS_ENABLE, value);
}

void PowerManagerClass::disableUSB()
{
	USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
}

void PowerManagerClass::enableUSB()
{
	USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
}

void PowerManagerClass::resetAllDigitalPins()
{
	for (uint8_t i = 0; i < NUM_DIGITAL_PINS; i++)
	{
		resetPin(i);
	}
}

void PowerManagerClass::sleep()
{
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;	// Set sleep mode
	disableUSB();
	__WFI();							//Enter sleep mode
	//...Sleep forever or until interupt has arrived.
	enableUSB();
}

void PowerManagerClass::resetPin(uint8_t pin)
{
	PORT->Group[g_APinDescription[pin].ulPort].PINCFG[g_APinDescription[pin].ulPin].reg = (uint8_t)(0);
	PORT->Group[g_APinDescription[pin].ulPort].DIRCLR.reg = (uint32_t)(1 << g_APinDescription[pin].ulPin);
	PORT->Group[g_APinDescription[pin].ulPort].OUTCLR.reg = (uint32_t)(1 << g_APinDescription[pin].ulPin);
}

PowerManagerClass PowerManager;

