#include "ATT_LoRaOne_common.h"
#include <Wire.h>

#define BATTERY_REPORT_SEC 59						//determins how frequently the interrupt clock wakes up to send battery stats.
#define BATTERY_REPORT_MIN 59
#define BATTERY_REPORT_HOUR 23
#define BATTERY_REPORT_EVERY RTCZero::MATCH_MMSS 	//MATCH_HHMMSS

//These constants are used for reading the battery voltage
#define ADC_AREF 3.3
#define BATVOLTPIN A5 
#define BATVOLT_R1 4.7
#define BATVOLT_R2 10

#define SENDBATTERYEVERY 86400000					//send battery every 24 hours, used to check time if we missed a hartbeat from the interrupt clock.

bool _ledsEnabled = true;


void RedLedOn() { if(_ledsEnabled) digitalWrite(LED_RED, LOW); };
void RedLedOff() { if(_ledsEnabled) digitalWrite(LED_RED, HIGH); };

void GreenLedOn() { if(_ledsEnabled) digitalWrite(LED_GREEN, LOW); };
void GreenLedOff() { if(_ledsEnabled) digitalWrite(LED_GREEN, HIGH); };

void BlueLedOn() { if(_ledsEnabled) digitalWrite(LED_BLUE, LOW); };
void BlueLedOff() { if(_ledsEnabled) digitalWrite(LED_BLUE, HIGH); };


//power

void initPower()
{
	pinMode(ENABLE_PIN_IO, OUTPUT);
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;		//prepare the board for deep sleep mode.
}

void setGPSPower(int8_t value)
{
	digitalWrite(GPS_ENABLE, value);
}

void setPower(int8_t value)
{
	digitalWrite(ENABLE_PIN_IO, value);
}

///pins and leds

void initLeds(bool enabled)
{
	pinMode(LED_RED, OUTPUT);
	pinMode(LED_GREEN, OUTPUT);
	pinMode(LED_BLUE, OUTPUT);
	digitalWrite(LED_RED, HIGH);			//turn off the leds
	digitalWrite(LED_GREEN, HIGH);
	digitalWrite(LED_BLUE, HIGH);
	_ledsEnabled = enabled;
}

void informStartOfCalibration()
{
	//let user know that the procedure will start
	SerialUSB.println("begin calibration");
	if(_ledsEnabled){
		GreenLedOff();
		RedLedOff();
		BlueLedOn();
		delay(4000);
		BlueLedOff();
		RedLedOn();
	}
}
void informEndOfCalibration()
{
	//let user know that the procedure is done.
	SerialUSB.println("end calibration");
	RedLedOff();
}

void signalSendStart()
{
	BlueLedOn();
	/*for(int i = 0; i < 5; i++){
		delay(200);
		BlueLedOn();
		delay(200);
		BlueLedOff();
	}*/
}

//sends the value to the NSP. If this operation failed, the red led will blink 3 times.
void signalSendResult(bool value)
{
	if(_ledsEnabled){						//when leds are disabled, don't do the entire routine. it just takes up time.
		BlueLedOff();
		if(value == false){					//if we failed to send the message, indicate with led.
			for(int i = 0; i < 5; i++){
				delay(200);
				RedLedOn();
				delay(200);
				RedLedOff();
			}
		}
		else{
			for(int i = 0; i < 5; i++){
				delay(200);
				GreenLedOn();
				delay(200);
				GreenLedOff();
			}
		}
	}
}


// wire communication

#define ACCEL_ADR 0b0011110

uint8_t writeReg(uint8_t reg, uint8_t val)
{
	Wire.beginTransmission(ACCEL_ADR);
	Wire.write(reg);
	Wire.write(val);
	Wire.endTransmission();
	delayMicroseconds(10000);
}

uint8_t readReg(uint8_t reg)
{
	Wire.beginTransmission(ACCEL_ADR);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(ACCEL_ADR, 0x01);

	uint8_t val = Wire.read();
	Wire.endTransmission();

	return val;
}

//board

volatile bool _reportMovement = false;
volatile bool _reportMagneto = false;

void onAccelInterrupt()
{
	_reportMovement = true;
}

void onMagnetoInterrupt()
{
	_reportMagneto = true;
}

void initAcceleroInterrupts()
{
	pinMode(ACCEL_INT1, INPUT_PULLUP);
	pinMode(ACCEL_INT2, INPUT_PULLUP);
	attach(ACCEL_INT1, onAccelInterrupt, FALLING);
	attach(ACCEL_INT2, onAccelInterrupt, FALLING);
}

void attach(int pin, voidFuncPtr callback, int mode)
{
	attachInterrupt(pin, callback, mode);
	//has to be set after first call to attachinterrupt
	SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;						// Set the XOSC32K to run in standby
	//possibly that we have to do |= in the next statement, to keep the settings done by the rtczero timer.
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;// Configure EIC to use GCLK1 which uses XOSC32K 
}

//returns true if the accelerometer reported movement
//resets the switch so that new movement can be detected on the next run.
bool hasMoved()
{
	if (_reportMovement) {
		_reportMovement = false;
		return true;
	}
	return false;
}

//returns true if the accelerometer reported a change in the magnetic field
//resets the switch so that a new change can be detected on the next run.
bool magnetoChanged()
{
	if (_reportMagneto) {
		_reportMagneto = false;
		return true;
	}
	return false;
}

void disableUSB()
{
	USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
}

void enableUSB()
{
	USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
}

//put the device is a deep sleep mode.
void sleep()
{
	disableUSB();
	__WFI();							//Enter sleep mode
	//...Sleep forever or until interupt has arrived.
	enableUSB();
	delay(180);							//after waking up, give the device a little bit of time to wake up, if we don't do this, the serial will block and the first call to SerialUSB will make the app fail.
}

//battery
volatile bool _reportBattery = false;						//flag to indicate if we need to report battery stats or not.
unsigned long _batteryLastReportedAt = 0;					//keeps track of the last time that we send a battery update. In case we missed a hartbeat.

//callback function for the external clock to indicate that it's time to report the battery status.
void onReportBattery()
{
	_reportBattery = true;
}

void startReportingBattery(RTCZero &rtc)
{
	_reportBattery = false;

	rtc.setAlarmSeconds(BATTERY_REPORT_SEC);						// Schedule the wakeup interrupt
	rtc.setAlarmMinutes(BATTERY_REPORT_MIN);
	rtc.setAlarmHours(BATTERY_REPORT_HOUR);
	rtc.enableAlarm(BATTERY_REPORT_EVERY);			//MATCH_HHMMSS		
	rtc.attachInterrupt(onReportBattery);			// Attach handler so that we can set the battery flag when the time has passed.
	rtc.setEpoch(0);								// This sets it to 2000-01-01
	SerialUSB.println("started reporting battery status");
}

void tryReportBattery(MicrochipLoRaModem &modem, ATTDevice &device)
{
	if (_reportBattery || _batteryLastReportedAt + SENDBATTERYEVERY <= millis())	//check if the battery status needs to be reported (flag set by the interrupt).
	{
		_reportBattery = false;							//switch the flag back off, so that we can report the battery status next run as well
		reportBatteryStatus(modem, device);
	}
}

void reportBatteryStatus(MicrochipLoRaModem &modem, ATTDevice &device)
{
	unsigned long curTime = millis();
	modem.WakeUp();
	SerialUSB.println("reporting battery status");
	uint16_t batteryVoltage = analogRead(BATVOLTPIN);
	uint16_t battery = (ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)batteryVoltage;
	battery = (battery - 3000) / 10;
	if (battery > 255)
		battery = 100;
	else
		battery = (100.0 / 255.0) * (uint8_t)battery;
	SerialUSB.print("level: ");  SerialUSB.println(battery);
	
	
	uint16_t voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)batteryVoltage);
    voltage = (voltage - 3000) / 10;

    SerialUSB.print("level 2: "); SerialUSB.println(voltage > 255 ? 255 : (uint8_t)voltage);
	
	
	
	signalSendStart();
	bool sendResult = device.Send((short)battery, BATTERY_LEVEL);
	signalSendResult(sendResult);
	if(sendResult)
		_batteryLastReportedAt = curTime;
		
	modem.Sleep();
}

