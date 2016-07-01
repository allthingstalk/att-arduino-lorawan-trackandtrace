

#include "AppBase.h"
#include "BoardFeatures.h"
#include "BoardFeatures.h"

//These constants are used for reading the battery voltage
#define ADC_AREF 3.3
#define BATVOLTPIN A5 
#define BATVOLT_R1 4.7
#define BATVOLT_R2 10

#define INT1 4

void AppBase::informStartOfCalibration()
{
	//let user know that the procedure will start
	SerialUSB.println("begin calibration");
	BoardFeatures.BlueLedOn();
	delay(4000);
	BoardFeatures.BlueLedOff();
	BoardFeatures.RedLedOn();
}
void AppBase::informEndOfCalibration()
{
	//let user know that the procedure is done.
	SerialUSB.println("end calibration");
	BoardFeatures.RedLedOff();
	BoardFeatures.GreenLedOn();
	delay(2000);
	BoardFeatures.GreenLedOff();
}

volatile bool _reportBattery = false;
volatile bool _reportMovement = false;
volatile bool _reportMagneto = false;

//callback function for the external clock to indicate that it's time to report the battery status.
void onReportBattery()
{
	_reportBattery = true;
}

void onAccelInterrupt()
{
	_reportMovement = true;
}

void onMagnetoInterrupt()
{
	_reportMagneto = true;
}

void AppBase::startReportingBattery()
{
	_reportBattery = false;

	rtc.begin();
	rtc.setAlarmSeconds(59);						// Schedule the wakeup interrupt
	rtc.setAlarmMinutes(3);
	rtc.enableAlarm(RTCZero::MATCH_SS);			
	rtc.attachInterrupt(onReportBattery);			// Attach handler so that we can set the battery flag when the time has passed.
	rtc.setEpoch(0);								// This sets it to 2000-01-01
	SerialUSB.println("started reporting battery status");
}

void AppBase::initAcceleroInterupts(LSM303& compass)
{
	pinMode(ACCEL_INT1, INPUT_PULLUP);
	pinMode(ACCEL_INT2, INPUT_PULLUP);
	attachInterrupt(ACCEL_INT1, onAccelInterrupt, FALLING);
	attachInterrupt(ACCEL_INT2, onAccelInterrupt, FALLING);
}

void AppBase::initMagnetoInterupts(LSM303& compass)
{
	pinMode(ACCEL_INT1, INPUT_PULLUP);
	pinMode(ACCEL_INT2, INPUT_PULLUP);
	attachInterrupt(ACCEL_INT1, onMagnetoInterrupt, FALLING);
	attachInterrupt(ACCEL_INT2, onMagnetoInterrupt, FALLING);
}


////see: http://www.st.com/content/ccc/resource/technical/document/datasheet/1c/9e/71/05/4e/b7/4d/d1/DM00057547.pdf/files/DM00057547.pdf/jcr:content/translations/en.DM00057547.pdf
void AppBase::startReportingMovement(LSM303& compass)
{
	_reportMovement = false;
	compass.writeReg(0x1F, 0b10000000);// Reboot: CTRL0
	compass.writeReg(LSM303::CTRL1, 0b01010111); //(0x20)// Set to 50z all axes active: CTRL1
	// Interrupt source 1
	compass.writeReg(LSM303::IG_CFG1, 0b10000010); //(0x30) Axes mask
	compass.writeReg(LSM303::IG_THS1, 0b00000011); //(0x32) Threshold
	compass.writeReg(LSM303::IG_DUR1, 0b00000000); //(0x33) Duration
	// Interrupt source 2
	compass.writeReg(LSM303::IG_CFG2, 0b10100000); // (0x34)Axes mask
	compass.writeReg(LSM303::IG_THS2, 0b00000101); // (0x36)Threshold
	compass.writeReg(LSM303::IG_DUR2, 0b00000000); // (0x37)Duration
	//setup sources
	compass.writeReg(LSM303::CTRL3, 0b00100000);// (0x22)CTRL3 (INT1 or int2 depnds on board), INT1 sources from IG_SRC1
	compass.writeReg(LSM303::CTRL4, 0b00100000);// (0x23)CTRL4 (INT2 or int1 depnds on board): INT2 sources from IG_SRC2
}

void AppBase::startReportingMagnetoChange(LSM303& compass)
{
	_reportMagneto = false;
	//compass.writeReg(0x1F, 0b10000000);// Reboot: CTRL0
	compass.writeReg(0x20, 0b01010111);// Set to 50z all axes active: CTRL1
									   //setup sources
	compass.writeReg(0x22, 0b00001000);
	compass.writeReg(0x23, 0b00001000);
									   // Interrupt source 1
	compass.writeReg(0x30, 0b10001000); // Axes mask
	compass.writeReg(0x32, 0b00111111); // Threshold
	compass.writeReg(0x33, 0b00000001); // Duration
										// Interrupt source 2
	compass.writeReg(0x34, 0b10000010); // Axes mask
	compass.writeReg(0x36, 0b00111111); // Threshold
	compass.writeReg(0x37, 0b00000001); // Duration
}

//returns true if the accelerometer reported movement
//resets the switch so that new movement can be detected on the next run.
bool AppBase::hasMoved()
{
	if (_reportMovement) {
		_reportMovement = false;
		return true;
	}
	return false;
}

//returns true if the accelerometer reported a change in the magnetic field
//resets the switch so that a new change can be detected on the next run.
bool AppBase::magnetoChanged()
{
	if (_reportMagneto) {
		_reportMagneto = false;
		return true;
	}
	return false;
}

void AppBase::stopAcceleroInterupt(LSM303& compass)
{
	compass.writeReg(0x1F, 0b10000000);// Reboot: CTRL0
	compass.writeReg(0x20, 0b01010111);// Set to 50z all axes active: CTRL1
									   //setup sources
	compass.writeReg(0x22, 0b00000000);// CTRL3 (INT1 or int2 depnds on board), INT1 sources from IG_SRC1
	compass.writeReg(0x23, 0b00000000);// CTRL4 (INT2 or int1 depnds on board): INT2 sources from IG_SRC2
}

void AppBase::stopMagnetoInterupt(LSM303& compass)
{
	stopAcceleroInterupt(compass);
}

//void AppBase::

void AppBase::loop()
{
	if (_reportBattery)									//check if the battery status needs to be reported (flag set by the interrupt).
	{
		SerialUSB.println("reporting battery status");
		_reportBattery = false;							//switch the flag back off, so that we can report the battery status next run as well
		uint16_t batteryVoltage = analogRead(BATVOLTPIN);
		uint16_t battery = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)batteryVoltage);
		battery = (battery - 3000) / 10;
		if (battery > 255)
			battery = 100;
		else
			battery = (uint16_t)((100.0 / 255) * battery);
		SerialUSB.print("level: ");  SerialUSB.println(battery);
		_device->Send((short)battery, BATTERY_LEVEL);
	}
}
