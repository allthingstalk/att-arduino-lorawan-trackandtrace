

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

volatile bool _reportBattery;
volatile bool _reportMovement;
volatile bool _reportMagneto;

//callback function for the external clock to indicate that it's time to report the battery status.
void onReportBattery()
{
	_reportBattery = true;
	SerialUSB.println("need to report battery");
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
	attachInterrupt(ACCEL_INT1, ISR1, FALLING);
	attachInterrupt(ACCEL_INT2, ISR2, FALLING);
}


////see: http://www.st.com/content/ccc/resource/technical/document/datasheet/1c/9e/71/05/4e/b7/4d/d1/DM00057547.pdf/files/DM00057547.pdf/jcr:content/translations/en.DM00057547.pdf
void AppBase::StartReportingMovement(LSM303& compass)
{
	_reportMovement = false;
	compass.writeReg(0x1F, 0b10000000);// Reboot: CTRL0
	compass.writeReg(0x20, 0b01010111);// Set to 50z all axes active: CTRL1
	//setup sources
	compass.writeReg(0x22, 0b00100000);// CTRL3 (INT1 or int2 depnds on board), INT1 sources from IG_SRC1
	compass.writeReg(0x23, 0b00100000);// CTRL4 (INT2 or int1 depnds on board): INT2 sources from IG_SRC2
	// Interrupt source 1
	compass.writeReg(0x30, 0b10001000); // Axes mask
	compass.writeReg(0x32, 0b00111111); // Threshold
	compass.writeReg(0x33, 0b00000000); // Duration
	// Interrupt source 2
	compass.writeReg(0x34, 0b10000010); // Axes mask
	compass.writeReg(0x36, 0b00111111); // Threshold
	compass.writeReg(0x37, 0b00000000); // Duration
}

void AppBase::startReportingMagnetoChange(LSM303& compass)
{
	_reportMagneto = false;
	compass.writeReg(0x1F, 0b10000000);// Reboot: CTRL0
	compass.writeReg(0x20, 0b01010111);// Set to 50z all axes active: CTRL1
									   //setup sources
	compass.writeReg(0x22, 0b00001000);// CTRL3 (INT1 or int2 depnds on board), INT1 sources from IG_SRC1
	compass.writeReg(0x23, 0b00001000);// CTRL4 (INT2 or int1 depnds on board): INT2 sources from IG_SRC2
									   // Interrupt source 1
	compass.writeReg(0x30, 0b10001000); // Axes mask
	compass.writeReg(0x32, 0b00111111); // Threshold
	compass.writeReg(0x33, 0b00000000); // Duration
										// Interrupt source 2
	compass.writeReg(0x34, 0b10000010); // Axes mask
	compass.writeReg(0x36, 0b00111111); // Threshold
	compass.writeReg(0x37, 0b00000000); // Duration
}

//returns true if the accelerometer reported movement
//resets the switch so that new movement can be detected on the next run.
bool AppBase::hasMoved()
{
	bool res = _reportMovement;
	_reportMovement = !_reportMovement;
	return res;
}

//returns true if the accelerometer reported a change in the magnetic field
//resets the switch so that a new change can be detected on the next run.
bool AppBase::magnetoChanged()
{
	bool res = _reportMagneto;
	_reportMagneto = !_reportMagneto;
	return res;
}

void AppBase::stopAcceleroInterupt(LSM303& compass)
{
	compass.writeReg(0x1F, 0b10000000);// Reboot: CTRL0
	compass.writeReg(0x20, 0b01010111);// Set to 50z all axes active: CTRL1
									   //setup sources
	compass.writeReg(0x22, 0b00000000);// CTRL3 (INT1 or int2 depnds on board), INT1 sources from IG_SRC1
	compass.writeReg(0x23, 0b00000000);// CTRL4 (INT2 or int1 depnds on board): INT2 sources from IG_SRC2
}

//void AppBase::

void AppBase::loop()
{
	if (_reportBattery)									//check if the battery status needs to be reported (flag set by the interrupt).
	{
		_reportBattery = false;							//switch the flag back off, so that we can report the battery status next run as well
		uint16_t batteryVoltage = analogRead(BATVOLTPIN);
		uint16_t battery = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)batteryVoltage);
		battery = (battery - 3000) / 10;
		if (battery > 255)
			battery = 100;
		else
			battery = (uint16_t)((100.0 / 255) * battery);
		SerialUSB.println(battery);
		_device->Send((short)battery, BATTERY_LEVEL);
	}
}
