
#ifndef _att_common_h
#define _att_common_h

#include <ATT_LoRa_IOT.h>
#include <MicrochipLoRaModem.h>
#include "ATT_LoRaOne_rtcZero.h"

#if defined(ARDUINO) && ARDUINO >= 100
  #include "arduino.h"
#else
  #include "WProgram.h"
#endif

//////////////////////
//power

//extern bool _ledsEnabled;

void initPower();
void setGPSPower(int8_t value);
void setPower(int8_t value);
//////////////////////
//init leds. When enabled, leds will be shown, otherwise not. default ist true.
void initLeds(bool enabled = true);
void informStartOfCalibration();
void informEndOfCalibration();
void signalSendResult(bool value);
void signalSendStart();

/*inline void RedLedOn() { if(_ledsEnabled) digitalWrite(LED_RED, LOW); };
inline void RedLedOff() { if(_ledsEnabled) digitalWrite(LED_RED, HIGH); };

inline void GreenLedOn() { if(_ledsEnabled) digitalWrite(LED_GREEN, LOW); };
inline void GreenLedOff() { if(_ledsEnabled) digitalWrite(LED_GREEN, HIGH); };

inline void BlueLedOn() { if(_ledsEnabled) digitalWrite(LED_BLUE, LOW); };
inline void BlueLedOff() { if(_ledsEnabled) digitalWrite(LED_BLUE, HIGH); };
*/
void RedLedOn();
void RedLedOff();

void GreenLedOn();
void GreenLedOff();

void BlueLedOn();
void BlueLedOff();


//////////////////////
// wire communication

uint8_t readReg(uint8_t reg);
uint8_t writeReg(uint8_t reg, uint8_t val);


//////////////////////
//board

void initAcceleroInterrupts();
void attach(int pin, voidFuncPtr callback, int mode);
//returns true if the accelerometer reported a change in the magnetic field
//resets the switch so that a new change can be detected on the next run.
bool magnetoChanged();
//returns true if the accelerometer reported movement
//resets the switch so that new movement can be detected on the next run.
bool hasMoved();
//put the device is a deep sleep mode.
void sleep();
void disableUSB();
void enableUSB();

//////////////////////
//battery

//set up the timer and start reporting battery level every 24 hours
void startReportingBattery(RTCZero &rtc);
//check if it's time to report battery status, and if so, do it.
void tryReportBattery(MicrochipLoRaModem &modem, ATTDevice &device);
//calculates the current battery level and sends it the cloud.
void reportBatteryStatus(MicrochipLoRaModem &modem, ATTDevice &device);

#endif
