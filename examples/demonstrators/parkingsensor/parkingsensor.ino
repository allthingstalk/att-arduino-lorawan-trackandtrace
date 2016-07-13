/****
 *  Copyright (c) 2016 AllThingsTalk. All rights reserved.
 *
 *  AllThingsTalk Developer Cloud IoT demonstrator for LoRaOne
 *  Version 1.0 dd 12/6/2016
 *  Original author: Jan Bogaerts 2016
 *
 *  This sketch is part of the AllThingsTalk LoRaOne productk development kit
 *  -> http://www.allthingstalk.com/lora-rapid-development-kit
 *
 *  For more information, please check our documentation
 *  -> http://docs.smartliving.io/kits/lora
 *
 **/
 
//#define DEBUG                             //put the following line in comment to turn on deep sleep mode. When debug is defined, the device will not go into deep sleep mode and the serial monitor will remain available.

#define MAGNETO_NR_CALIBRATION_STEPS 20     // the nr of iterations used during calibration of the device.
#define MAGNETO_INTERRUPT_SENSITIVITY 70    // (max 15 bits) the higher the number, the less sensitive the sensor is (the bigger the change in magnetic field has to be)
#define WAKEUP_EVERY_SEC 30                 //how often the timer wakes up to check if the car has left. 
 
#include <ATT_LoRa_IOT.h>
#include <MicrochipLoRaModem.h>
#include <Wire.h>
#include <ATT_LoRaOne_RTCZero.h>
#include <ATT_LoRaOne_common.h>

#include "keys.h"

MicrochipLoRaModem Modem(&Serial1, &SerialUSB);
ATTDevice Device(&Modem, &SerialUSB);
RTCZero rtc;

bool _isEmpty = true;                           //when true -> there is no car parked.                  
int16_t _baseX, _baseY, _baseZ;                 //base line that identifies an empty parking space
volatile bool wakeFromTimer = false;

void readMagneto(int16_t &x, int16_t &y, int16_t &z)
{
    x = (readReg(0x09) << 8) | readReg(0x08);
    y = (readReg(0x0B) << 8) | readReg(0x0A);
    z = (readReg(0x0D) << 8) | readReg(0x0C);
}


//read the compass for a nr of seconds, first allow it to stabelize (user might want to remove a ladder first)
//after stabelized, take an average of the values and store as base line that identifies an empty parking space. 
//let the user know the state through the led:
//     - blue: waiting to start calibration, time to remove the ladder
//     - red: calibrating
//     - green: calibrated.  after 1 sec, the light goes out.
uint16_t calibrateEmptyParking()
{   
    informStartOfCalibration();
    float x, y, z = 0;
    uint16_t maxVal = 0;
    int16_t x_val, y_val, z_val;
    for (int i = 1; i <= MAGNETO_NR_CALIBRATION_STEPS; i++)
    {
        readMagneto(x_val, y_val, z_val);
        SerialUSB.print("x: "); SerialUSB.print(x_val); SerialUSB.print("y: "); SerialUSB.print(y_val);  SerialUSB.print("z: "); SerialUSB.println(z_val);
        x = x - (x / i) + ((float)x_val / i);
        y = y - (y / i) + ((float)y_val / i);
        z = z - (z / i) + ((float)z_val / i);
        SerialUSB.print("avg x: "); SerialUSB.print(x); SerialUSB.print("avg y: "); SerialUSB.print(y);  SerialUSB.print("avg z: "); SerialUSB.println(z);
        uint16_t curMax = max(max(x_val, y_val), z_val);
        if(curMax > maxVal){
            maxVal = curMax;
            SerialUSB.print("new max: "); SerialUSB.println(maxVal); 
        }
        delay(100);
    }
    _baseX = (int16_t)x;
    _baseY = (int16_t)y;
    _baseZ = (int16_t)z;
    informEndOfCalibration();
    return maxVal;
}

uint16_t reCalibrate()
{
    int16_t x_val, y_val, z_val;
    uint16_t maxVal = 0;
    for (int i = 1; i <= MAGNETO_NR_CALIBRATION_STEPS; i++)
    {
        readMagneto(x_val, y_val, z_val);
        SerialUSB.print("x: "); SerialUSB.print(x_val); SerialUSB.print("y: "); SerialUSB.print(y_val);  SerialUSB.print("z: "); SerialUSB.println(z_val);
        uint16_t curMax = max(max(x_val, y_val), z_val);
        if(curMax > maxVal){
            maxVal = curMax;
            SerialUSB.print("new max: "); SerialUSB.println(maxVal); 
        }
        delay(100);
    }
    return maxVal;
}

//sets up the mangeto sensor so that it will produce an interrupt at the desired threshold
void prepareMagneto(uint16_t maxMagneto)
{
    uint16_t threshold;
    SerialUSB.println("preparing to detect change from empty to full parking space.");
    writeReg(0x12, 0b11100001); // Axes mask
    threshold = (maxMagneto + MAGNETO_INTERRUPT_SENSITIVITY) & ~(1 << 15);
    
    writeReg(0x14, (byte) threshold & 0x00FF);
    writeReg(0x15, (byte)(threshold >> 8));
    
    writeReg(0x22, 0b00001000);                         //start interrupt 
    writeReg(0x23, 0b00000000);
}

void stopMagnetoIntertupt()
{
    writeReg(0x22, 0b00000000);
}

void setup() 
{
	SerialUSB.begin(57600);
    #ifdef DEBUG											//when debugging is selected, we need the serialUSB. When in real-world use mode, this would make the application stuck.
    while(!SerialUSB){}
	#endif
    SerialUSB.println("start");

    initPower();
    initLeds();
    setPower(HIGH);                                     //turn board on
    resetAllDigitalPins();
    setGPSPower(LOW);                                   // Disable GPS
        
    Serial1.begin(Modem.getDefaultBaudRate());          // init the baud rate of the serial connection so that it's ok for the modem. It is more power efficient to leave Serial1 running
    while (!Device.Connect(DEV_ADDR, APPSKEY, NWKSKEY));
    SerialUSB.println("Ready to send data");    
        
    Wire.begin();
    
    initAcceleroInterrupts();
    writeReg(0x1F, 0b10000000);                         //reboot
    writeReg(0x24, 0b11110000);                         // CTRL5: Enable Temperature, High Resolution, 50hz, no interrupts 
    writeReg(0x25, 0b01100000);                         //CTRL6:  +/- 12 gauss
    writeReg(0x26, 0b00000000);                         // CTRL7: Magnetic sensor in Continous-conversion mode
    
    prepareMagneto(calibrateEmptyParking());
	Modem.WakeUp();
    signalSendResult(Device.Send(true, BINARY_SENSOR)); // send state to cloud + inform user if the send failed.
	Modem.Sleep();
    delay(15000);                                       //make certain that we don't over-user the lora network.
    reportBatteryStatus(Modem, Device);                 //send the current battery status at startup to report init state.
    startReportingBattery(rtc);
}

void onSleepDone() 
{
    wakeFromTimer = true;
}

//set up an interupt to wake up the device in 5 minutes time
void setWakeUpClock()
{
    rtc.setAlarmSeconds(WAKEUP_EVERY_SEC);                      // Schedule the wakeup interrupt
    rtc.enableAlarm(RTCZero::MATCH_SS);
    rtc.attachInterrupt(onSleepDone);                           // Attach handler so that we can set the battery flag when the time has passed.
    rtc.setEpoch(0);                                            // This sets it to 2000-01-01
    SerialUSB.println("sleep for 30 sec");
}

//checks if the parkingspace is currently empty or not.
bool checkEmptyParkingSpace()
{
    //todo: make certain that accelerometer/magnetometer is turned on.
    int16_t x_val, y_val, z_val;
    readMagneto(x_val, y_val, z_val);
    if(x_val > _baseX + MAGNETO_INTERRUPT_SENSITIVITY || x_val < _baseX - MAGNETO_INTERRUPT_SENSITIVITY){
        SerialUSB.println("x out of range: parking space still occupied");
        return false;
    }
    if(y_val > _baseY + MAGNETO_INTERRUPT_SENSITIVITY || y_val < _baseY - MAGNETO_INTERRUPT_SENSITIVITY){
        SerialUSB.println("y out of range: parking space still occupied");
        return false;
    }
    if(z_val > _baseZ + MAGNETO_INTERRUPT_SENSITIVITY || z_val < _baseZ - MAGNETO_INTERRUPT_SENSITIVITY){
        SerialUSB.println("z out of range: parking space still occupied");
        return false;
    }
    return true;
}


//store the state of the parking space, let the user know the state by means of the led and send it to the NSP over lora. 
//Signal the user if send failed.
void updateParkingSpace(bool value)
{
	Modem.WakeUp();
    _isEmpty = value;
    int pin = value ? LED_GREEN : LED_RED;
    digitalWrite(pin, LOW);
    bool sendResult = Device.Send(_isEmpty, BINARY_SENSOR);
    digitalWrite(pin, HIGH);
    signalSendResult(sendResult);
	Modem.Sleep();
}

void loop()
{
    tryReportBattery(Modem, Device);
    if (wakeFromTimer == true) {                    //we got woken up by the timer, so check if car left.
        wakeFromTimer = false;
        if(checkEmptyParkingSpace()){
            SerialUSB.println("parking space became free");
            updateParkingSpace(true);
            startReportingBattery(rtc);             //need to stop the wakeup clock and activate the clock again for reporting battery status.
            prepareMagneto(reCalibrate());          //do this first, so that the accelero stops producing interrupts
        }
        else
            setWakeUpClock();
    }
    if(hasMoved()){
        SerialUSB.println("parking space became occupied");
        updateParkingSpace(false);
        stopMagnetoIntertupt();
        //todo: shutdown mangeto meter while not used to trigger anything?
        setWakeUpClock();
    }
    //have to read reg 0x13 on every run, otherwise the interrupt wont trigger.
    int16_t x_val, y_val, z_val;
    readMagneto(x_val, y_val, z_val);
    SerialUSB.println(String("Magnetometer Readings: ") + x_val + ", " + y_val + ", " + z_val);
    readReg(0x13);

    #ifndef DEBUG
    SerialUSB.println("going to sleep");
    sleep();
    #else
    delay(500);
    #endif
}
