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
 
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 **/

#include <ATT_LoRa_IOT.h>
#include <MicrochipLoRaModem.h>
#include <Wire.h>
#include <ATT_LoRaOne_LSM303.h>
#include <ATT_LoRaOne_RTCZero.h>
#include <ATT_LoRaOne_UBlox_GPS.h>
#include <ATT_LoRaOne_common.h>

#include "keys.h"

//#define DEBUG                                 //put this line in comment to turn on deep sleep mode. When debug is defined, the device will not go into deep sleep mode and the serial monitor will remain available. 
#define NOGPS                                   //put this line in comment if you  want to send gps coordinates (real usage), to block sending gps coordinates for indoor testing, remove commment



#define ACCELERO_SENSITIVY 4                    //sensitivity of the accelerometer when in deep sleep mode (waiting for first movment), the closer to 0, the more sensitive. Each point represent 16 milli g force.
#define MAX_MOVEMENT_TOLERANCE 400              //sensitivity of the accelerometer while in motion (to verify if the device is still moving), the closer to 0, the more sensitive. expressed in absolute accelero values
#define WAKEUP_EVERY_SEC 30                     //seconds part of the clock that wakes up the device while moving, to verity if the device is still moving.
#define WAKEUP_EVERY_MIN 0                      //minutes part of the clock that wakes up the device while moving, to verity if the device is still moving.

#define ACCELERO_NR_CALIBRATION_STEPS 20        // the nr of iterations used during calibration of the device.
#define MOVEMENT_NR_OF_SAMPLES 10               // the number of samples taken from the accelero in order to determine if the device is moving or not. (there can only be 2 measurements with diffeernt g forces)

#define GPS_FIX_TIMEOUT 900L                    //nr of seconds after which the GPS will time out and no GPS position will be ssent.
#define RESEND_DELAY 15                         //nr of seconds to wait before retrying to send a state value (moving or not)
#define MAX_SEND_RETRIES 5                      //the maximum nr of times the system will retry to send a state update/gps coordinate.  When state is true and it fails more times, the system will still try to resend it at least every 24 hours.



MicrochipLoRaModem Modem(&Serial1, &SerialUSB);
ATTDevice Device(&Modem, &SerialUSB);

LSM303 compass;
bool _wasMoving;                                //keeps track of the moving state. If this is true, the device was moving when last checked.
bool _resendWasMoving = false;                  //when this flag is true, then the system needs to try and resend the fact that the device has moved -> it failed, but this is an important state to send.

RTCZero rtc;
volatile bool wakeFromTimer = false;


#define ACCEL_ADR 0b0011110                     //ACCELERO ADDRESS, DO NOT CHANGE.
#define ACCELERO_COMPENSATION 0.061             // used to convert acceleration to g, do not change


//calibrate without giving info to user through leds.
void quickCalibrate(int16_t &x, int16_t &y)
{
    x = 0;
    y = 0;
    for (int i = 1; i <= ACCELERO_NR_CALIBRATION_STEPS; i++)
    {
        compass.read();

        SerialUSB.print("x: "); SerialUSB.print(compass.a.x * ACCELERO_COMPENSATION);
        SerialUSB.print(", y: "); SerialUSB.print(compass.a.y * ACCELERO_COMPENSATION);
        SerialUSB.print(", z: "); SerialUSB.println(compass.a.z * ACCELERO_COMPENSATION);

        if (abs(compass.a.x) > x) {
            x = abs(compass.a.x);
            SerialUSB.print("max x: "); SerialUSB.println(x * ACCELERO_COMPENSATION);
        }
        if (abs(compass.a.y) > y) {
            y = abs(compass.a.y);
            SerialUSB.print("max y: "); SerialUSB.println(y * ACCELERO_COMPENSATION);
        }
        delay(100);
    }
}


//read the compass for a nr of seconds, first allow it to stabelize (user might want to remove a ladder first)
//after stabelized, take an average of the values and store as base line. 
//let the user know the state through the led:
//     - blue: waiting to start calibration
//     - red: calibrating
//     - green: calibrated.  after 1 sec, the light goes out.
void calibrate(int16_t &x, int16_t &y)
{
    informStartOfCalibration();
    quickCalibrate(x, y);
    informEndOfCalibration();
} 

void setThresholds(int16_t x, int16_t y)
{
    SerialUSB.print("base x: "); SerialUSB.println(x);
    SerialUSB.print("base y: "); SerialUSB.println(y);
    x = (round((x * ACCELERO_COMPENSATION) / 16)) + ACCELERO_SENSITIVY;                 //16 mg step size at 2g accuratie
    y = (round((y * ACCELERO_COMPENSATION) / 16)) + ACCELERO_SENSITIVY;
    SerialUSB.print("x: 0x"); SerialUSB.print(x, HEX); SerialUSB.print("; 0b"); SerialUSB.println(x, BIN);
    SerialUSB.print("y: 0x"); SerialUSB.print(y, HEX); SerialUSB.print("; 0b"); SerialUSB.println(y, BIN);

    writeReg(0x32, x & 0x7F); // Threshold (gen 1)
    writeReg(0x36, y & 0x7F); // Threshold (gen 2)
}

void activateAcceleroInterupts()
{
    writeReg(0x22, 0b00100000);
    writeReg(0x23, 0b00100000);
}

void disableAcceleroInterupts()
{   
    writeReg(0x22, 0b00000000);
    writeReg(0x23, 0b00000000);
}


void prepareInterrupts(int16_t x, int16_t y)
{
    initAcceleroInterrupts();
    
    writeReg(0x1F, 0b10000000); // reboot
    writeReg(0x20, 0b01010111); // ctrl1
    writeReg(0x30, 0b10000010); // Axes mask (gen 1)
    writeReg(0x34, 0b10001000); // Axes mask (gen 2)
    setThresholds(x, y);
    writeReg(0x33, 0b00000000); // Duration (gen 1)
    writeReg(0x37, 0b00000000); // Duration (gen 2)
    activateAcceleroInterupts();
}

//gets the current location from the gps and sends it to the cloud.
//withSendDelay: when true, the function will pause (15 sec) before sending the message. The 15 sec is in total, so getting a gps fix is included in that time.
void sendGPSFix(bool withSendDelay)
{
    sodaq_gps.setDiag(SerialUSB);

    unsigned long start = millis();
    uint32_t timeout = GPS_FIX_TIMEOUT * 1000;
    SerialUSB.println(String("waiting for fix ..., timeout=") + timeout + String("ms"));
    #ifndef NOGPS
    //scan turns on the gps, 'false' turns it off again.
    if (sodaq_gps.scan(false, timeout)) {
        SerialUSB.println(String(" time to find fix: ") + (millis() - start) + String("ms"));
        if(withSendDelay){
            unsigned long delayTime = 15000 - (millis() - start);
            SerialUSB.println(String("sleep for: ") + (millis() - start) + String(" before sending to prevent lora congestion"));
            delay(delayTime);
        }
        SerialUSB.println(String(" datetime = ") + sodaq_gps.getDateTimeString());
        SerialUSB.println(String(" lat = ") + String(sodaq_gps.getLat(), 7));
        SerialUSB.println(String(" lon = ") + String(sodaq_gps.getLon(), 7));
        SerialUSB.println(String(" num sats = ") + String(sodaq_gps.getNumberOfSatellites()));

        Device.Queue((float)sodaq_gps.getLat());
        Device.Queue((float)sodaq_gps.getLon());
        Device.Queue((float)10.);
        Device.Queue((float)10.);
        signalSendResult(Device.Send(GPS));
    }
    else {
        SerialUSB.println("No Fix");
    }
    #else
    SerialUSB.println("simulating GPS coordinates");
    delay(15000);
    SerialUSB.println("sleep 15 sec before getting gps fix and sending it, so that we don't send the data to quickly for lora"); 
    Device.Queue((float)50.);
    Device.Queue((float)11.);
    Device.Queue((float)10.);
    Device.Queue((float)10.);
    signalSendResult(Device.Send(GPS));
    #endif
}

//sends the specified state value to the NSP. Also sends a GPS fix if we succeeded in sending the state.
//also signals the user about the state (red or green led are activated.)
void sendState(bool value)
{
    SerialUSB.print("sending state: "); SerialUSB.println(value);
    int8_t retryCount = 0;
    int pin = value ? LED_RED : LED_GREEN;
    digitalWrite(pin, LOW);                                 //turn the led on to signal the state.
    bool sendResult = Device.Send(value, PUSH_BUTTON);
    digitalWrite(pin, HIGH);
    while (!sendResult && retryCount < MAX_SEND_RETRIES){   //it's crucial that we report movement. Could be the sign of a stolen object. so block until we managed to send it out.
        retryCount++;
        signalSendResult(false);
        delay((RESEND_DELAY * 1000) - 2000);                //we do 2000 cause that's how long the signalSendResult takes.
        sendResult = Device.Send(value, PUSH_BUTTON);
    }
    if(sendResult){
        _resendWasMoving = false;                           //don't need to resend the same value cause we were succesful in sending it out.
        sendGPSFix(true);
    }
    else if (value == true){
        _resendWasMoving = true;                            //the fact that the device is moving is very important, it could be stolen, so if we failed to send the message, retry later on.
        SerialUSB.println("retrying to send value 'true' on next run"); 
    }
}
 
//stores and sends the 'movement value to the cloud + also sends the gps coordinates.
void setState(bool value)
{
    _wasMoving = value;
    SerialUSB.print("curr moving state: "); SerialUSB.println(value);
    sendState(value);
} 
 
void setup()
{
    SerialUSB.begin(57600);
    
    sodaq_gps.init();                                       //do this as early as possible, so we have a workign gps.
    while(!SerialUSB){}
    SerialUSB.println("start");
    initPower();
    initLeds();
    setPower(HIGH);                                         //turn board on
    resetAllDigitalPins();

    // Note: It is more power efficient to leave Serial1 running
    Serial1.begin(Modem.getDefaultBaudRate());              // init the baud rate of the serial connection so that it's ok for the modem
    while (!Device.Connect(DEV_ADDR, APPSKEY, NWKSKEY));
    SerialUSB.println("Ready to send data");

    Wire.begin();
    compass.init(LSM303::device_D);
    compass.enableDefault();
    int16_t x, y = 0;                                       //the current x, y, z acceleration values (for gravity compensation)
    calibrate(x, y);
    prepareInterrupts(x, y);
    setState(false);
    delay(15000);                                           //make certain that we don't over-user the lora network.
    reportBatteryStatus(Device);                            //send the current battery status at startup to report init state.
    startReportingBattery(rtc);
    
    
    // Put LoRa to sleep of 30s
    //Serial1.println("sys sleep 30000");
    //delay(100);
}

void onSleepDone() 
{
    wakeFromTimer = true;
}

//set up an interupt to wake up the device in 5 minutes time
void setWakeUpClock()
{
    rtc.setAlarmSeconds(WAKEUP_EVERY_SEC);                      // Schedule the wakeup interrupt
    rtc.setAlarmMinutes(WAKEUP_EVERY_MIN);
    rtc.enableAlarm(RTCZero::MATCH_MMSS);                       // MATCH_SS
    rtc.attachInterrupt(onSleepDone);                           // Attach handler so that we can set the battery flag when the time has passed.
    rtc.setEpoch(0);                                            // This sets it to 2000-01-01
    SerialUSB.println("sleep for some time");
}

bool isMoving()
{
    int16_t _prevX;
    int16_t _prevY;
    int16_t _prevZ;

    compass.read();
    _prevX = compass.a.x;                                       // need to establish a base line for detecting movement.
    _prevY = compass.a.y;
    _prevZ = compass.a.z;
        
    int8_t count = 0;
    for(int i = 0; i < MOVEMENT_NR_OF_SAMPLES; i++){
        delay(100);
        compass.read();
        int div = abs(_prevX - compass.a.x) + abs(_prevY - compass.a.y) + abs(_prevZ - compass.a.z);
        _prevX = compass.a.x;                                   // need to establish a base line for detecting movement.
        _prevY = compass.a.y;
        _prevZ = compass.a.z;
        SerialUSB.print("accelero dif: "); SerialUSB.println(div);
        count += div > MAX_MOVEMENT_TOLERANCE? 1: 0;
    }
    return count > 2;
}


void loop()
{
    tryReportBattery(Device); 
    if(_resendWasMoving)                                        //if the device started moving and we failed to send this to the cloud, then retry to send this value untill succesfull. The device might have been locked, if we don't send out this value (and the gps coordinates), then there is no alert triggered.
        sendState(true);
    if (wakeFromTimer == true) {                                //we got woken up by the timer, so check if still moving.  
        wakeFromTimer = false;
        int16_t x, y = 0;                                       //the current x, y, z acceleration values (for gravity compensation)
        if (isMoving() == false) {                              //no more movement: report movement has stopped + start reporting battery again  + enable accelero interrupts again.
            SerialUSB.println("movement stopped");
            setState(false);
            startReportingBattery(rtc);                         //this resets the timer routine, so no more wakeup every x minutes.
            quickCalibrate(x, y);           
            setThresholds(x, y);
            activateAcceleroInterupts();
        }
        else{
            SerialUSB.println("movement continues");
            sendGPSFix(false);
            setWakeUpClock();
        }
    }
    else if(hasMoved()){
        SerialUSB.println("start of movement detected");
        disableAcceleroInterupts();
        setState(true);
        setWakeUpClock();
    }
    #ifndef DEBUG
    SerialUSB.println("going to sleep");
    sleep();
    #endif
}
