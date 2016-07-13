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

//#define DEBUG                                 //put the following line in comment to turn on deep sleep mode. When debug is defined, the device will not go into deep sleep mode and the serial monitor will remain available.

#define ACCELERO_NR_CALIBRATION_STEPS 20        //nr of samples used to calibrate the accelero interrupt and gate position.
#define ACCELERO_SENSITIVY 2                    //the higher, the less sensitive, closer to 0 = most sensitive.
#define HEADING_TOLERANCE 3                     //nr of degrees tolerance that the algoritme uses to determine if the gate is closed or not.

 
#include <ATT_LoRa_IOT.h>
#include <MicrochipLoRaModem.h>
#include <Wire.h>
#include <ATT_LoRaOne_LSM303.h>
#include <ATT_LoRaOne_RTCZero.h>
#include <ATT_LoRaOne_common.h>

#include "keys.h"

MicrochipLoRaModem Modem(&Serial1, &SerialUSB);
ATTDevice Device(&Modem, &SerialUSB);

LSM303 compass;
bool _isOpen = false; 
float _closedHeading = 0;

RTCZero rtc;
volatile bool wakeFromTimer = false;

#define ACCELERO_COMPENSATION 0.061 


//calibrate without giving info to user through leds.
//the calibration process calculates the heading when the gate is closed and finds the current value for the x and z
//axis so that we can calculate the threshold.
void quickCalibrate(int16_t &x, int16_t &z)
{
    x = 0;
    z = 0;
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
        if (abs(compass.a.z) > z) {
            z = abs(compass.a.z);
            SerialUSB.print("max z: "); SerialUSB.println(z * ACCELERO_COMPENSATION);
        }
        float curHeading = compass.heading();
        SerialUSB.print("heading: "); SerialUSB.println(curHeading);
        _closedHeading = _closedHeading - (_closedHeading / i) + ((float)curHeading / i);   //take the (running) average of the heading when closed, so we can check.
        delay(100);
    }
    SerialUSB.print("avg heading: "); SerialUSB.println(_closedHeading);
}

//use the leds to inform the user while calibrating the device.
void calibrate(int16_t &x, int16_t &z)
{
    //read the compass for a nr of seconds, first allow it to stabelize (user might want to remove a ladder first)
    //after stabelized, take an average of the values and store as base line. 
    //let the user know the state through the led:
    //     - blue: waiting to start calibration, time to remove the ladder
    //     - red: calibrating
    //     - green: calibrated.  after 1 sec, the light goes out.
    informStartOfCalibration();
    quickCalibrate(x, z);
    informEndOfCalibration();
} 


//turn accelero interrupts on
void activateAcceleroInterupts()
{
    writeReg(0x22, 0b00100000);
    writeReg(0x23, 0b00100000);
}

//assign the threshold to the accelero interrupt
void setThresholds(int16_t x, int16_t z)
{
    SerialUSB.print("base x: "); SerialUSB.println(x);
    SerialUSB.print("base z: "); SerialUSB.println(z);
    x = (round((x * ACCELERO_COMPENSATION) / 16)) + ACCELERO_SENSITIVY;     //16 mg step size at 2g accuratie
    z = (round((z * ACCELERO_COMPENSATION) / 16)) + ACCELERO_SENSITIVY;
    SerialUSB.print("x: 0x"); SerialUSB.print(x, HEX); SerialUSB.print("; 0b"); SerialUSB.println(x, BIN);
    SerialUSB.print("z: 0x"); SerialUSB.print(z, HEX); SerialUSB.print("; 0b"); SerialUSB.println(z, BIN);

    writeReg(0x32, x & 0x7F);                                               // Threshold (gen 1)
    writeReg(0x36, z & 0x7F);                                               // Threshold (gen 2)
}

//do everything required to get interrupts from the accelerometer
void prepareInterrupts(int16_t x, int16_t z)
{
    initAcceleroInterrupts();
    
    writeReg(0x1F, 0b10000000); // reboot
    writeReg(0x20, 0b01010111); // ctrl1
    writeReg(0x30, 0b10000010); // Axes mask (gen 1)
    writeReg(0x34, 0b10100000); // Axes mask (gen 2)
    setThresholds(x, z);
    writeReg(0x33, 0b00000000); // Duration (gen 1)
    writeReg(0x37, 0b00000000); // Duration (gen 2)
    activateAcceleroInterupts();
}

void setup()
{
    SerialUSB.begin(57600);
    #ifdef DEBUG											//when debugging is selected, we need the serialUSB. When in real-world use mode, this would make the application stuck.
    while(!SerialUSB){}
	SerialUSB.println("start");
	#endif
    initPower();
    initLeds();
    setPower(HIGH);                             		//turn board on
    resetAllDigitalPins();
	setGPSPower(LOW);                                   // Disable GPS
    
    Serial1.begin(Modem.getDefaultBaudRate());  // init the baud rate of the serial connection so that it's ok for the modem. It is more power efficient to leave Serial1 running
    while (!Device.Connect(DEV_ADDR, APPSKEY, NWKSKEY));
    SerialUSB.println("Ready to send data");

    Wire.begin();
    compass.init(LSM303::device_D);
    writeReg(0x20, 0b01010111);                 // ctrl1
    writeReg(0x24, 0b11110000);                 // CTRL5: Enable Temperature, High Resolution, 50hz, no interrupts 
    writeReg(0x25, 0b01100000);                 // CTRL6:  +/- 12 gauss
    writeReg(0x26, 0b00000000);                 // CTRL7: Magnetic sensor in Continous-conversion mode
    
    compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
    compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
    //todo: should still calibrate to improve m_min and m_max, but for simply detecting open/close, this should do.
  
    int16_t x, y = 0;                           //the current x, y, z acceleration values (for gravity compensation)
    calibrate(x, y);
    prepareInterrupts(x, y);
    signalSendResult(Device.Send(false, BINARY_SENSOR));
    delay(15000);                                           //make certain that we don't over-user the lora network.
    reportBatteryStatus(Modem, Device);                            //send the current battery status at startup to report init state.
    startReportingBattery(rtc);
}

//checks if the gate is currently open or closed.
bool isGateOpen()
{
    writeReg(0x20, 0b01010111);     // ctrl1: have to reset these registers, otherwise the accelerometer doesn't report the correct data after the interrupt.
    writeReg(0x24, 0b11110000);
    writeReg(0x25, 0b01100000);
    writeReg(0x26, 0b00000000);
    compass.read();
    
    float curHeading = 0;
    for (int i = 1; i <= ACCELERO_NR_CALIBRATION_STEPS; i++)
    {
        compass.read();
        float newHeading = compass.heading();
        SerialUSB.print("heading: "); SerialUSB.println(newHeading);
        curHeading = curHeading - (curHeading / i) + ((float)newHeading / i);   //take the (running) average of the heading when closed, so we can check.
        delay(100);
    }
    SerialUSB.print("avg heading: "); SerialUSB.println(curHeading);
    return !((curHeading >= _closedHeading - HEADING_TOLERANCE) && (curHeading <= _closedHeading + HEADING_TOLERANCE));
}

//send the value to the cloud and let the user know the current state of the gate with the led.
void signalNewGatePosition(bool value)
{
    int pin = value ? LED_RED : LED_GREEN;
    digitalWrite(pin, LOW);
    bool sendResult = Device.Send(value, BINARY_SENSOR);
    digitalWrite(pin, HIGH);
    signalSendResult(sendResult);
}

void loop()
{
    #ifndef DEBUG
    SerialUSB.println("going to sleep");
    sleep();
    #endif
    tryReportBattery(Modem, Device);
    if(hasMoved()){
        bool newPos = isGateOpen();
        if(newPos == true && _isOpen == false){
            _isOpen = true;
            SerialUSB.println("gate open");
            signalNewGatePosition(true);
        }
        else if(newPos == false && _isOpen == true){
            _isOpen = false;
            SerialUSB.println("gate closed");
            signalNewGatePosition(false);
        }
    }
}
