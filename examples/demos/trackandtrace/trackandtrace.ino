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

#include <ATT_IOT_LoRaWAN.h>
#include <MicrochipLoRaModem.h>
#include <Wire.h>
#include <ATT_LoRaWAN_LSM303.h>
#include <ATT_LoRaWAN_RTCZero.h>
#include <ATT_LoRaWAN_UBlox_GPS.h>
#include <ATT_LoRaWAN_TRACKTRACE.h>
#include "Config.h"
#include "BootMenu.h"




#define GPS_WAKEUP_EVERY_SEC 5                 //seconds part of the clock that wakes up the device for retrieving a GPS fix.    
#define GPS_WAKEUP_EVERY_MIN 0                  //minutes part of the clock that wakes up the device for retrieving a GPS fix
#define MAX_MOVEMENT_TOLERANCE 500              //sensitivity of the accelerometer while in motion (to verify if the device is still moving), the closer to 0, the more sensitive. expressed in absolute accelero values
#define ACCELERO_NR_CALIBRATION_STEPS 20        // the nr of iterations used during calibration of the device.
#define MOVEMENT_NR_OF_SAMPLES 10               // the number of samples taken from the accelero in order to determine if the device is moving or not. (there can only be 2 measurements with diffeernt g forces)

#define HEX_CHAR_TO_NIBBLE(c) ((c >= 'a') ? (c - 'a' + 0x0A) : ((c >= 'A') ? (c - 'A' + 0x0A) : (c - '0')))
#define HEX_PAIR_TO_BYTE(h, l) ((HEX_CHAR_TO_NIBBLE(h) << 4) + HEX_CHAR_TO_NIBBLE(l))

MicrochipLoRaModem Modem(&Serial1, &SerialUSB);
ATTDevice Device(&Modem, &SerialUSB);

LSM303 compass;
bool _wasMoving;                                //keeps track of the moving state. If this is true, the device was moving when last checked.
//bool _resendWasMoving = false;                  //when this flag is true, then the system needs to try and resend the fact that the device has moved -> it failed, but this is an important state to send.
bool _gpsScanning = false;                      //the gps scans for a location fix in an async way, this flag keeps track of the scan state: are we currently looking for a gps fix or not.
bool _foundGPSFix = false;						//have we found a 1st gps fix, if not, we time out longer, if we have, we can query the gps faster.

RTCZero rtc;
volatile bool wakeFromTimer = false;


#define ACCEL_ADR 0b0011110                     //ACCELERO ADDRESS, DO NOT CHANGE.
#define ACCELERO_COMPENSATION 0.061             // used to convert acceleration to g, do not change



void calibrate(int16_t &x, int16_t &y)
{
    x = 0;
    y = 0;
    SerialUSB.println("begin calibration");
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
    SerialUSB.println("end calibration");
}



void setThresholds(int16_t x, int16_t y)
{
    uint8_t sensitivity = params.getAcceleroSensitivity();
    SerialUSB.print("base x: "); SerialUSB.println(x);
    SerialUSB.print("base y: "); SerialUSB.println(y);
    x = (round((x * ACCELERO_COMPENSATION) / 16)) + sensitivity;                 //16 mg step size at 2g accuratie
    y = (round((y * ACCELERO_COMPENSATION) / 16)) + sensitivity;
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
    writeReg(0x33, 0b00010010); // Duration (gen 1)
    writeReg(0x37, 0b00010010); // Duration (gen 2)
    activateAcceleroInterupts();
}

void onSleepDone() 
{
    wakeFromTimer = true;
}

//unsigned long _nextIntervalAt = 0;								// when a wakeupclock is initialized, this value keeps track of when the next interval impuls should be given, so that we can also do	

//set up an interupt to wake up the device in 5 minutes time
void setWakeUpClock()
{
    if(_gpsScanning){
		if(_foundGPSFix){										//if we already found a fix before, we can get a new one really fast, so we use a short delay. If we havent, it will take a longer time.
			rtc.setAlarmSeconds(GPS_WAKEUP_EVERY_SEC);                      // Schedule the wakeup interrupt
			rtc.setAlarmMinutes(GPS_WAKEUP_EVERY_MIN);
			SerialUSB.print("sleep for "); SerialUSB.print(GPS_WAKEUP_EVERY_SEC);  SerialUSB.println(" seconds");
		}
		else{
			rtc.setAlarmSeconds(GPS_WAKEUP_EVERY_SEC * 6);
			rtc.setAlarmMinutes(GPS_WAKEUP_EVERY_MIN);
			SerialUSB.print("sleep for "); SerialUSB.print(GPS_WAKEUP_EVERY_SEC * 6);  SerialUSB.println(" seconds");
		}
    }else{
        rtc.setAlarmSeconds(params.getFixIntervalSeconds());                      // Schedule the wakeup interrupt
        rtc.setAlarmMinutes(params.getFixIntervalMinutes());
		SerialUSB.print("sleep for "); SerialUSB.print(params.getFixIntervalMinutes());  SerialUSB.println(" minutes");
    }
    rtc.enableAlarm(RTCZero::MATCH_MMSS);                       // MATCH_SS
    rtc.attachInterrupt(onSleepDone);                           // Attach handler so that we can set the battery flag when the time has passed.
    rtc.setEpoch(0);                                            // This sets it to 2000-01-01
}

/*
unsigned long _tempLastReportedAt = 0;				//last time that temp was sent to the cloud.

void tryReportTemp()
{
	if(_tempLastReportedAt + (params.getFixIntervalSeconds()) * 1000 <= millis()){		//we try to report temp at roughly the same rate as the timer interval.
		_tempLastReportedAt = millis();
		
		reportTemp();
		
	}
}*/

/**
* Initializes the LSM303 or puts it in power-down mode.
WARNING:::  only use this function when not using the accelero interrupts (when turning on). Off is ok
*/
void setLsm303Active(bool on)
{
    if (on) {
        if (!compass.init(LSM303::device_D, LSM303::sa0_low)) {
            SerialUSB.println("Initialization of the LSM303 failed!");
            return;
        }

        compass.enableDefault();
        compass.writeReg(LSM303::CTRL5, compass.readReg(LSM303::CTRL5) | 0b10000000);
    }
    else {
        compass.writeReg(LSM303::CTRL1, 0);			// disable accelerometer, power-down mode
        compass.writeReg(LSM303::CTRL5, 0);			// zero CTRL5 (including turn off TEMP sensor)
        compass.writeReg(LSM303::CTRL7, 0b00000010);	// disable magnetometer, power-down mode
    }
}

void reportTemp()
{
	if(params.getUseAccelero() == false){
		setLsm303Active(true);
		delay(500);										//give the accelero some time to do some meausurements.
	}

	uint8_t tempL = compass.readReg(LSM303::TEMP_OUT_L);
	uint8_t tempH = compass.readReg(LSM303::TEMP_OUT_H);
	int16_t temp = (int16_t)(((uint16_t)tempH << 8) | tempL);
	
	if(params.getUseAccelero() == false)
		setLsm303Active(false);
	
	SerialUSB.print("Device temperature = "); SerialUSB.println(temp);
	
	signalSendStart();
	signalSendResult(Device.Send((float)temp, TEMPERATURE_SENSOR));
}

//starts up the gps unit and sets a flag so that the main loop knows we need to scan the gps (async scanning).
void startGPSFix()
{
    if(_gpsScanning == false){                                  //only try to start it if the gps is already running, otherwise, we might reset the thing and never get a fix.
		if(!params.getUseAccelero())								//when using the timer to send gps fix, we can still send a second message  (without congesting the lora network)
			reportTemp();
        unsigned long start = millis();
        uint32_t timeout = params.getGpsFixTimeout() * (_foundGPSFix ? 5000 : 30000);
        SerialUSB.println(String("spinning up gps ..., timeout in ") + (timeout / 1000) + String("s"));
        sodaq_gps.startScan(start, timeout);
        _gpsScanning = true;
        setWakeUpClock();
    }
}

//stops the async scanning of the gps module
void stopGPSFix()
{
    SerialUSB.println("shutting down gps");
    sodaq_gps.endScan();
    _gpsScanning = false;
    if(params.getUseAccelero())
        startReportingBattery(rtc);                         //after we found the gps, the timer becomes free (is no longer used), so battery would no longer be checked. need to start this up again.
    else
        setWakeUpClock();
}

//gets the current location from the gps and sends it to the cloud.
//withSendDelay: when true, the function will pause (15 sec) before sending the message. The 15 sec is in total, so getting a gps fix is included in that time.
//returns true if the operation was terminated. Otherwise false
bool trySendGPSFix()
{
	SerialUSB.println("scanning gps");
    if (sodaq_gps.tryScan(10)) {
        unsigned long duration = millis() - sodaq_gps.getScanStart();
        SerialUSB.println(String(" time to find fix: ") + duration + String("ms"));
        Modem.WakeUp(); 
        stopGPSFix();
        SerialUSB.println(String(" datetime = ") + sodaq_gps.getDateTimeString());
        SerialUSB.println(String(" lat = ") + String(sodaq_gps.getLat(), 7));
        SerialUSB.println(String(" lon = ") + String(sodaq_gps.getLon(), 7));
		SerialUSB.println(String(" alt = ") + String(sodaq_gps.getAlt(), 7));
		SerialUSB.println(String(" spd = ") + String(sodaq_gps.getSpeed(), 7));
        SerialUSB.println(String(" num sats = ") + String(sodaq_gps.getNumberOfSatellites()));

        Device.Queue((float)sodaq_gps.getLat());
        Device.Queue((float)sodaq_gps.getLon());
        Device.Queue((float)sodaq_gps.getAlt());
        Device.Queue((float)10.);
        signalSendStart();
        signalSendResult(Device.Send(GPS));
		if(!params.getUseAccelero()){								//when user the timer to send gps fix, we can still send a second message  (without congesting the lora network)
			signalSendStart();
			signalSendResult(Device.Send((short)sodaq_gps.getSpeed(), INTEGER_SENSOR));
		}
        Modem.Sleep();
		_foundGPSFix = true;
		return true;
    }
    else if(sodaq_gps.scanTimedOut()){
        stopGPSFix();
        SerialUSB.println("GPS module stopped: failed to find fix. new GPS fix will be attempted upon next change in movement");
		return true;
    }
    else
        SerialUSB.println("No GPS Fix yet");
	return false;
}

//sends the specified state value to the NSP. Also sends a GPS fix if we succeeded in sending the state.
//also signals the user about the state (red or green led are activated.)
bool sendState(bool value)
{
    startGPSFix();                                      //spin up the gps module as fast as possible
    Modem.WakeUp();                                         //the modem is sleeping by default, need to wake it up to send something
    SerialUSB.print("sending state: "); SerialUSB.println(value);
    signalSendStart();
    bool result = Device.Send(value, PUSH_BUTTON);
    Modem.Sleep();                                          //when done, put the modem back to sleep to save battery power.
    signalSendResult(result);
    if(result){
        //_resendWasMoving = false;                           //don't need to resend the same value cause we were succesful in sending it out.
        trySendGPSFix();
    }
    else{ 
        stopGPSFix();                                       //something went wrong, can't send a value, so can't send gps, no need to get a fix, save battery.
//        if (value == true){
//            _resendWasMoving = true;                            //the fact that the device is moving is very important, it could be stolen, so if we failed to send the message, retry later on.
//            SerialUSB.println("retrying to send value 'true' on next run"); 
//        }
    }    
    return result;
}
 
//stores and sends the 'movement value to the cloud + also sends the gps coordinates.
bool setState(bool value)
{
    _wasMoving = value;
    SerialUSB.print("curr moving state: "); SerialUSB.println(value);
    return sendState(value);
} 
 
/**
 * Shows and handles the boot up menu if there is a serial port.
 */
void handleSerialPort()
{
    SerialUSB.begin(57600);
    if (params.keysAndAddrSpecified()){                     //main configs have been supplied, use short boot cycle.
        unsigned long start = millis();
        pinMode(BUTTON, INPUT_PULLUP);
        int sensorVal = digitalRead(BUTTON);
        while(sensorVal == HIGH && start + 5000 > millis())
            sensorVal = digitalRead(BUTTON);
        if(sensorVal == LOW){
			while(!SerialUSB){}                                 //wait until the serial port is connected, or time out if there is none.
            do {
                showBootMenu(SerialUSB);
            } while (!params.checkConfig(SerialUSB) || !params.keysAndAddrSpecified());
            params.showConfig(&SerialUSB);
        }
    }
    else{
        while(!SerialUSB){}                                 //wait until the serial port is connected, or time out if there is none.
        do {
            showBootMenu(SerialUSB);
        } while (!params.checkConfig(SerialUSB) || !params.keysAndAddrSpecified());
        params.showConfig(&SerialUSB);
    }
}

/**
 * Converts the given hex array and returns true if it is valid hex and non-zero.
 * "hex" is assumed to be 2*resultSize bytes.
 */
bool convertAndCheckHexArray(uint8_t* result, const char* hex, size_t resultSize)
{
    bool foundNonZero = false;

    uint16_t inputIndex = 0;
    uint16_t outputIndex = 0;

    // stop at the first string termination char, or if output buffer is over
    while (outputIndex < resultSize && hex[inputIndex] != 0 && hex[inputIndex + 1] != 0) {
        if (!isxdigit(hex[inputIndex]) || !isxdigit(hex[inputIndex + 1])) {
            return false;
        }

        result[outputIndex] = HEX_PAIR_TO_BYTE(hex[inputIndex], hex[inputIndex + 1]);

        if (result[outputIndex] > 0) {
            foundNonZero = true;
        }

        inputIndex += 2;
        outputIndex++;
    }

    result[outputIndex] = 0; // terminate the string

    return foundNonZero;
}

void connect()
{
	uint8_t devAddr[4];
	uint8_t appSKey[16];
	uint8_t nwkSKey[16];

	bool allParametersValid = convertAndCheckHexArray((uint8_t*)devAddr, params.getDevAddrOrEUI(), sizeof(devAddr))
		&& convertAndCheckHexArray((uint8_t*)appSKey, params.getAppSKeyOrEUI(), sizeof(appSKey))
		&& convertAndCheckHexArray((uint8_t*)nwkSKey, params.getNwSKeyOrAppKey(), sizeof(nwkSKey));
		
	if(!allParametersValid){
		SerialUSB.println("Invalid parameters for the lora connection, can't start up lora modem.");
		return;
	}
	//Device.SetMinTimeBetweenSend(150000);					//we are sending out many messages after each other, make certain that they don't get blocked by base station.
	// Note: It is more power efficient to leave Serial1 running
    Serial1.begin(Modem.getDefaultBaudRate());              // init the baud rate of the serial connection so that it's ok for the modem
    Modem.Sleep();                                          //make certain taht the modem is synced and awake.
    delay(50);
    Modem.WakeUp();
    while (!Device.Connect(devAddr, appSKey, nwkSKey));
    SerialUSB.println("Ready to send data");
}
 
void setup()
{   
    sodaq_gps.init();                                       //do this as early as possible, so we have a workign gps.
    //sodaq_gps.setDiag(SerialUSB);
	params.read();                                          //get any previously saved configs, if there are any (otherwise use default).
    initPower();
    initLeds(params.getIsLedEnabled());
    setPower(HIGH);                                         //turn board on
    BlueLedOn();                                            //indicate start of device
    handleSerialPort();
	delay(1000);
    SerialUSB.println("start of track and trace demo v1.0");
	rtc.begin();
    connect();

    Wire.begin();
    BlueLedOff();                                           //indicate end of init  
	//initLeds(params.getIsLedEnabled());
    if(params.getUseAccelero()){
        compass.init(LSM303::device_D);
        compass.enableDefault();
        int16_t x, y = 0;                                       //the current x, y, z acceleration values (for gravity compensation)
        calibrate(x, y);
        prepareInterrupts(x, y);
        //the default state of the accelero meter is shut-down mode.
        if(setState(false) == true){
            delay(3000);                                            //small delay for the lora congestion. It's not enough, but will do. First gps fix (3th message) usually takes a while 
            reportBatteryStatus(Modem, Device);                     //send the current battery status at startup to report init state. This also puts the modem a sleep.
        }
	}
    else{
		reportBatteryStatus(Modem, Device);
        startGPSFix();
	}
    SerialUSB.println("tracker demo running");
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
    return count > params.getAcceleroSensitivity();
}

//called if the device detected previous movement and is know using a timer to wake up and check if there is still movement.
//When this is the case, send gps fix. Otherwise, signal that movement has stopped, recalibrate and set up the interrupts again.
void checkIfStillMoving()
{
    int16_t x, y = 0;                                       //the current x, y, z acceleration values (for gravity compensation)
    if (isMoving() == false) {                              //no more movement: report movement has stopped + start reporting battery again  + enable accelero interrupts again.
        SerialUSB.println("movement stopped");
        setState(false);
        calibrate(x, y);           
        setThresholds(x, y);
        activateAcceleroInterupts();
		if(_gpsScanning)									//if we are still scanning the gps, then the timer also needs to be activated again.
			setWakeUpClock();
		else
			startReportingBattery(rtc);
    }
    else{
        SerialUSB.println("movement continues");
		if(_gpsScanning == false)
			startGPSFix();
		else
			setWakeUpClock();										//restart the timer
    }
}

void loop()
{
    tryReportBattery(Modem, Device); 
	//tryReportTempAndSpeed();
    if(params.getUseAccelero()){
        if (wakeFromTimer == true) {                                //we got woken up by the timer, so check if still moving.  
            wakeFromTimer = false;
            
            if(_wasMoving)
                checkIfStillMoving();
            else if(_gpsScanning){                                       //if we still need to send a gps fix, try to do it now.             
                if(!trySendGPSFix())								//if the gps fix operation didn't finish, then we still need to restart the timer, otherwise it already happened.
					setWakeUpClock();
			}
        }
        else if(hasMoved()){
            SerialUSB.println("start of movement detected");
            disableAcceleroInterupts();
			if(_wasMoving == false)								//don't want to send the same value 2 times. This appears to happen sometimes at first, if the accelerometer buffered some data.
				setState(true);
            setWakeUpClock();
        }
	}
    else if (wakeFromTimer == true){
		SerialUSB.println("wake up from timer");
        wakeFromTimer = false;
        if(_gpsScanning){                                       //if we already started to send a gps fix, try to see if we have some data to send.  
            trySendGPSFix();
			setWakeUpClock();
		}
        else
            startGPSFix();
    }
	if(!params.getIsDebugMode())
		sleep();
}
