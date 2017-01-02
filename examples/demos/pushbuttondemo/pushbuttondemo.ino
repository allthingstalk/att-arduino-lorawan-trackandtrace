/****
 *  AllThingsTalk Developer Cloud IoT experiment for LoRa
 *  version 1.0 dd 09/11/2015
 *  Original author: Jan Bogaerts 2015
 *
   Copyright 2015-2016 AllThingsTalk

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 
 *
 *  This example sketch is based on the Proxilmus IoT network in Belgium
 *  The sketch and libs included support the
 *  - MicroChip RN2483 LoRa module
 *  - Embit LoRa modem EMB-LR1272
 *  
 *  For more information, please check our documentation
 *  -> http://allthingstalk.com/docs/tutorials/lora/setup
 */
#include <Wire.h>
#include <ATT_IOT_LoRaWAN.h>
#include <MicrochipLoRaModem.h>
#include "keys.h"

#define SERIAL_BAUD 57600

MicrochipLoRaModem Modem(&Serial1, &SerialUSB);
ATTDevice Device(&Modem, &SerialUSB);

bool sensorVal = false;

void setup() 
{
  pinMode(ENABLE_PIN_IO, OUTPUT);
  digitalWrite(ENABLE_PIN_IO, HIGH);
  pinMode(BUTTON, INPUT_PULLUP);               			// initialize the digital pin as an input
  pinMode(LED_GREEN, OUTPUT);
  SerialUSB.begin(SERIAL_BAUD);                   		// set baud rate of the default serial debug connection
  Serial1.begin(Modem.getDefaultBaudRate());   			// set baud rate of the serial connection between Mbili and LoRa modem
  while(!Device.Connect(DEV_ADDR, APPSKEY, NWKSKEY))
	Serial.println("Retrying...");						// initialize connection with the AllThingsTalk Developer Cloud
  SerialUSB.println("Ready to send data");
  digitalWrite(LED_GREEN, HIGH);
  SendValue(0);                        					// send initial state

}


bool SendValue(bool val)
{
  SerialUSB.print("Data: ");SerialUSB.println(val);
  bool res = Device.Send(val, BINARY_SENSOR, true);
  if(res == false)
    SerialUSB.println("Ooops, there's an error sending data. Try again in couple of seconds");
  return res;
}

void loop() 
{
  bool sensorRead = digitalRead(BUTTON);      			// read status Digital Sensor
  if (sensorRead == 0 ) {                               // verify if value has changed
     if(SendValue(!sensorVal) == true) {
         digitalWrite(LED_GREEN, sensorVal);
         sensorVal = !sensorVal;
         }
   delay(1000);
   }
}

void serialEvent1()
{
  Device.Process();                           			//for future extensions -> actuators
}

