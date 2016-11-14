/*
   Copyright 2016 AllThingsTalk

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/

#include <Wire.h>
#include <ATT_LoRaWAN_RTCZero.h>



#define CONSOLE_SERIAL SerialUSB

LSM303 compass;


void setup() 
{
	while(!CONSOLE_SERIAL){}
	CONSOLE_SERIAL.println("Testing Accelerometer");
  	
	Wire.begin();	
	
	compass.init(LSM303::device_D);
	compass.enableDefault();
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
  
  //compass.writeReg(0x14, (byte) 0);
  //compass.writeReg(0x15, (byte)0);

  //compass.writeReg(0x16, 0);
  //compass.writeReg(0x17, 0);

  //compass.writeReg(0x18, 0);
  //compass.writeReg(0x19, 0);

  //compass.writeReg(0x1A, 0);
  //compass.writeReg(0x1B, 0);

  
}

void loop()
{

  compass.read();
  SerialUSB.print("x: "); SerialUSB.print(compass.m.x); SerialUSB.print("y: "); SerialUSB.print(compass.m.y);  SerialUSB.print("z: "); SerialUSB.println(compass.m.z);
  //SerialUSB.print("x: "); SerialUSB.print(compass.a.x * 0.061);
  //SerialUSB.print(", y: "); SerialUSB.print(compass.a.y* 0.061);
  //SerialUSB.print(", z: "); SerialUSB.println(compass.a.z* 0.061);
	float curHeading = compass.heading();
	SerialUSB.print("heading: "); SerialUSB.println(curHeading);
 

  delay(500);

}




