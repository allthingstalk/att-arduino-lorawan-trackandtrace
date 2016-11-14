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
#include "ATT_LoRaWAN_LSM303.h"


#define ACCEL_ADR 0b0011110
#define ACCELERO_COMPENSATION 0.061
#define ACCEL_SENSITIVITY 2

#define CONSOLE_SERIAL SerialUSB

LSM303 compass;
volatile bool int1_flag = false;
volatile bool int2_flag = false;

void ISR1()
{
  int1_flag = true;
}

void ISR2()
{
  int2_flag = true;
}

void calibrate(int16_t &x, int16_t &y)
{
	x = 0;
	y = 0;
	for (int i = 1; i <= 20; i++)
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

void setThresholds(int16_t x, int16_t y)
{
	SerialUSB.print("base x: "); SerialUSB.println(x);
	SerialUSB.print("base y: "); SerialUSB.println(y);
	x = (round((x * ACCELERO_COMPENSATION) / 16)) + 2;					//16 mg step size at 2g accuratie
	y = (round((y * ACCELERO_COMPENSATION) / 16)) + 2;
	SerialUSB.print("x: 0x"); SerialUSB.print(x, HEX); SerialUSB.print("; 0b"); SerialUSB.println(x, BIN);
	SerialUSB.print("y: 0x"); SerialUSB.print(y, HEX); SerialUSB.print("; 0b"); SerialUSB.println(y, BIN);

	writeReg(0x32, x & 0x7F); // Threshold
	writeReg(0x33, 0b00000000); // Duration

	writeReg(0x34, 0b10001000); // Axes mask
	writeReg(0x36, y & 0x7F); // Threshold
}

void activateAcceleroInterupts()
{

	writeReg(0x22, 0b00100000);
	writeReg(0x23, 0b00100000);
}

void setup() 
{
	while(!CONSOLE_SERIAL){}
	CONSOLE_SERIAL.println("Testing Accelerometer");

	// Allow power to remain on
    pinMode(ENABLE_PIN_IO, OUTPUT);
    digitalWrite(ENABLE_PIN_IO, HIGH);
	
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;	// Set sleep mode
	
	CONSOLE_SERIAL.println("sleeping for a few seconds so you can update new soft");
	delay(30000);
  	
	Wire.begin();
		
	compass.init(LSM303::device_D);
	compass.enableDefault();
	int16_t x, y = 0;										//the current x, y, z acceleration values (for gravity compensation)
	calibrate(x, y);
	
	pinMode(ACCEL_INT1, INPUT_PULLUP);
	pinMode(ACCEL_INT2, INPUT_PULLUP);
	attachInterrupt(ACCEL_INT1, ISR1, FALLING);
	
	//has to be set after first call to attachinterrupt
	// Set the XOSC32K to run in standby
	SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;

	// Configure EIC to use GCLK1 which uses XOSC32K 
	// This has to be done after the first call to attachInterrupt()
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | 
                    GCLK_CLKCTRL_GEN_GCLK1 | 
                    GCLK_CLKCTRL_CLKEN;
	
	attachInterrupt(ACCEL_INT2, ISR2, FALLING);
	
	// Set the XOSC32K to run in standby
	SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;

	// Configure EIC to use GCLK1 which uses XOSC32K 
	// This has to be done after the first call to attachInterrupt()
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | 
                    GCLK_CLKCTRL_GEN_GCLK1 | 
                    GCLK_CLKCTRL_CLKEN;
	
	
	writeReg(0x1F, 0b10000000); //reboot
	writeReg(0x20, 0b01010111); //ctrl1
	writeReg(0x30, 0b10000010); // Axes mask
	setThresholds(x, y);
	writeReg(0x37, 0b00000000);
	activateAcceleroInterupts();
	

}

void loop()
{
  if (int1_flag) {
    int1_flag = false;
    CONSOLE_SERIAL.println("INT1 Interrupt");
  }
  if (int2_flag) {
    int2_flag = false;
    CONSOLE_SERIAL.println("INT2 Interrupt");
  }
  compass.read();
  SerialUSB.print("x: "); SerialUSB.print(compass.a.x * 0.061);
  SerialUSB.print(", y: "); SerialUSB.print(compass.a.y* 0.061);
  SerialUSB.print(", z: "); SerialUSB.print(compass.a.z* 0.061);
//  
  CONSOLE_SERIAL.println("31 response: 0x" + String(readReg(0x31), BIN));
  CONSOLE_SERIAL.println("35 response: 0x" + String(readReg(0x35), BIN));
 

  CONSOLE_SERIAL.println("going to sleep");
  //Disable USB
  USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
  __WFI();							//Enter sleep mode
  //Enable USB
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
  digitalWrite(LED_RED, LOW);
  delay(3000);
  digitalWrite(LED_RED, HIGH);
}

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
