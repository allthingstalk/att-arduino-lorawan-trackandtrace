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

#define CONSOLE_SERIAL SerialUSB

#define ACCEL_ADR 0b0011110


volatile bool int1_flag = false;


void ISR1()
{
  int1_flag = true;
}

int16_t _baseX, _baseY, _baseZ;

void calibrate()
{
	int16_t x_val, y_val, z_val;
	float x, y, z = 0;
	for (int i = 1; i <= 50; i++)
	{
		readMagneto(x_val, y_val, z_val);
		SerialUSB.print("x: "); SerialUSB.print(x_val); SerialUSB.print("y: "); SerialUSB.print(y_val);  SerialUSB.print("z: "); SerialUSB.println(z_val);
		x = x - (x / i) + ((float)x_val / i);
		y = y - (y / i) + ((float)y_val / i);
		z = z - (z / i) + ((float)z_val / i);
		SerialUSB.print("avg x: "); SerialUSB.print(x); SerialUSB.print("avg y: "); SerialUSB.print(y);  SerialUSB.print("avg z: "); SerialUSB.println(z);
		delay(100);
	}
	_baseX = (int16_t)x;
	_baseY = (int16_t)y;
	_baseZ = (int16_t)z;
}

void setup() 
{
	digitalWrite(LED_RED, LOW);
	
	while(!CONSOLE_SERIAL){}
	CONSOLE_SERIAL.println("Testing Magnetometer");

	Wire.begin();
	
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	
	pinMode(ACCEL_INT1, INPUT_PULLUP);
	pinMode(ACCEL_INT2, INPUT_PULLUP);
	attachInterrupt(ACCEL_INT1, ISR1, FALLING);
	SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;						// Set the XOSC32K to run in standby
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
	attachInterrupt(ACCEL_INT2, ISR1, FALLING);
	SYSCTRL->XOSC32K.bit.RUNSTDBY = 1;						// Set the XOSC32K to run in standby
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
	

	writeReg(0x1F, 0b10000000);  //reboot
	writeReg(0x24, 0b11110000);
	writeReg(0x25, 0b01100000);
	writeReg(0x26, 0b00000000);
	calibrate();
	writeReg(0x12, 0b11100001); // Axes mask
	uint16_t threshold = (max(max(_baseX, _baseY), _baseZ) + 50) & ~(1 << 15);
	writeReg(0x14, (byte) threshold & 0x00FF);
	writeReg(0x15, (byte)(threshold >> 8));
	writeReg(0x22, 0b00001000);
	writeReg(0x23, 0b00000000);
	
	delay(3000);
	digitalWrite(LED_RED, HIGH);
}

void readMagneto(int16_t &x, int16_t &y, int16_t &z)
{
	x = (readReg(0x09) << 8) | readReg(0x08);
	y = (readReg(0x0B) << 8) | readReg(0x0A);
	z = (readReg(0x0D) << 8) | readReg(0x0C);
}

void loop()
{
	if (int1_flag) {
		int1_flag = false;
		CONSOLE_SERIAL.println("INT1 Interrupt");
	}
	
	int16_t x_val, y_val, z_val;
	readMagneto(x_val, y_val, z_val);
	CONSOLE_SERIAL.println(String("Magnetometer Readings: ") + x_val + ", " + y_val + ", " + z_val);
	CONSOLE_SERIAL.print("Reading from register: 0x" + String(0x13, HEX));
	CONSOLE_SERIAL.print(", response: ");
	CONSOLE_SERIAL.println(readReg(0x13), BIN);
	USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE;
	__WFI();							//Enter sleep mode
	USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
	
	digitalWrite(LED_RED, LOW);
	delay(3000);
	digitalWrite(LED_RED, HIGH);
	delay(1000);
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

uint8_t writeReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  delayMicroseconds(10000);
}
