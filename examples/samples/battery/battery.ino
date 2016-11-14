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

#define ADC_AREF 3.3f
#define BATVOLT_R1 2.0f
#define BATVOLT_R2 2.0f
#define BATVOLT_PIN BAT_VOLT

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin(57600);
  
  while(!SerialUSB){
    //wait for Serial Monitor to be opened
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t bat = getBatteryVoltage();
  SerialUSB.println(bat);

  
  bat = (uint16_t)((100.0/ 1200.) * (float) (3000 - bat));
  SerialUSB.println(bat);
  
  delay(1000);
}

uint16_t getBatteryVoltage()
{
    uint16_t voltage = (uint16_t)((ADC_AREF / 1.023) * (BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2 * (float)analogRead(BATVOLT_PIN));

    return voltage;
}
