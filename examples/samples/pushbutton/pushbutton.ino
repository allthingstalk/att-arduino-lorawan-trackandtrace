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

void setup() {
  //start serial connection
  SerialUSB.begin(9600);
  //configure pin2 as an input and enable the internal pull-up resistor
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED_GREEN, OUTPUT);
}

void loop() {
  //read the pushbutton value into a variable
  int sensorVal = digitalRead(BUTTON);
  //print out the value of the pushbutton
  SerialUSB.print("new value: ");
  SerialUSB.println(sensorVal);

  // Turn on the LED when the Button is pushed
  if (sensorVal == HIGH) {
    digitalWrite(LED_GREEN, HIGH);
  } else {
    digitalWrite(LED_GREEN, LOW);
  }
}



