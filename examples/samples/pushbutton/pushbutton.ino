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



