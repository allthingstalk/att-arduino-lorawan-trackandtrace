// Accel Test

#include <Wire.h>
#include "LSM303.h"



#define CONSOLE_SERIAL SerialUSB

LSM303 compass;

LSM303::vector<int16_t> base;
void setup() 
{
	while(!CONSOLE_SERIAL){}
	CONSOLE_SERIAL.println("Testing Accelerometer");
  	
	Wire.begin();	
	
	compass.init(LSM303::device_D);
	compass.enableDefault();
	
	compass.read();
	SerialUSB.print("x: "); SerialUSB.print(compass.m.x); SerialUSB.print("y: "); SerialUSB.print(compass.m.y);  SerialUSB.print("z: "); SerialUSB.println(compass.m.z);
	base = compass.m;
	
 
  
}

void loop()
{

  compass.read();
  SerialUSB.print("x: "); SerialUSB.print(compass.m.x); SerialUSB.print("y: "); SerialUSB.print(compass.m.y);  SerialUSB.print("z: "); SerialUSB.println(compass.m.z);
	double div = abs(base.x - compass.m.x) + abs(base.y - compass.m.y) + abs(base.z - compass.m.z);
	SerialUSB.print("dif: "); SerialUSB.println(div);
   
  delay(500);

}




