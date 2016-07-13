// Magnetometer Interrupt Test

#include <Wire.h>

#define CONSOLE_SERIAL SerialUSB

#define ACCEL_ADR 0b0011110

#define TEST_INT1
//#define TEST_INT2

volatile bool int1_flag = false;
volatile bool int2_flag = false;

float oldVec[3];
float curVec[3];

void setup() 
{
  delay(5000);
  Wire.begin();

  CONSOLE_SERIAL.println("Testing Magnetometer");

  pinMode(ACCEL_INT1, INPUT_PULLUP);
  pinMode(ACCEL_INT2, INPUT_PULLUP);
  attachInterrupt(ACCEL_INT1, ISR1, FALLING);
  attachInterrupt(ACCEL_INT2, ISR2, FALLING);

  // Reboot 
  // CTRL0
  writeReg(0x1F, 0b10000000);

  // Enable Temperature, High Resolution, 50hz, no interrupts
  // CTRL5
  writeReg(0x24, 0b11110000);

  // +/- 12 gauss
  // CTRL6
  writeReg(0x25, 0b01100000);

  // Magnetic sensor in Continous-conversion mode (reset)
  // CTRL7
  writeReg(0x26, 0b00000000);

  // Threshold values
  uint16_t threshold = 2000 & ~(1 << 15);
  uint8_t threshold_L = (uint8_t)threshold;
  uint8_t threshold_H = (uint8_t)(threshold >> 8);

  // Interrupt source IGM
  // Set x,y,z sensitivity
  writeReg(0x12, 0b11100001); // Axes mask
  writeReg(0x14, threshold_L); // Threshold Low byte
  writeReg(0x15, threshold_H); // Threshold High byte
  
  // CTRL3 (INT1)
  // INT1 sources from IGM
#ifdef TEST_INT1
  writeReg(0x22, 0b00001000);
#else
  writeReg(0x22, 0b00000000);
#endif

  // CTRL4 (INT2)
  // INT2 sources from IGM
  // Note the bit positions are different
  // in CTRL3 and CTRL4 Datasheet p.38
#ifdef TEST_INT2
  writeReg(0x23, 0b00010000);
#else
  writeReg(0x23, 0b00000000);
#endif
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
  
  int16_t t_val = (readReg(0x06) << 8) | readReg(0x05);
  CONSOLE_SERIAL.println(String("Temperature Reading: ") + t_val);

  CONSOLE_SERIAL.print("Reading from register: 0x" + String(0x13, HEX));
  CONSOLE_SERIAL.print(", response: ");
  printBin(readReg(0x13));
  CONSOLE_SERIAL.println();
  
  float magnitude = getVector(curVec);
  //CONSOLE_SERIAL.println(String("Previous coefficients: ") + oldVec[0] + ", " + oldVec[1] + ", " + oldVec[2]);
  //CONSOLE_SERIAL.println(String("Cosine coefficients: ") + curVec[0] + ", " + curVec[1] + ", " + curVec[2]);
  //CONSOLE_SERIAL.println(String("Magnitude: ") + magnitude);
  
  float angleDiff = dotProduct(curVec, oldVec);
  //CONSOLE_SERIAL.println(String("Angle difference: ") + angleDiff);
  
  copyVec(curVec, oldVec);

  for (uint16_t i = 0; i < 1000; i++) {
    delayMicroseconds(1000);
  }
}

// Writes the normalised magnetometer vector to vec
// Returns the magnitude
float getVector(float vec[3])
{
  vec[0] = (float)((int16_t)(readReg(0x09) << 8) | readReg(0x08));
  vec[1] = (float)((int16_t)(readReg(0x0B) << 8) | readReg(0x0A));
  vec[2] = (float)((int16_t)(readReg(0x0D) << 8) | readReg(0x0C));

  float magnitude = 0.0f;
  for (uint8_t i = 0; i < 3; i++) {
    magnitude += (vec[i] * vec[i]);
  }
  magnitude = sqrt(magnitude);

  for (uint8_t i = 0; i < 3; i++) {
    vec[i] /= magnitude;
  }

  return magnitude;
}

// Assumes Vec1 & Vec2 are normalised
// Returns value in degrees
float dotProduct(float vec1[3], float vec2[3])
{
  return acos((vec1[0] * vec2[0]) + (vec1[1] * vec2[1]) + (vec1[2] * vec2[2])) * 57.2958f;
}

// Copies vec1 into vec2
void copyVec(float vec1[3], float vec2[3])
{
  for (uint8_t i = 0; i < 3; i++) {
    vec2[i] = vec1[i];
  }
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

void writeReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
  delayMicroseconds(10000);
}

void ISR1()
{
  int1_flag = true;
}

void ISR2()
{
  int2_flag = true;
}

void printBin(uint8_t b)
{
  CONSOLE_SERIAL.print("0b");
  for (uint8_t i = 0; i < 8; i++) {
    if (b & (1 << (7 - i)))
    {
      CONSOLE_SERIAL.print("1");
    }
    else {
      CONSOLE_SERIAL.print("0");
    }
  }
}
