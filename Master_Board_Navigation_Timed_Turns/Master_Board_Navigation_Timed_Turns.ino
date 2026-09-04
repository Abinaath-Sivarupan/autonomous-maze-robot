//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  UoN EEEBot                                          *//
//*                                                      *//
//*  Skeleton Master Code for Use with the               *//
//*  EEEBot_MainboardESP32_Firmware Code                 *//
//********************************************************//

// the following code acts as a 'bare bones' template for your own custom master code that works with the firmware code provided
// therefore, the variable names are non-descriptive - you should rename these variables appropriately
// you can either modify this code to be suitable for the project week task, or use the functions as inspiration for your own code

#include <Wire.h>
#include <NewPing.h>
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal

// For ultrasonic sensing
#define LED_ULTRASONIC 25
#define TRIG 32
#define ECHO 33
#define MAX_DIST 400
NewPing sonar(TRIG, ECHO, MAX_DIST);

// setting PWM properties
const int freq = 5000;
const int ledChannelU = 11; // Ultrasonic LED PWM channel
const int resolution = 8;

// ---------- Defining variables ----------
int leftMotor = 0;
int rightMotor = 0;
int steeringAngle = 90;
float distance = 100.0f;

int16_t enc1Count = 0; // Left encoder
int16_t enc2Count = 0; // Right encoder

int16_t enc1Bef = 0;
int16_t enc2Bef = 0;

//flag to go straight is 0
int flag = 0;
/* FLAG IS HOW THE BOT KNOWS WHAT STEP TO TAKE IN NAVIGATION */

void setup()
{
  Serial.begin(9600);

  // For Ultrasonic Sensor LED
  ledcAttachChannel(LED_ULTRASONIC, freq, resolution, ledChannelU);

  Wire.begin();   // join i2c bus (address optional for the master) - on the ESP32 the default I2C pins are 21 (SDA) and 22 (SCL)

  //initially go forwards
  goForwards();
}



void loop()
{
  getEncoderValues();
  writeMotorAndSteering();
  getUltrasonicDistance();

  defaultLeftNavigationTime();
  
  delay(100);
}

#define TURN_90_COUNT 120
#define TURN_180_COUNT 240

#define FORWARD      0
#define TURN_LEFT    1
#define CHECK_FRONT  2
#define TURN_180     3
#define BACKTRACK    4
#define WALL         20
#define DELAY_90     410
#define DELAY_180    950
#define REST_DELAY   800


void defaultLeftNavigationTime() {
  switch(flag) {

    case FORWARD:
      goForwards();
      if (distance <= WALL) {
        flag = TURN_LEFT;
      }
      break;

    case TURN_LEFT:
      rotateAntiClock();
      delay(DELAY_90);        // approximate 90° turn
      stopMotors();
      delay(REST_DELAY);            // rest
      getUltrasonicDistance();
      flag = CHECK_FRONT; // move to check front state
      break;

    case CHECK_FRONT:
      getUltrasonicDistance();
      if (distance > WALL) {
        flag = FORWARD;   // path clear → go forward
      } else {
        flag = TURN_180;  // blocked → prepare 180° turn
      }
      break;

    case TURN_180:
      rotateClock();
      delay(DELAY_180);        // approximate 180° turn
      stopMotors();
      delay(REST_DELAY);            // rest
      getUltrasonicDistance();
      if (distance > WALL) {
        flag = FORWARD;   // path clear → go forward
      } else {
        flag = BACKTRACK; // blocked → backtrack
      }
      break;

    case BACKTRACK:
      rotateClock();
      delay(DELAY_90 + 300);        // approximate 90° turn back
      stopMotors();
      delay(REST_DELAY);      // rest
      getUltrasonicDistance();
      if (distance > WALL) {
        flag = FORWARD;       // path clear → resume forward
      } else {
        flag = BACKTRACK;     // still blocked → try another 90° turn
      }
      break;
  }
}

void getEncoderValues()
{
  // two 16-bit integer values are requested from the slave
  
  uint8_t bytesReceived = Wire.requestFrom(I2C_SLAVE_ADDR, 4);  // 4 indicates the number of bytes that are expected
  uint8_t a16_9 = Wire.read();  // receive bits 16 to 9 of a (one byte)
  uint8_t a8_1 = Wire.read();   // receive bits 8 to 1 of a (one byte)
  uint8_t b16_9 = Wire.read();   // receive bits 16 to 9 of b (one byte)
  uint8_t b8_1 = Wire.read();   // receive bits 8 to 1 of b (one byte)

  enc1Count = (a16_9 << 8) | a8_1; // combine the two bytes into a 16 bit number
  enc2Count = (b16_9 << 8) | b8_1; // combine the two bytes into a 16 bit number

  Serial.print(enc1Count);
  Serial.print("\t");
  Serial.println(enc2Count);
  Serial.print("\t");
}

void writeMotorAndSteering()
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  /* depending on the microcontroller, the int variable is stored as 32-bits or 16-bits
     if you want to increase the value range, first use a suitable variable type and then modify the code below
     for example; if the variable used to store x and y is 32-bits and you want to use signed values between -2^31 and (2^31)-1
     uncomment the four lines below relating to bits 32-25 and 24-17 for x and y
     for my microcontroller, int is 32-bits hence x and y are AND operated with a 32 bit hexadecimal number - change this if needed

     >> X refers to a shift right operator by X bits
  */
  //Wire.write((byte)((x & 0xFF000000) >> 24)); // bits 32 to 25 of x
  //Wire.write((byte)((x & 0x00FF0000) >> 16)); // bits 24 to 17 of x
  Wire.write((byte)((leftMotor & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  //Wire.write((byte)((y & 0xFF000000) >> 24)); // bits 32 to 25 of y
  //Wire.write((byte)((y & 0x00FF0000) >> 16)); // bits 24 to 17 of y
  Wire.write((byte)((rightMotor & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.write((byte)((steeringAngle & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(steeringAngle & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  Wire.endTransmission();   // stop transmitting
}

void getUltrasonicDistance()
{
  // controls LED brightness aswell
  int brightness = 0;

  // gets echo time in us
  unsigned long echo_us = sonar.ping();
  
  if(echo_us == 0){
    //Serial.println("Error");
    return;
  }
  
  // convert directly to cm
  distance = (echo_us * 0.0343f) / 2.0f;
  float scale = constrain((100.0 - distance) / 90.0, 0.0, 1.0);
  brightness = 255 * scale * scale;

  Serial.println(distance);
  //Serial.print("\n");
  //Serial.println(brightness);

  ledcWrite(ledChannelU, brightness);
}

void stopMotors()
{
  leftMotor = 0;
  rightMotor = 0;
  steeringAngle = 90;
  writeMotorAndSteering();
}

void goForwards()
{
  leftMotor = 200;
  rightMotor = 200;
  steeringAngle = 90;
  writeMotorAndSteering();
}

void rotateAntiClock()
{
  leftMotor = -200;
  rightMotor = 200;
  steeringAngle = 0;
  writeMotorAndSteering();
}

void rotateClock()
{
  leftMotor = 200;
  rightMotor = -200;
  steeringAngle = 180;
  writeMotorAndSteering();
}