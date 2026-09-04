#include <Wire.h>
#define I2C_SLAVE_ADDR 0x04

#define WAIT 2000
#define WAIT_LOW 500

// Sensor pins
#define PD6 36  // Left
#define PD5 39  // Mid-Left
#define PD4 34  // Center-Left
#define PD3 35  // Center-Right
#define PD2 26  // Mid-Right
#define PD1 27  // Right

// calibration arrays
int sensorMin[6] = {4095, 4095, 4095, 4095, 4095, 4095};
int sensorMax[6] = {0, 0, 0, 0, 0, 0};
int sensorCalibrated[6];

// sensor weights (positions in mm from center)
// with ~20mm spacing: -50, -30, -10, 10, 30, 50
int weights[6] = {-50, -30, -10, 10, 30, 50};

// PID variables
float Kp = 1.0;   // proportional gain - original values 1.0
float Ki = 0.005;   // integral gain 
float Kd = 0.5;   // derivative gain - 0.5
// preferecnces
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float pidOutput = 0;

// motor values
int baseSpeed = 110;  // motor speed
int leftMotor = 0;
int rightMotor = 0;
int steeringAngle = 90;

// scaling factor for motor speed adjustment in eqn
float K_motor = 0.3;  // ensure < 1


void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  // setup sensor pins
  pinMode(PD1, INPUT);
  pinMode(PD2, INPUT);
  pinMode(PD3, INPUT);
  pinMode(PD4, INPUT);
  pinMode(PD5, INPUT);
  pinMode(PD6, INPUT);
  pinMode(25, OUTPUT); //for led

  writeMotorAndSteering(); // will reset servo and motors upon start
  
  delay(WAIT);
  // run calibration
  calibrateSensors();
  
  delay(WAIT);
  Serial.println("Starting line following...");
}

void loop() {
  readSensors();
  calculatePID();
  applyPIDControl();
  
  delay(50);  // loop rate just in case
}

void calibrateSensors() {
  Serial.println("Starting calibration...");
  Serial.println("Place robot over black line in 3 seconds...");

  // LED visual thing
  for(int i = 0; i < 2; i++) {
    digitalWrite(25, HIGH); 
    delay(WAIT_LOW);
    digitalWrite(25, LOW);
    delay(WAIT_LOW);
  }

  // read black values
  Serial.println("Reading black values...");
  digitalWrite(25, HIGH); 
  for(int i = 0; i < 100; i++) {
    int sensors[6] = {
      analogRead(PD1), analogRead(PD2), analogRead(PD3),
      analogRead(PD4), analogRead(PD5), analogRead(PD6)
    };
    
    for(int j = 0; j < 6; j++) {
      if(sensors[j] < sensorMin[j]) sensorMin[j] = sensors[j];
      if(sensors[j] > sensorMax[j]) sensorMax[j] = sensors[j];
    }
    delay(10);
  }
  delay(WAIT_LOW);
  digitalWrite(25, LOW);

  /* ---------------------------------- */
  
  Serial.println("Place robot over white surface in 3 seconds...");
  // LED visual thing
  for(int i = 0; i < 2; i++) {
    digitalWrite(25, HIGH); 
    delay(WAIT_LOW);
    digitalWrite(25, LOW);
    delay(WAIT_LOW);
  }
  
  // Calibrate on WHITE
  Serial.println("Reading white values...");
  digitalWrite(25, HIGH); 
  for(int i = 0; i < 100; i++) {
    int sensors[6] = {
      analogRead(PD1), analogRead(PD2), analogRead(PD3),
      analogRead(PD4), analogRead(PD5), analogRead(PD6)
    };
    
    for(int j = 0; j < 6; j++) {
      if(sensors[j] < sensorMin[j]) sensorMin[j] = sensors[j];
      if(sensors[j] > sensorMax[j]) sensorMax[j] = sensors[j];
    }
    delay(10);
  }
  delay(WAIT_LOW);
  digitalWrite(25, LOW);
  
  /* ---------------------------------- */

  Serial.println("Calibration complete!");
  Serial.println("Min values:");
  for(int i = 0; i < 6; i++) {
    Serial.print(sensorMin[i]); 
    Serial.print(" ");
  }
  Serial.println();
  Serial.println("Max values:");
  for(int i = 0; i < 6; i++) {
    Serial.print(sensorMax[i]); 
    Serial.print(" ");
  }
  Serial.println();
}

void readSensors() {
  int sensors[6] = {
    analogRead(PD1), analogRead(PD2), analogRead(PD3),
    analogRead(PD4), analogRead(PD5), analogRead(PD6)
  };
  
  for(int i = 0; i < 6; i++) {
    // map to 1000-0 instead of 0-1000 to invert the output for eqn 2
    // this means: black surface = high value (1000)
    //             white surface = low value  (0)
    sensorCalibrated[i] = map(sensors[i], sensorMin[i], sensorMax[i], 1000, 0);
    sensorCalibrated[i] = constrain(sensorCalibrated[i], 0, 1000);
    Serial.print(sensorCalibrated[i]); Serial.print(",");

  }
  Serial.println();
}

float calculateWeightedAverage() {
  long numerator = 0;
  long denominator = 0;
  
  for(int i = 0; i < 6; i++) {
    numerator += (long)weights[i] * sensorCalibrated[i];
    denominator += sensorCalibrated[i];
  }
  
  if(denominator == 0) {
    return 0;  // no line detected
  }
  
  return (float)numerator / denominator;
}

void calculatePID() {
  float weightedAvg = calculateWeightedAverage();
  
  // error = setpoint - measurement
  // setpoint is 0 (center of sensor array)
  error = 0 - weightedAvg;
  
  // proportional term
  float P = Kp * error;
  
  // integral term (cumulative error)
  integral += error;
  float I = Ki * integral;
  
  // derivative term (rate of change of error)
  derivative = error - lastError;
  float D = Kd * derivative;
  
  // calc output
  pidOutput = P + I + D;
  
  // update last error
  lastError = error;
  /*
  // serial output
  Serial.print("Error: "); Serial.print(error);
  Serial.print(" | P: "); Serial.print(P);
  Serial.print(" | I: "); Serial.print(I);
  Serial.print(" | D: "); Serial.print(D);
  Serial.print(" | PID: "); Serial.println(pidOutput);
  */
}

void applyPIDControl() {
  // steering angle adjustment (Equation 6)
  steeringAngle = 90 + (int)pidOutput;
  steeringAngle = constrain(steeringAngle, 0, 180);
  
  // motor speed adjustment (Equations 7 & 8)
  leftMotor = baseSpeed + (int)(K_motor * pidOutput);
  rightMotor = baseSpeed - (int)(K_motor * pidOutput);
  
  // constrain motor speeds
  leftMotor = constrain(leftMotor, 0, 255);
  rightMotor = constrain(rightMotor, 0, 255);
  
  // send to mainboard
  writeMotorAndSteering();
}

void writeMotorAndSteering() {
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