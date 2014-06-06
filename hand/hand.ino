/***********************************************************************
 * ECE4007 Senior Design
 * Hand and glove sensor code
 * Author: Peihsun (Ben) Yeh
 * 
 * Connections
 * Analog in    0 - thumb flex sensor
 *              1 - pointer flex sensor
 *              2 - middle flex sensor
 *              3 - ring flex sensor
 *              4 - pinky flex sensor
 * Digital out  11 - thumb servo signal
 *              10 - pointer servo signal
 *               9 - middle servo signal
 *               6 - ring servo signal
 *               5 - little servo signal
 ***********************************************************************/
#include <Servo.h>

// Object definition
Servo thumbServo, pointerServo, middleServo, ringServo, littleServo;

// Variables 
// index mapping: 0 - thumb
//                1 - pointer
//                2 - middle
//                3 - ring
//                4 - little
int sensorValue[5]; // array to hold flex sensor readings
int angles[5];      // array to hold angles to write to motors
int lowerBound[5];  // array to hold values for maximum flex
int upperBound[5];  // array to hold values for minimum flex
int thumbPin = 11, pointerPin = 10, middlePin = 9, ringPin = 6, littlePin = 5;
int upperCalibrateLed = 13;
int lowerCalibrateLed = 12;

int thumbRelaxed   = 50,  thumbBent   = 90;
int pointerRelaxed = 40,  pointerBent = 105;
int middleRelaxed  = 125, middleBent  = 65;
int ringRelaxed    = 105, ringBent    = 35;
int littleRelaxed  = 90, littleBent  = 40;

void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  
  // CALIBRATION PROCEDURE:
  // Hold hand open while LED 1 lit
  // Close hand while LED 2 lit
  // Done when both LEDs are off
  Serial.println("Calibrating...");
  
  pinMode(upperCalibrateLed, OUTPUT);
  pinMode(lowerCalibrateLed, OUTPUT);
  
  Serial.println("close hand...");
  digitalWrite(lowerCalibrateLed, 0);
  digitalWrite(upperCalibrateLed, 1);
  delay(5000);
  for(int i = 0; i < 5; i++){
    upperBound[i] = analogRead(i);
  }
  // light led 1, turn off led 2
  Serial.println("open hand...");
  digitalWrite(lowerCalibrateLed, 1);
  digitalWrite(upperCalibrateLed, 0);
  delay(5000);
  for(int i = 0; i < 5; i++){
    lowerBound[i] = analogRead(i);
  }
  // light led 2, turn off led 1
  digitalWrite(lowerCalibrateLed, 0);
  Serial.println("Done!");
  // calibration done
  
  // initialize servos
  thumbServo.attach(thumbPin);
  thumbServo.write(thumbRelaxed);

  pointerServo.attach(pointerPin);
  pointerServo.write(pointerRelaxed);

  middleServo.attach(middlePin);
  middleServo.write(middleRelaxed);

  ringServo.attach(ringPin);
  ringServo.write(ringRelaxed);

  littleServo.attach(littlePin);  
  littleServo.write(littleRelaxed);
  delay(1000);
}

void loop() {
  // read sensors and map its value in the range of [lowerBound, upperBound] 
  sensorValue[0] = analogRead(0);
  angles[0] = map(sensorValue[0], lowerBound[0], upperBound[0], thumbRelaxed, thumbBent);
  sensorValue[1] = analogRead(1);
  angles[1] = map(sensorValue[1], lowerBound[1], upperBound[1], pointerRelaxed, pointerBent);
  sensorValue[2] = analogRead(2);
  angles[2] = map(sensorValue[2], lowerBound[2], upperBound[2], middleRelaxed, middleBent);
  sensorValue[3] = analogRead(3);
  angles[3] = map(sensorValue[3], lowerBound[3], upperBound[3], ringRelaxed, ringBent);
  sensorValue[4] = analogRead(4);
  angles[4] = map(sensorValue[4], lowerBound[4], upperBound[4], littleRelaxed, littleBent);
  
  // write to finger motors
  thumbServo.write(angles[0]);
  pointerServo.write(angles[1]);
  middleServo.write(angles[2]);
  ringServo.write(angles[3]);
  littleServo.write(angles[4]);
  delay(10);  
}

