/***********************************************************************
 * ECE4007 Senior Design
 * Arm control code 
 * Accelerometer control code
 * Author: Peihsun (Ben) Yeh
 * 
 * Accelerometer library provided by Nathan Seidle (Sparkfun Electronics)

 * Connections:
 * Digital pin 10 - base servo signal 
 *              9 - shoulder servo signal 
 *              6 - elbow servo signal 
 *              5 - wrist servo signal 
 * Analog in    4 - SDA from glove
 *              5 - SCL from glove
 ***********************************************************************/

#include <Servo.h>
#include <Wire.h>
#include <math.h>

/*** Accelerometer address and register definitions ***/
// The SparkFun breakout board defaults to 1, 
// set to 0 if SA0 jumper on the bottom of the board is set
// 0x1D if SA0 is high, 0x1C if low 
#define MMA8452_ADDRESS 0x1D 
//Define a few of the registers that we will be accessing on the MMA8452
#define OUT_X_MSB 0x01
#define XYZ_DATA_CFG 0x0E
#define WHO_AM_I 0x0D
#define CTRL_REG1 0x2A
#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.

#define PI 3.14159

// Object definitions
Servo baseServo, shoulderServo, elbowServo, wristServo;

// Motor ranges hardcoded for normal arm
int basePin = 10, shoulderPin = 9, elbowPin = 6, wristPin = 5; 
int baseMin     = 10, baseMax     = 150;
int shoulderMin = 95, shoulderMax = 170;
int elbowMin    = 45, elbowMax    = 130;
int wristMin    = 10, wristMax    = 170;
int armAngles[5] = {0, 100, 60, 90, 0}; // 0 - dummy, 1 - base, 2 - shoulder
                                        // 3 - elbow, 4 - wrist
int accelCount[3]; // stores accelerometer raw data
float accelG[3]; // stores converted accelerometer data

// Function prototypes
void readAccelData(int *destination);
void initMMA8452();
void MMA8452Standby();
void MMA8452Active();
void readRegisters(byte addressToRead, int bytesToRead, byte * dest);
byte readRegister(byte addressToRead);
void writeRegister(byte addressToWrite, byte dataToWrite);
void convertAccelerometerData();

void setup() {
  Serial.begin(9600);      

  // initialize servos
  baseServo.attach(basePin);
  baseServo.write(90);
  
  shoulderServo.attach(shoulderPin);
  shoulderServo.write(100);
  
  elbowServo.attach(elbowPin);
  elbowServo.write(130);
  
  wristServo.attach(wristPin);
  wristServo.write(90);
    
  //Initialize accelerometer
  Wire.begin();
  initMMA8452();
  delay(1000);
}

void loop() {
  // get data as they become available
  if(Serial.available() > 3){
    for(int i = 1; i < 4; i++){
      armAngles[i] = Serial.read();
    }  
  }
  
  // mapping the accelerometer readout to wrist angles
  readAccelData(accelCount);
  convertAccelerometerData();
  armAngles[4] = (atan2(accelG[2], accelG[0])*(180.0/PI));
  //Serial.println(armAngles[4]);
	if(armAngles[4] >= 0 && armAngles[4] < 90){ 					//[0, 90] -> [270, 180]
    armAngles[4] = map(armAngles[4], 0, 90, 270, 180);
  } else if(armAngles[4] >=90 && armAngles[4] <= 180){ 	//[90, 180] -> [180, 90]
    armAngles[4] = map(armAngles[4], 90, 180, 180, 90); 
  } else if(armAngles[4] > -180 && armAngles[4] < -90){ //[-180, -90] -> [90, 0]
    armAngles[4] = map(armAngles[4], -180, -90, 90, 0); 
  } else if(armAngles[4] >= -90 && armAngles[4] < 0){ 	//[-90, 0] -> [360, 270]
    //armAngles[4] = map(armAngles[4], -90, 0, 360, 270);
    armAngles[4] = 0;  
  }
  
  // write to base motor
  if(armAngles[1] >= baseMin && armAngles[1] <= baseMax) baseServo.write(armAngles[1]);
  else if(armAngles[1] <= baseMin) baseServo.write(baseMin);
  else if(armAngles[1] >= baseMax) baseServo.write(baseMax);
  delay(10);
  
  // write to shoulder motor
  if(armAngles[2] >= shoulderMin && armAngles[2] <= shoulderMax) shoulderServo.write(armAngles[2]);
  else if(armAngles[2] <= shoulderMin) shoulderServo.write(shoulderMin);
  else if(armAngles[2] >= shoulderMax) shoulderServo.write(shoulderMax);
  delay(10);
  
  // write to elbow motor
  if(armAngles[3] >= elbowMin && armAngles[3] <= elbowMax) elbowServo.write(armAngles[3]);
  else if(armAngles[3] <= elbowMin) elbowServo.write(elbowMin);
  else if(armAngles[3] >= elbowMax) elbowServo.write(elbowMax);
  delay(10);
  
  // write to wrist motor
  if(armAngles[4] >= wristMin && armAngles[4] <= wristMax) wristServo.write(armAngles[4]);
  else if(armAngles[4] <= wristMin) wristServo.write(wristMin);
  else if(armAngles[4] >= wristMax) wristServo.write(wristMax);
  delay(10);
}

/**********************************************************************
 * Function definitions
 *********************************************************************/
void convertAccelerometerData(){
  for (int i = 0 ; i < 3 ; i++)
  {
    accelG[i] = (float) accelCount[i] / ((1<<12)/(2*GSCALE)); // get actual g value, this depends on scale being set
  }
}

/**********************************************************************
 * Accelerometer library function definitions - not mine
 *********************************************************************/
void readAccelData(int *destination)
{
  byte rawData[6]; // x/y/z accel register data stored here

  readRegisters(OUT_X_MSB, 6, rawData); // Read the six raw data registers into data array

  // Loop to calculate 12-bit ADC and g value for each axis
  for(int i = 0; i < 3 ; i++)
  {
    int gCount = (rawData[i*2] << 8) | rawData[(i*2)+1]; //Combine the two 8 bit registers into one 12-bit number
    gCount >>= 4; //The registers are left align, here we right align the 12-bit integer

    // If the number is negative, we have to make it so manually (no 12-bit data type)
    if (rawData[i*2] > 0x7F)
    {
      gCount = ~gCount + 1;
      gCount *= -1; // Transform into negative 2's complement #
    }

    destination[i] = gCount; //Record this gCount into the 3 int array
  }
}

// Initialize the MMA8452 registers
// See the many application notes for more info on setting all of these registers:
// http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=MMA8452Q
void initMMA8452()
{
  byte c = readRegister(WHO_AM_I); // Read WHO_AM_I register
  if (c == 0x2A) // WHO_AM_I should always be 0x2A
  {
    //Serial.println("MMA8452Q is online...");
  }
  else
  {
    //Serial.print("Could not connect to MMA8452Q: 0x");
    //Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  MMA8452Standby(); // Must be in standby to change registers

  // Set up the full scale range to 2, 4, or 8g.
  byte fsr = GSCALE;
  if(fsr > 8) fsr = 8; //Easy error check
  fsr >>= 2; // Neat trick, see page 22. 00 = 2G, 01 = 4A, 10 = 8G
  writeRegister(XYZ_DATA_CFG, fsr);

  //The default data rate is 800Hz and we don't modify it in this example code

  MMA8452Active(); // Set to active to start reading
}

// Sets the MMA8452 to standby mode. It must be in standby to change most register settings
void MMA8452Standby()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c & ~(0x01)); //Clear the active bit to go into standby
}

// Sets the MMA8452 to active mode. Needs to be in this mode to output data
void MMA8452Active()
{
  byte c = readRegister(CTRL_REG1);
  writeRegister(CTRL_REG1, c | 0x01); //Set the active bit to begin detection
}

// Read bytesToRead sequentially, starting at addressToRead into the dest byte array
void readRegisters(byte addressToRead, int bytesToRead, byte * dest)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(MMA8452_ADDRESS, bytesToRead); //Ask for bytes, once done, bus is released by default

  while(Wire.available() < bytesToRead); //Hang out until we get the # of bytes we expect

  for(int x = 0 ; x < bytesToRead ; x++)
    dest[x] = Wire.read();
}

// Read a single byte from addressToRead and return it as a byte
byte readRegister(byte addressToRead)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToRead);
  Wire.endTransmission(false); //endTransmission but keep the connection active

  Wire.requestFrom(MMA8452_ADDRESS, 1); //Ask for 1 byte, once done, bus is released by default

  while(!Wire.available()) ; //Wait for the data to come back
  return Wire.read(); //Return this one byte
}

// Writes a single byte (dataToWrite) into addressToWrite
void writeRegister(byte addressToWrite, byte dataToWrite)
{
  Wire.beginTransmission(MMA8452_ADDRESS);
  Wire.write(addressToWrite);
  Wire.write(dataToWrite);
  Wire.endTransmission(); //Stop transmitting
}
