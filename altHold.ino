/*
 * Altitude hold code
 * FEEG2001 Multirotor UAV Group Design Project 2016/17
 * 
 * Team 3:
 * Mohamed Nawabuddin
 * Amir Vafabakhsh
 * Daniel Amargianitakis
 * Adrian Weishaeupl
 * Peter Wattle
 * 
 * 
 * This code is organised into the following hierarchy:
 * 
 * 1) the void loop contains the high-level state machine which checks if autopilot mode is engaged
 * 2) the FlyTo function is called and it controls all the other low-level functions
 * 
 * The rest of the functions are mostly simpler versions taken from the full autopilot sketch. We recommend going through that sketch first.
 * 
 */

#include <PID_v1.h>                      //PID library by Brett Beauregard from http://playground.arduino.cc/Code/PIDLibrary
#include <Servo.h>                       //Arduino Servo library
#include <AltSoftSerial.h>              //Software Serial library by Paul Stoffregen from https://github.com/PaulStoffregen/AltSoftSerial
#include <QuickStats.h>                 //Statistics library by David Dubins from http://playground.arduino.cc/Main/QuickStats

                                        //NOTE: Some variables are declared with type 'double' so that the PID library will work

const short int IRSensorPinX = 0;       //Arduino Pin assignments
const short int IRSensorPinY = 1;
const short int IRSensorPinZ = 2;
const short int doorPin = 12;
const short int throCmdPin = 8;
const short int rollCmdPin = 9;
const short int pitchCmdPin = 10;
const short int yawCmdPin = 11;
const short int throInPin = 2;
const short int rollInPin = 3;
const short int pitchInPin = 4;
const short int yawInPin = 6;
const short int APInPin = 7;

short int rawIRZ;                       //Raw ADC number
float uncorrIRZ;                      //Distance reading before correction applied
double IRDistanceZ;                     //Measured IR distance

short int throValRx;;                //throttle PWM from pilot
double throVal;                     //PWM command variables
short int rollVal;
short int pitchVal;
short int yawVal;

double target = 100.0;                  //Target distance in cm
const short int targetTol = 2;          //+- tolerance from target in cm

const byte attRequest[] =               //Attitude request byte stream to the naze32
          {0x24, 0x4D, 0x3C, 0x00, 0x6C, 0x6C};
const byte IMURequest[] =               //IMU request byte stream to the naze32
          {0x24, 0x4D, 0x3C, 0x00, 0x66, 0x66};
float pitch, roll, yaw = 0;                  //Parsed attitude variables from naze32 byte stream (in deg)
short int accX, accY;                         //Parsed accelerometer variables from naze32 byte stream (in metres per second)
short int accZ = 9.81;
const short int accTol = 2;         //+- tolerance from target acceleration in metres per second

short int hoverCount = 0;           //these variables store the throttle value used for the hoverCheck function
short int hoverStore[3];
short int hoverMean = 1500;

const short int lMat[3][3] = {       // 3x3 matrix which contains projected lengths from C of G to each IR sensor in each axis
  {5, 4, 3},                        // (used in the attitude correction function correctIR()
  {1, 5, 3},
  {3, 3, 4}
};

volatile int startPulse = 0;       //variables used for checking if autopilot mode is engaged
volatile int APSig = 1500;
const short int APThreshold = 1600;

short int APState =0;              //state variables used by the state machine
bool doorState = 0;

bool rxSenseErr = 0;               //to check for any error while sensing receiver values

float startTime, endTime, runTime = 0;   //looptime measuring code

short int FDRCount = 1;                 //Count for skipping FDR loops
const short int FDRTrigger = 5;         //Loop which triggers FDR routine    

Servo doorServo;                        //Dropper servo

Servo throCmd;                      //Objects for each PWM command channel
Servo rollCmd;
Servo pitchCmd;
Servo yawCmd;

QuickStats stats;                       //Statistics object (used for sorting data, finding mean, finding median)

PID throPID                      //PID object for controlling throttle PWM using IR reading
    (&IRDistanceZ, &throVal, &target, 1.0, 0.2, 0, 0);

AltSoftSerial bt;              //Bluetooth data logging connection. Uses AltSoftSerial lib with Timer3 as the timer to prevent clashes with Servo lib


void ISRRising() {             //ISRs for checking when autopilot is engaged
  startPulse = micros();
}


void ISRFalling() {  
  APSig = micros() - startPulse;
}


void setup() {
  
  attachInterrupt(digitalPinToInterrupt(APInPin), ISRRising, RISING);
  attachInterrupt(digitalPinToInterrupt(APInPin), ISRFalling, FALLING);

  Serial.begin(115200);               //Serial is the connection to the PC
  Serial1.begin(115200);              //Serial1 is the connection to the naze32 UART2
  bt.begin(38400);                     //bt is for bluetooth Flight Data Recording (library forces to use pin 5 for tx and 13 for rx)

  throPID.SetMode(AUTOMATIC);           //Begin PID computation automatically
  throPID.SetOutputLimits(1500, 1600);  //Maximum PWM Output Limits of the PID are set here
  throPID.SetSampleTime(50);            //IR variable sampling rate is set here

  doorServo.attach(doorPin);                       //Attach door servo to pin
  doorServo.write(0);
  throCmd.attach(throCmdPin, 1000, 2000);  //Attach PWM commands to pins and set maximum PWM limits
  rollCmd.attach(rollCmdPin, 1000, 2000);
  pitchCmd.attach(pitchCmdPin, 1000, 2000);
  yawCmd.attach(yawCmdPin, 1000, 2000);

}


void loop() {
  
  startTime = micros();   //to check looptime

  if (APSig > APThreshold && APState == 0) {    //check if autopilot is engaged
    throPID.SetOutputLimits(hoverMean, hoverMean + 15);  //if yes then tune PID limits and make APState 1
    APState = 1;
  } else if (APSig < APThreshold && APState == 1) {   //if not then make APState 0
    APState = 0;
  }
  
  FlyTo('Z');                       //main function that controls the low-level functions

  payloadCheck();                   //function that checks if target altitude is reached and opens door
  
  endTime = micros();           //end of looptime check
  runTime = endTime - startTime;
  
  sendFlightData();               //check if bluetooth data needs sending this loop, and then send bluetooth data
  
}


void FlyTo (char axis) {    //function which takes in axis to go towards, and then runs low-level functions

  computePID();             //make PID library start computation
    
  sendNazeReq('A');         //request attitude data from naze32
  
  ReadIR(axis);             //read, filter and calibrate IR data
  
  readAttData();            //read incoming attitude data
  
  sendNazeReq('I');         //send IMU request to naze32
  
  correctIR(axis);          //correct IR distance using attitude data

  readAccData();            //read incoming IMU data and extract accelerometer readings

  readRx();                 //read pilot's signals

  hoverCheck();             //check pilot's throttle value if drone is hovering, and use that to tune throttle PID limits

  sendPWM();                //finally send the set of PWM signals
    
}


void sendNazeReq(char req) {    //Function to send data requests to naze32
  if (req == 'A')  Serial1.write(attRequest, sizeof(attRequest));         //Send attitude request to naze32
  else if (req == 'I') Serial1.write(IMURequest, sizeof(IMURequest));     //Send accelerometer request to naze32
}


void skipNazeBuffer(int bytes) {        //Function to skip over unwanted bytes in incoming naze32 byte stream
  for(int i = 1; i <= bytes; i++) {
    Serial1.read();
  }
}


void ReadIR(char axis) {           //Function to read IR, then filter and calibrate
  
  float IRReadingArray[10];                //Raw IR readings are stored here
  float IRMedian;                          //Median of IRReadingArray array after bubble sort operation
  float IRDistanceArray[3];                //Converted IR distances are stored here
  float IRDistance;                        //Average of IRDistanceArray
  
  for (int j = 0; j < sizeof(IRDistanceArray)/sizeof(IRDistanceArray[0]); j++) {    //Outer For loop which filters and finds IR distance
    for (int i = 0; i < sizeof(IRReadingArray)/sizeof(IRReadingArray[0]); i++) {    //Inner For loop which reads and stores analogue voltages
      if (axis == 'Z') IRReadingArray[i] = analogRead(IRSensorPinZ);
      delay(2);
    }
    IRMedian = stats.median(IRReadingArray, sizeof(IRReadingArray)/sizeof(IRReadingArray[0]));  //Finding median of the readings
    if (axis == 'Z') { rawIRZ = IRMedian; IRDistanceArray[j] = 9533 * pow(IRMedian, -1);}       //Converting ADC to distance
  }
  IRDistance = stats.average(IRDistanceArray, sizeof(IRDistanceArray)/sizeof(IRDistanceArray[0]));  //Finding mean of the array
   
  if (isnan(IRDistance)) IRDistance = 150;    //If IRDistance is not a number, change to 150 cm
  if (IRDistance < 0) IRDistance = 0;         //If IRDistance is out of range, change to endpoint in cm
  if (IRDistance > 150) IRDistance = 150;     
  
  if (axis == 'Z') IRDistanceZ = IRDistance;    //Transfer value to global variable
  
}


void readAttData() {            //Function to read Naze32 attitude data and parse angles
  
  byte nazeData[6];                        //Received attitude byte stream from naze32
  
  while (Serial1.available()) {         //While naze32 buffer is not 0, read in attitude data and store into variables
    skipNazeBuffer(5);
    for (int i = 0; i <= 5; i++) {       //Store attitude bytes into array and skip non-pertinent bytes
      nazeData[i] = (Serial1.read());
    }
    skipNazeBuffer(1);
    
    pitch = (uint8_t(nazeData[3]) << 8 | uint8_t(nazeData[2])) * -0.1;    //Change byte order from little-endian to big-endian and store in variable
    roll = (uint8_t(nazeData[1]) << 8 | uint8_t(nazeData[0])) * 0.1;
    yaw = (uint8_t(nazeData[5]) << 8 | uint8_t(nazeData[4]));
  }

  if (isnan(pitch)) pitch = 0;        //If data is not a number, change to 0 degrees
  if (isnan(roll)) roll = 0;
  if (isnan(yaw)) yaw = 0;
  if (yaw > 180) yaw -= 360;          //Change yaw from bearing format to (-179 <= yaw <= 180) format
  
}


void readAccData() {            //Function to read Naze32 attitude data and parse angles

  byte nazeData[6];                        //Received accelerometer byte stream from naze32

  while (Serial1.available()) {      //Store accelerometer bytes into array and skip non-pertinent bytes
    skipNazeBuffer(5);
    for (int i = 0; i <= 5; i++) {
      nazeData[i] = (Serial1.read());
    }
    skipNazeBuffer(13);
    
    accX = (uint8_t(nazeData[1]) << 8 | uint8_t(nazeData[0])) * 9.81 / 512;  //Change byte order from little-endian to big-endian and convert to metres per second
    accY = (uint8_t(nazeData[3]) << 8 | uint8_t(nazeData[2])) * 9.81 / 512;
    accZ = (uint8_t(nazeData[5]) << 8 | uint8_t(nazeData[4])) * 9.81 / 512;
  }
  
}


void correctIR(char axis) {   //Function to correct IR reading using attitude data

  if (axis == 'Z') {          //Uses matrix and formula explained in report
    uncorrIRZ = IRDistanceZ;
    IRDistanceZ  = (IRDistanceZ
                     + lMat[2][2]
                     + lMat[2][1]*tan(radians(pitch))
                     + lMat[2][0]*tan(radians(roll)) 
                   ) * cos(radians(pitch))
                   * cos(radians(roll));
  }
  
}


void hoverCheck() {     //Function to check if the drone is hovering, and then capture the pilot throttle value for tuning the PID in another function later
                        //Can work with just one reading, but is able to take a maximum of 3 if it can (3 is the size of hoverStore array)
                        
  if (hoverCount < (sizeof(hoverStore)/sizeof(hoverStore[0])) - 1) {
    if ((accX >= (0 - accTol)) && (accX <= (0 + accTol)) &&
        (accY >= (0 - accTol)) && (accY <= (0 + accTol)) &&
        (accZ >= (9.8 - accTol)) && (accZ <= (9.8 + accTol))) {
          hoverStore[hoverCount] = throValRx;
          hoverMean = 0;
          for (int i = 0; i <= hoverCount; i++) hoverMean += hoverStore[i];
          hoverMean /= (hoverCount + 1);
          hoverCount += 1;
    }
  }

}


void computePID() {   //Function that makes the PID library update its computation.

  throPID.Compute();
  
}


void readRx() {          //Function to read the pilot's signals from the receiver. Used by the hoverCheck function and sendPWM function

  throValRx = pulseIn(throInPin, HIGH, 30000);
  pitchVal = pulseIn(pitchInPin, HIGH, 30000);
  rollVal = pulseIn(rollInPin, HIGH, 30000);
  yawVal = pulseIn(yawInPin, HIGH, 30000);

  if (throValRx > 2000 || throValRx < 1000) {rxSenseErr = 1; throValRx = 1500;}  else rxSenseErr = 0;  //Display message and set to default if read error
  if (pitchVal > 2000 || pitchVal < 1000) {rxSenseErr = 1; pitchVal = 1500;}  else rxSenseErr = 0;
  if (rollVal > 2000 || rollVal < 1000) {rxSenseErr = 1; rollVal = 1500;}  else rxSenseErr = 0;
  if (yawVal > 2000 || yawVal < 1000) {rxSenseErr = 1; yawVal = 1500;}  else rxSenseErr = 0;

}


void sendPWM() {            //Function to send the PWM data to the Naze32
  
  throCmd.writeMicroseconds(throVal);       //Write throttle PID outputs to Naze32
  rollCmd.writeMicroseconds(rollVal);       //Pass receiver PWM signals to Naze32
  pitchCmd.writeMicroseconds(pitchVal);
  yawCmd.writeMicroseconds(yawVal);
  
}


void payloadCheck() {     //Checks if drone is at target (+- tolerance), and drops payload
  
  if (!doorState) {                                //Check if door state is not already 1
    if (IRDistanceZ >= (target - targetTol) && IRDistanceZ <= (target + targetTol)) {    //If door is 0, check if target distances are met
      doorServo.write(90);                   //Open door and change door state to 1
      doorState = 1;
    }
  }
  
}


void sendFlightData() {       //Function to send flight data over bluetooth
  
  if (FDRCount == FDRTrigger) {                       //Check if FDR needs to happen this loop

    bt.print("RAW ALT\t\t");
    bt.println(rawIRZ);
    bt.print("UNCORR ALT\t");
    bt.println(uncorrIRZ);
    bt.print("PITCH\t");
    bt.print(pitch);
    bt.print("\tROLL\t");
    bt.print(roll);
    bt.print("\tYAW\t");
    bt.println(yaw);
    bt.print("CORR ALT\t");
    bt.println(IRDistanceZ);

    bt.println();
    if (APState == 1) {
      bt.println("\tAP");
    }

    if (rxSenseErr) {
      bt.println("\tRX SENSE ERROR");
    }
    
    if (doorState) {
      bt.println("\tDOOR OPEN");
    }
    bt.println();

    bt.print("HOVER PID LVL\t");
    bt.println(hoverMean);
    bt.print("RX THRO\t\t");
    bt.println(throValRx);
    bt.print("THRO CMD\t");
    bt.println(throVal);
    bt.print("ROLL CMD\t");
    bt.println(rollVal);
    bt.print("PITCH CMD\t");
    bt.println(pitchVal);
    bt.print("YAW CMD\t\t");
    bt.println(yawVal);
    bt.println();
  
    bt.print("LOOPTIME:\t");
    bt.println(runTime);
    bt.println();
  
    FDRCount = 0;
  } else {                                        //Else increment FDR counter
    FDRCount += 1;
  }
  
}

