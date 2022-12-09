#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const uint8_t DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN 
  
const float DXL_PROTOCOL_VERSION = 2.0;

#include <PID_v1.h>   // PID

double Pk0 = 0.1; 
double Ik0 = 0.0;
double Dk0 = 0.0025;

double Setpoint0, Input0, Output0, Output0a;    // PID variables
PID PID0(&Input0, &Output0, &Setpoint0, Pk0, Ik0 , Dk0, DIRECT);    // PID Setup

double Pk1 = 0.1; 
double Ik1 = 0.0;
double Dk1 = 0.05;

double Setpoint1, Input1, Output1, Output1a;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

double Pk2 = 0.1; 
double Ik2 = 0.0;
double Dk2 = 0.1;

double Setpoint2, Input2, Output2, Output2a;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

int sw1;
int sw2;
int sw3;
int sw4;
int sw5;
float pot1 = 512;
float pot2 = 512;
float pot3 = 512;
float pot1Filtered = 512;
float pot2Filtered = 512;
float pot3Filtered = 512;
float pot4;
float pot5;
float pot6;
float pot4a;
float pot5a;
float pot6a;
int pot4Scaled;
int pot5Scaled;
int pot6Scaled;

int wheel1;
int wheel2;
int wheel3;
int wheel1a;
int wheel2a;
int wheel3a;

float dyn0 = 180;
float dyn1 = 180;
float dyn2 = 180;
float dyn3 = 180;

float dyn0Filtered = 180;
float dyn1Filtered = 180;
float dyn2Filtered = 180;
float dyn2FilteredPrev = 180;
float dyn3Filtered = 180;

// serial vars:
int var1;
int var2;
int var3;
int check1;

float var1a;
float var2a;
float var3a;

float var1aFiltered;
float var2aFiltered;
float var3aFiltered;

int foot1;
int foot2;
int foot3;
int foot1Flag = 0;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer
long previousNodMillis = 0;    // set up timers
long previousFoot1Millis = 0;    // set up timers
long previousHandMillis = 0;    // set up timers
int nodFlag = 0;
int handFlag = 0;
int nodDepth = 10;
int nodTime = 250;

int dyn0Bookmark;
int dyn1Bookmark;
int dyn2Bookmark;
int dyn3Bookmark;
int dyn0aBookmark;
int dyn1aBookmark;
int dyn2aBookmark;
int dyn3aBookmark;
int dyn0bBookmark;
int dyn1bBookmark;
int dyn2bBookmark;
int dyn3bBookmark;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {

  pinMode(22, INPUT_PULLUP);      // switches
  pinMode(24, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);
  pinMode(28, INPUT_PULLUP);
  pinMode(30, INPUT_PULLUP); 

  pinMode(53, INPUT_PULLUP);      // footswitches
  pinMode(51, INPUT_PULLUP);
  pinMode(49, INPUT_PULLUP);
  
  pinMode(3, OUTPUT);             // motor PWMs
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(115200);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  dxl.ping(0);
  dxl.ping(1);
  dxl.ping(2);
  dxl.ping(3);

  // Turn off torque when configuring items in EEPROM area
  dxl.torqueOff(0);
  dxl.torqueOff(1);
  dxl.torqueOff(2);
  dxl.torqueOff(3);
  dxl.setOperatingMode(0, OP_POSITION);
  dxl.setOperatingMode(1, OP_POSITION);
  dxl.setOperatingMode(2, OP_POSITION);
  dxl.setOperatingMode(3, OP_POSITION);
  dxl.torqueOn(0);
  dxl.torqueOn(1);
  dxl.torqueOn(2);
  dxl.torqueOn(3);

  Serial2.begin(115200);      // serial comms to Nano

  PID0.SetMode(AUTOMATIC);              
  PID0.SetOutputLimits(-390, 390);
  PID0.SetSampleTime(10);

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-640, 640);
  PID1.SetSampleTime(10);
  
  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-640, 640);
  PID2.SetSampleTime(10);

  // Limit the maximum velocity in Position Control Mode. Use 0 for Max speed
  dxl.writeControlTableItem(PROFILE_VELOCITY, 0, 120);      // pan
  dxl.writeControlTableItem(PROFILE_VELOCITY, 2, 65);      // arm
  dxl.writeControlTableItem(PROFILE_VELOCITY, 1, 120);      // tilt
  dxl.writeControlTableItem(PROFILE_VELOCITY, 3, 120);      // zoom

  dxl.setGoalPosition(2, 180, UNIT_DEGREE);     // arm  
}

void loop() {

    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {  // start timed event    
          previousMillis = currentMillis;


          if (Serial2.available() > 1){
            check1 = Serial2.parseInt();
                if (check1 == -20) {               //wait for the check value to come around before reading the rest of the data
                    var1 = Serial2.parseInt();     // detection width
                    var2 = Serial2.parseInt();     // left to right
                    var3 = Serial2.parseInt();     // top
                    if (var1 > 300) {              // filter out small items in background
                      var1a = float(var1);
                      var2a = float(var2);
                      var3a = float(var3);
                    }                   
                }
          }
 
    pot1 = analogRead(A8);
    pot2 = analogRead(A9);
    pot3 = analogRead(A10);
    pot4 = analogRead(A11);
    pot5 = analogRead(A12);
    pot6 = analogRead(A13);

    foot1 = digitalRead(49);
    foot2 = digitalRead(51);
    foot3 = digitalRead(53);

    pot1Filtered = filter(pot1, pot1Filtered,30);
    pot2Filtered = filter(pot2, pot2Filtered,30);
    pot3Filtered = filter(pot3, pot3Filtered,30);

    pot4a = thresholdStick(pot4);                   // manual driving knobs
    pot5a = thresholdStick(pot5);
    pot6a = thresholdStick(pot6);
  
    pot4Scaled = map(pot4a,-512,512,255,-255);
    pot5Scaled = map(pot5a,-512,512,255,-255);
    pot6Scaled = map(pot6a,-512,512,-255,255);
    
    sw1 = digitalRead(22);
    sw2 = digitalRead(24);
    sw3 = digitalRead(26);
    sw4 = digitalRead(28);
    sw5 = digitalRead(30);

    var1aFiltered = filter(var1a, var1aFiltered, 50);
    var2aFiltered = filter(var2a, var2aFiltered, 30);
    var3aFiltered = filter(var3a, var3aFiltered, 30);  
  
    if (sw1 == 0) {         /// vision tracking

        //pan

        Input1 = var2aFiltered;           // centre horiz
        Setpoint1 = 640;
        PID1.Compute();
        dyn0 = dyn0 + (Output1/300);
        dyn0 = constrain(dyn0,135,225);

        // tilt           
        Input2 = var3aFiltered-300;      // top
        Setpoint2 = -200;
        PID2.Compute();       
        Output2 = Output2 + 100;        // create and remove offsets so there is equal swing around zero - otherwise the distance between the top of the detection is smaller at the top.
        Output2 = map(Output2,0,100,-100,000);
        dyn1 = dyn1 - (Output2/300);
        dyn1 = constrain(dyn1,135,225);
         
        // zoom
        Input0 = var1aFiltered - 1000;     // width
        Setpoint0 = 0;
        PID0.Compute();  
        dyn3 = Output0+50;
        dyn3 = map(dyn3,60,290,290,60);
        dyn3 = constrain(dyn3,60,290);       
  
    }

    else if (sw1 == 1) {                  // use knobs in front to fix position
      dyn0 = map(pot1Filtered,0,1023,135,225);
      dyn1 = map(pot2Filtered,0,1023,135,225);
      dyn3 = map(pot3Filtered,0,1023,60,290);
    }

    // footswitch - look away to preset position

    if (foot1 == 0 && foot1Flag == 0 ) {         // boommark current positions
          dyn0Bookmark = dyn0;
          dyn1Bookmark = dyn1;
          dyn3Bookmark = dyn3;
          previousFoot1Millis =  currentMillis;
          foot1Flag = 1;    
    }

    else if (foot1Flag == 1) {                    // go to preset positions
          dyn0 = map(pot1Filtered,0,1023,135,225);
          dyn1 = map(pot2Filtered,0,1023,135,225);
          dyn3 = map(pot3Filtered,0,1023,60,290);
          if (foot1 == 0 && currentMillis - previousFoot1Millis >= 500) {
            foot1Flag = 2;
            previousFoot1Millis =  currentMillis;
          }  
    }

   else if (foot1Flag == 2) {     // go back to the book marks.
          dyn0 = dyn0Bookmark;
          dyn1 = dyn1Bookmark;
          dyn3 = dyn3Bookmark;
          if (currentMillis - previousFoot1Millis >= 1500) {
            foot1Flag = 0;
          }
    }

    Serial.println(foot1Flag);

    // nod for yes when footswitch 2 is pressed      

    if (foot2 == 0 && nodFlag == 0) {
      previousNodMillis =  currentMillis;
      dyn1aBookmark = dyn1;
      dyn1 = dyn1aBookmark + nodDepth;
      nodFlag = 1;
    }
    else if (nodFlag == 1 && currentMillis - previousNodMillis >= nodTime) {
      previousNodMillis =  currentMillis;
      dyn1 = dyn1aBookmark - nodDepth;
      nodFlag = 2;
    }
    else if (nodFlag == 2 && currentMillis - previousNodMillis >= nodTime) {
      previousNodMillis =  currentMillis;
      dyn1 = dyn1aBookmark + nodDepth;
      nodFlag = 3;
    }
    else if (nodFlag == 3 && currentMillis - previousNodMillis >= nodTime) {
      previousNodMillis =  currentMillis;
      dyn1 = dyn1aBookmark - nodDepth;
      nodFlag = 4;
    }
    else if (nodFlag == 4 && currentMillis - previousNodMillis >= nodTime) {
      previousNodMillis =  currentMillis;
      dyn1 = dyn1aBookmark + nodDepth;
      nodFlag = 5;
    }
    else if (nodFlag == 5 && currentMillis - previousNodMillis >= nodTime) {
      previousNodMillis =  currentMillis;
      dyn1 = dyn1aBookmark - nodDepth;
      nodFlag = 6;
    }
    else if (nodFlag == 6 && currentMillis - previousNodMillis >= nodTime) {
      previousNodMillis =  currentMillis;
      dyn1 = dyn1aBookmark + nodDepth;
      nodFlag = 7;
    }
    else if (nodFlag == 7 && currentMillis - previousNodMillis >= nodTime) {
      dyn1 = dyn1aBookmark;
      if (currentMillis - previousNodMillis >= nodTime) {
        nodFlag = 0;
      }
    }


    

    // *** thumbs up

    if (foot3 == 0 && handFlag == 0) {
      dyn0bBookmark = dyn0;
      dyn1bBookmark = dyn1;
      dyn3bBookmark = dyn3;
      handFlag = 1;
      previousHandMillis =  currentMillis; 
    }
    else if (handFlag == 1) {
      dyn2 = 75;          // arm
      dyn3 = 290;         // zoom  
      dyn0 = dyn0bBookmark;
      dyn1 = dyn1bBookmark;          
      if (currentMillis - previousHandMillis >= 2000) {
        dyn2 = 180;    
        dyn3 = dyn3bBookmark;    
        handFlag = 0;
      }
    }    

    // write to servos
  
    dyn0Filtered = filter(dyn0, dyn0Filtered, 10);
    dyn1Filtered = filter(dyn1, dyn1Filtered, 10);    
    dyn3Filtered = filter(dyn3, dyn3Filtered, 10);
    dyn2Filtered = filter(dyn2, dyn2Filtered, 25);    /// arm

     // *** write to dynamixels  

    if (dyn2Filtered != dyn2FilteredPrev) {
        dxl.setGoalPosition(2, dyn2Filtered, UNIT_DEGREE);      // only write to arm if the value changes
    }
    dyn2FilteredPrev = dyn2Filtered;   
    
    dxl.setGoalPosition(1, dyn1Filtered, UNIT_DEGREE);
    dxl.setGoalPosition(3, dyn3Filtered, UNIT_DEGREE);  

    if (sw2 == 1) {                                           // operate pan servo
      dxl.setGoalPosition(0, dyn0Filtered, UNIT_DEGREE);      
    }

    // *** use the wheels for transaltion instead of pan ***

    else if (sw2 == 0) {                              
      dxl.setGoalPosition(0, 180, UNIT_DEGREE);               // fix pan servo in the middle
      pot5Scaled = map(var2aFiltered, 0,1280,-512,512);
      if (pot5Scaled > 100) {
        pot5Scaled = pot5Scaled - 100;                        // deadspot
      }
      else if (pot5Scaled < -100) {
        pot5Scaled = pot5Scaled + 100;
      }
      else {
        pot5Scaled = 0;
      }
      pot5Scaled = constrain(pot5Scaled,-255,255);
    }    
  
    // *** drive motors ***    
  
    wheel1 = pot4Scaled - (pot5Scaled*.66) - pot6Scaled;  
    wheel3 = pot4Scaled + (pot5Scaled*.66) + pot6Scaled;
    wheel2 = pot5Scaled - pot6Scaled;

    wheel1 = constrain(wheel1,-255,255);
    wheel2 = constrain(wheel2,-255,255);
    wheel3 = constrain(wheel3,-255,255);
  
    if (sw3 == 0) {   // allow motors to move
  
      if (wheel1 > 0) {
        analogWrite(10, wheel1);
        analogWrite(3, 0);
      }
      else if (wheel1 < 0) {
        wheel1a = abs(wheel1);
        analogWrite(3, wheel1a);
        analogWrite(10, 0); 
      }
      else {
        analogWrite(3, 0);
        analogWrite(10, 0);
      }
  
      // *****
  
      if (wheel2 > 0) {
        analogWrite(4, wheel2);
        analogWrite(5, 0);
      }
      else if (wheel2 < 0) {
        wheel2a = abs(wheel2);
        analogWrite(5, wheel2a);
        analogWrite(4, 0);
      }
      else {
        analogWrite(4, 0);
        analogWrite(5, 0);
      }
  
      // *****
  
      if (wheel3 > 0) {
        analogWrite(8, wheel3);
        analogWrite(9, 0);
      }
      else if (wheel3 < 0) {
        wheel3a = abs(wheel3);
        analogWrite(9, wheel3a);
        analogWrite(8, 0);
      }
      else {
        analogWrite(8, 0);
        analogWrite(9, 0);
      }
      
    }  
      else if (sw2 == 1) {
        analogWrite(3, 0);
        analogWrite(10, 0);
    
        analogWrite(4, 0);
        analogWrite(5, 0);
    
        analogWrite(8, 0);
        analogWrite(9, 0);
    } 
  

  }  // end of 10ms timed loop

}   // end of main


