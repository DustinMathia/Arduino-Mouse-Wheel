//   _____                     _____     _                    _   _   _         
//  |     |___ _ _ ___ ___ ___|   __|___|_|___ ___ ___ ___   | |_| |_|_|___ ___ 
//  | | | | . | | |_ -| -_|___|__   | . | |   |   | -_|  _|  |  _|   | |   | . |
//  |_|_|_|___|___|___|___|   |_____|  _|_|_|_|_|_|___|_|    |_| |_|_|_|_|_|_  |
//                                  |_|                                    |___|
//   
//         by Dustin Mathia
//
//
//                  (\-.
//                  / _`> .---------.
//          _)     / _)=  |'-------'|
//         (      / _/    |O   O   o|
//          `-.__(___)_   | o O . o |
//                        `---------'
//
// 5/9/18
// Incremental encoder firmware + stepper motor control

#include <inttypes.h>
#include "HardwareSerial.h"

#define ENCODERPINA 2         // Rotary Encoder Left Pin //
#define ENCODERPINB 3         // Rotary Encoder Right Pin //
#define ENCODERPINZ 4         // Rotary Encoder Full Revolution (360 deg) //
#define STP 5
#define DIR 6
#define MS1 7
#define MS2 8
#define MS3 9
#define EN  10 
#define RST 11
#define SLEEP  12
#define CLUTCH  13
#define COUNTSPERREV 1024     // Encoder counts per revolution, hardware specific
#define STP360  400       // Steps per revolution (360 deg)

volatile int State_old = 1;            // Initial encoder state = 1: permits a left or right step
volatile int State_new = 0;
volatile int Max = 1024;                // max encoder counts (application dependent)
volatile int Min = -1024;               // min encoder counts (application dependent)
volatile int encCount = 0;             // encoder count
volatile int isCCW = 0;                // counter-clockwise rotation: TRUE or FALSE
volatile int tick = 0;                 // quadrature 'ticks', define the binary encoder state
volatile float encAngle = 0;           // encoder angle, full rotation is 360 deg
volatile int i = 0;
int x;
int numSteps;
int stepAngleDeg;         // commanded angle [deg]
float desiredTime;
float stepSpeed;
float oneStepAng = 360.0 / STP360;
int stepDirection;
int command, old_command;


void setup() {
  // put your setup code here, to run once:

Serial.println("Setup........");

  Serial.begin(9600); //Open Serial connection for debugging
  
  command = 0;
  old_command = 0;
  desiredTime = 2000;
  stepAngleDeg = 360;
  
  pinMode(STP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);
  pinMode(EN, OUTPUT);
  pinMode(RST, OUTPUT);
  pinMode(SLEEP, OUTPUT);
  pinMode(CLUTCH, OUTPUT);
  pinMode(ENCODERPINA, INPUT);
  digitalWrite(ENCODERPINA, HIGH);
  pinMode(ENCODERPINB, INPUT);
  digitalWrite(ENCODERPINB, HIGH);
  pinMode(ENCODERPINZ, INPUT);
  digitalWrite(ENCODERPINZ, HIGH);
  resetBEDPins();   // Default pin settings
  
  Serial.println("Initial conditions:");
  Serial.print("State_old=");
  Serial.print(State_old);
  Serial.print(", State_new=");
  Serial.print(State_new);
  Serial.print(", isCCW=");
  Serial.print(isCCW);
  Serial.print(", tick=");
  Serial.print(tick);
  Serial.print(", encCount=");
  Serial.print(encCount);
  Serial.print(", encAngle=");
  Serial.print(encAngle,4);
  Serial.println("");

  attachInterrupt(digitalPinToInterrupt(2), encoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encoderB, CHANGE);

  Serial.println("Starting Loop........");

}

void loop() {
  
  if (Serial.available()) 
  {
    command = Serial.read() - 48;
    if (command != old_command)
    {
      Serial.print("command = "); Serial.println(command);
    }
  }
      
    switch (command) {
    case 0:
        //Serial.println("Nothing selected...");
        break;
    case 1:
      stepAngleDegPositive(stepAngleDeg, desiredTime);
      Serial.println("");
      break;
    case 2:
      stepAngleDegNegative(stepAngleDeg, desiredTime);
      Serial.println("");
      break;
    case 3:
      Stop();
      Serial.println("");
      break;
      case 4:
      clutchEngage();
      Serial.println("");
      break;
      case 5:
      clutchDisengage();
      Serial.println("");
      break;
    default:
        Serial.println("Invalid command. Try again.");
        Serial.println("");
    break;
  }
  old_command = command;
  command = 0;    
}

//-------------------------------------------------------
// Reset Big Easy Driver pins to default states
//-------------------------------------------------------
void resetBEDPins()
{
  digitalWrite(STP, LOW);
  digitalWrite(DIR, LOW);
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  digitalWrite(MS3, LOW);
  digitalWrite(EN, HIGH);
  digitalWrite(RST, HIGH);
  digitalWrite(SLEEP, HIGH);
}  // end resetBEDPins


//-------------------------------------------------------
// Move positive (counterclockwise) by a specified angle in
// degrees.
//-------------------------------------------------------
void stepAngleDegPositive(float sAng, float desiredTime)
{
  int nStep;

  Serial.println("stepAngleDegForward...");

  // Compute required number of steps for the commanded angle
  nStep = round(sAng / oneStepAng);

  // Command the required number of steps
  numStepsPositive(nStep, desiredTime);
}

//-------------------------------------------------------
// Move negative (clockwise) by a specified angle in
// degrees.
//-------------------------------------------------------
void stepAngleDegNegative(float sAng, float desiredTime)
{
  int nStep;

  Serial.println("stepAngleDegBackward...");

  // Compute required number of steps for the commanded angle
  nStep = round(sAng / oneStepAng);

  // Command the required number of steps
  numStepsNegative(nStep, desiredTime);
}


//-------------------------------------------------------
// Move Positive (counterclockwise) by Steps
//-------------------------------------------------------
void numStepsPositive(int nStp, float desiredTime)
{
  Serial.println("numStepsPositive...");
  digitalWrite(EN, LOW);      // Enable (turn on) driver FETs for motion
  delay(1);
  digitalWrite(DIR, HIGH);    // ensure transition from HIGH to LOW: start with HIGH
  digitalWrite(DIR, LOW);     // transition to LOW: 'forward' direction is positive angle (CCW)

  for (x = 1; x < nStp; x++)  // Loop through the specified number pof steps
  {
    command = Serial.read() - 48;
    //Emergency Stop
    if (command == 3)
    {
      break;
    }

    else
    {
    // Step forward
    digitalWrite(STP, HIGH);  // transition LOW->HIGH, start step
    delay(chooseVelocity(desiredTime, nStp) - 1);                // delay after step determines velocity
    digitalWrite(STP, LOW);   // transition reset
    delay(1);                 // 1 ms for reset
    }
  } // end for

  digitalWrite(EN, HIGH);     // Turn driver FETs off for safety
  Serial.println("numStepsPositive end.");
}


//-------------------------------------------------------
// Move Negative (clockwise) by Steps
//-------------------------------------------------------
void numStepsNegative(int nStp, float desiredTime)
{
  Serial.println("numStepsNegative...");
  digitalWrite(EN, LOW);      // Enable (turn on) driver FETs for motion
  delay(1);
  digitalWrite(DIR, LOW);    // ensure transition from HIGH to LOW: start with HIGH
  digitalWrite(DIR, HIGH);     // transition to LOW: 'forward' direction is positive angle (CCW)

  for (x = 1; x < nStp; x++)  // Loop through the specified number pof steps
  {
    command = Serial.read() - 48;
    //Emergency Stop
    if (command == 3)
    {
      break;
    }

    else
    {
    // Step backwards
    digitalWrite(STP, HIGH);  // transition LOW->HIGH, start step
    delay(chooseVelocity(desiredTime, nStp) - 1);               // delay after step determines velocity
    digitalWrite(STP, LOW);   // transition reset
    delay(1);                 // 1 ms for reset
    }
  } // end for

  digitalWrite(EN, HIGH);     // Turn driver FETs off for safety
  Serial.println("numStepsNegative end.");
}


//-------------------------------------------------------
//Calculate Velcocity per Step
//-------------------------------------------------------
float chooseVelocity(float desiredTime, int numSteps)
{
  stepSpeed = desiredTime/numSteps;
  return stepSpeed;
  Serial.print("stepSpeed = ");
  Serial.println(stepSpeed);
}


//-------------------------------------------------------
//Calculate Velcocity per Step
//-------------------------------------------------------
int changeDirection(int stepDirection)
{
  Serial.println("changeDirection...");

  stepDirection = stepDirection * (-1);
  Serial.print("stepDirection = "); Serial.println(stepDirection);
  
  return stepDirection;
}


//-------------------------------------------------------
//Stop
//-------------------------------------------------------
void Stop()
{
  Serial.println("Stopping...");
  digitalWrite(EN, HIGH);  // Turn off power for EN PIN
}


//-------------------------------------------------------
//Start
//-------------------------------------------------------
void Start()
{
  Serial.println("Starting...");
  digitalWrite(EN, HIGH); // Turn on power for EN PIN
}


//-------------------------------------------------------
//Clutch ON
//-------------------------------------------------------
void clutchEngage()
{
  Serial.println("Clutch Engaging...");
  digitalWrite(CLUTCH, HIGH); // Activate the Clutch
}


//-------------------------------------------------------
//Clutch OFF
//-------------------------------------------------------
void clutchDisengage()
{
  Serial.println("Clutch Disengaging...");
  digitalWrite(CLUTCH, LOW); // Deactivate the Clutch
}

//-----------------------ENCODER-------------------------
void encoderA() {

  State_new = (digitalRead(ENCODERPINB) * 2) + digitalRead(ENCODERPINA);



  if (State_new != State_old)
  {
    // Determine state transition: CW or CCW? Here: no hardware error checking.
    isCCW = ((State_old == 0) && (State_new == 1)) || ((State_old == 1) && (State_new == 3)) ||
            ((State_old == 3) && (State_new == 2)) || ((State_old == 2) && (State_new == 0));

    // Forward or backward step?
    if (isCCW) 
      tick = tick + 1;
    else 
      tick = tick - 1;

    // Incremental, quadrature encoder: for 'ticks' per encoder count
    if (tick > 3)
    {
      tick = 0;
      encCount = encCount + 1;
    }
    else if (tick < -3)
    {
      tick = 0;
      encCount = encCount - 1;
    }

    // Limit encoder counts to the specified motion range
    if (encCount < Min) 
      encCount = Min;
    if (encCount > Max) 
      encCount = Max;

    // Compute encoder angle in degrees
    encAngle = (float) encCount*360/COUNTSPERREV;

  State_old = State_new;  
  }
}

void encoderB() {

  i = i + 1;
  State_new = (digitalRead(ENCODERPINB) * 2) + digitalRead(ENCODERPINA);

  if (State_new != State_old)
  {
    // Determine state transition: CW or CCW? Here: no hardware error checking.
    isCCW = ((State_old == 0) && (State_new == 1)) || ((State_old == 1) && (State_new == 3)) ||
            ((State_old == 3) && (State_new == 2)) || ((State_old == 2) && (State_new == 0));

    // Forward or backward step?
    if (isCCW) 
      tick = tick + 1;
    else 
      tick = tick - 1;

    // Incremental, quadrature encoder: for 'ticks' per encoder count
    if (tick > 3)
    {
      tick = 0;
      encCount = encCount + 1;
    }
    else if (tick < -3)
    {
      tick = 0;
      encCount = encCount - 1;
    }

    // Limit encoder counts to the specified motion range
    if (encCount < Min) 
      encCount = Min;
    if (encCount > Max) 
      encCount = Max;

    // Compute encoder angle in degrees
    encAngle = (float) encCount*360/COUNTSPERREV;
    if (i == 250) {
          i = 0;
          Serial.println(encAngle,3);
      }

  }
  State_old = State_new;
  
}
