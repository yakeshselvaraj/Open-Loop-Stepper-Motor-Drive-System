#include <AccelStepper.h>

//// Pins ////
const byte stepper1EnablePin = 4;
const byte stepper1StepPin = 2;
const byte stepper1DirectionPin = 3;
const byte endstop1Pin = 8;


//// Constants (User Configured) ////
const int stepper1Speed = 500; //microsteps/s
const int stepper1Acceleration = 2000; // microsteps/s^2
const int stepper1HomingSpeed = 100; // microsteps/s
const int stepper1HomingBumpDiviser = 2; // amount to divide "stepper1HomingSpeed" by for the second homing bump
const int stepper1HomingBumpDistance = 20; // number of full steps moved away from the endstop after the first homing bump
const int stepper1HomeOffset = 0; // number of full steps to move away from the endstop after the homing sequency completes
const int stepper1DriverMinimumPulseWidth = 1; // minimum pulse width required by the stepper driver on the step input
const int stepper1Microstepping = 16; // microstepping mode set on the stepper driver
const int stepper1HomedMaxLimit = 1150; // maximum full step limit after homing has completed
const int stepper1HomedMinLimit = 0; // minimum full step limit after homing has completed
const unsigned long stepperDisableTime = 30000; // time (ms) that the stepper motor remains powered while not in motion
const unsigned long endstopPressTimeForProgramRun = 3000; // time (ms) that the endstop must remain pressed to execute the program

//// Variables ////
byte stepper1Enable = 0;
unsigned long lastStepperMove = 0;


//// Constructs ////
AccelStepper Stepper1(AccelStepper::DRIVER, stepper1StepPin, stepper1DirectionPin, false); // stepper is driven by a stepper driver using inputs for step and direction. outputs are not enabled at construction time


//// Setup ////
void setup() {
  pinMode(endstop1Pin, INPUT_PULLUP); // use internal pullup on the endstop pin, the endstop is wired normally closed between the endstop pin and GND
  
  // Stepper Driver Setup
  Stepper1.setPinsInverted(false, false, true);
  Stepper1.setEnablePin(stepper1EnablePin);
  Stepper1.disableOutputs();
  Stepper1.setMinPulseWidth(stepper1DriverMinimumPulseWidth);
  Stepper1.setMaxSpeed(stepper1Speed*stepper1Microstepping);
  Stepper1.setAcceleration(stepper1Acceleration*stepper1Microstepping);
}


//// Main Loop ////
void loop() {
  if(digitalRead(endstop1Pin)==1) { // check if endstop is pressed, if so, wait the user specified time for program run
    delay(endstopPressTimeForProgramRun);
    if(digitalRead(endstop1Pin)==1) { // if endstop is still pressed, wait 2s
      delay(2000);
      if(digitalRead(endstop1Pin)==0) { // if endstop is no longer pressed, run the preprogramed movements
        // Preprogramed movements
        HomeSteppers(); // home the carriage
        delay(1000); // wait 1s for dramatic effect
        MoveSteppers(1150*stepper1Microstepping); // move the carriage to the position corresponding to 1150 full steps (furthest from the stepper motor)
        delay(500); // wait 0.5s
        MoveSteppers((1150/2)*stepper1Microstepping); // move the carriage to the position corresponding to 1150/2 full steps (half way from the stepper motor)
        delay(500); // wait 0.5s
        MoveSteppers(2000*stepper1Microstepping); // move the carriage to the position corresponding to 2000 full steps
                                                  // (this is further than the length of the rail, software endstops will limit this to the "stepper1HomedMaxLimit")
        }
      }
    }

  if(stepper1Enable==1 && millis()-lastStepperMove>=stepperDisableTime) { // if it has been longer than "stepperDisableTime" since the last stepper movement then disable the stepper motor
    Stepper1.disableOutputs();
  }
}


//// Motion ////
void MoveSteppers(long int stepper1Position) {
  // Maximum travel limit software endstop
  if(stepper1Position > stepper1HomedMaxLimit*stepper1Microstepping) { // limit the movement to the maximum travel limit
    Stepper1.moveTo(stepper1HomedMaxLimit*stepper1Microstepping);
  }
  // Minimum travel limit software endstop
  else if(stepper1Position < stepper1HomedMinLimit*stepper1Microstepping) { // limit the movement to the minimum travel limit
    Stepper1.moveTo(stepper1HomedMinLimit*stepper1Microstepping);
  }
  else {
    Stepper1.moveTo(stepper1Position);
  }
  
  while(Stepper1.distanceToGo() != 0) { // move to the requested position with travel limits applied
    Stepper1.run();
  }
  lastStepperMove = millis(); // store when the most recent stepper movement occured
}


//// Homing ////
void HomeSteppers() {
  byte stepper1Homed = 0; // variable to keep track of homing progress

  // Stepper 1 Homing
  if(stepper1Enable==0) { // make sure the stepper is powered
    Stepper1.enableOutputs();
    stepper1Enable = 1;
  }
  Stepper1.setCurrentPosition(0); // set position to zero
  Stepper1.moveTo((-stepper1HomedMaxLimit-100)*stepper1Microstepping); // set the stepper to move the carriage towards the endstop
  Stepper1.setMaxSpeed(stepper1HomingSpeed*stepper1Microstepping); // set the stepper speed to the homing speed
  while(stepper1Homed==0) { // move the carriage towards the endstop until the endstop is triggered
    Stepper1.run();
    if(digitalRead(endstop1Pin)==1) stepper1Homed=1; // first homing bump is complete
  }
  Stepper1.setCurrentPosition(0); // set position to zero
  MoveSteppers(stepper1HomingBumpDistance*stepper1Microstepping); // bounce off the endstop
  Stepper1.moveTo((-stepper1HomingBumpDistance-25)*stepper1Microstepping); // set the stepper to move the carriage towards the endstop
  Stepper1.setMaxSpeed((stepper1HomingSpeed*stepper1Microstepping)/stepper1HomingBumpDiviser); // set the stepper speed to the homing speed with bump divisor applied
  while(stepper1Homed==1) { // move the carriage towards the endstop until the endstop is triggered
    Stepper1.run();
    if(digitalRead(endstop1Pin)==1) stepper1Homed=2; // second homing bump is complete
  }
  Stepper1.setCurrentPosition(stepper1HomeOffset*stepper1Microstepping); // set the home position
  Stepper1.setMaxSpeed(stepper1Speed*stepper1Microstepping); // revert the stepper speed to "stepper1Speed"
}
