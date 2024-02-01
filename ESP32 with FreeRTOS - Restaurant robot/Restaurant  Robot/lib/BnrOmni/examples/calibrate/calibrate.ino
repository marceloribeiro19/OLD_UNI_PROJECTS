/*
Omni-3MD 3 motor controller from botnroll.com http://www.botnroll.com/en/controllers/84-bot-n-roll-omni-3md-.html

Calibrate the Omni-3MD:
 - this code assumes you are using 1,2 or 3 DC motors with encoders attached to Omni3MD board;
 - this example was developed using Arduino 1.7.10 open-source Arduino environment
 - this example is compatible with Omni3MD firmware version 1.74 or higher. Visit http://www.botnroll.com/omni3md/ to check if your version is updated.

Purpose:
 This example explains the calibration routine and all necessary setup for the different types of movement.
 Calibrating for Omnidirectional (Holonomic) movement and Differential movement is not the same:
 In Omnidirectional movement calibration, all motors must make the robot rotate counter clockwise (CCW).
 For Differential movement calibration, motor1 and motor3 must make the robot move forward.
 The calibration command uses three parameters, one for each motor. 0 and 1 are the possible values for the parameters.
 0 makes the motor rotate in one direction (CW for example), and 1 makes the motor rotate the other way around (CCW).
 Each parameter defines the positve direction of the respective motor. Input the parameters according to your system.

 Omni3MD board features: 
 - Holonomic movement integrated control for 3 motors with PID control;
 - Diferencial movement (SI units) with PID control;
 - Linear movement of 3 independent motors with or without PID control;
 - Position control of 3 motors using motor encoders.
 - Encoder reading;
 - Battery reading;
 - Temperature reading;
 
 The circuit:
 * Omni3MD board attached to Arduino analog input 4 (SDA) and 5 (SCL), GND and 5V DC.
 
 This example was created by José Cruz (www.botnroll.com)
 on 26 September 2012
 Updated on 17 January 2017

 This code example is in the public domain. 
 http://www.botnroll.com
*/
   
#include <Wire.h>                   //required by BnrOmni.cpp
#include <BnrOmni.h>

//constants definitions
#define OMNI3MD_ADDRESS 0x30        //default factory address

BnrOmni omni;                       //declaration of object variable to control the Omni3MD

void setup()
{
  //setup routines
    delay(1500);                       // wait electronics to stabilize
    omni.i2cConnect(OMNI3MD_ADDRESS);  // set i2c connection
    omni.stop();                       // stop all motors at once
    omni.setMinBat(9.7);               // Battery discharge protection voltage (Lithium 3S)
    omni.calibrate(1,1,1);             // send the calibration command with parameters, to configure the OMmni3MD board
    delay(20000);                      // wait 20s for calibration to end
  /*
  Important:
    For Omnidirectional movement calibration, all motors must make the robot rotate counter clockwise (CCW).
    For Differential movement calibration, motor1 and motor3 must make the robot move forward.
    Adjust the calibration routine parameters to acheive the necessary requirements for each type of calibration.
  */      
}

void loop()
{
  
}
