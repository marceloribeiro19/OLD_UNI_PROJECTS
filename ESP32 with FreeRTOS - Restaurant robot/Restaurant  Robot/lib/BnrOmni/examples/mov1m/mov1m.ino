/*
 Omni-3MD 3 motor controller from botnroll.com http://www.botnroll.com/en/controllers/84-bot-n-roll-omni-3md-.html

 Move 1 motor using Omni3MD:
 - this code assumes you are using 1,2 or 3 DC motors with encoders attached to Omni3MD board;
 - this example was developed using Arduino 1.7.10 open-source Arduino environment
 - this example is compatible with Omni3MD firmware version 1.74 or higher. Visit http://www.botnroll.com/omni3md/ to check if your version is updated.

 Purpose:
  This example demonstrates linear movimentation of motors:
 
 Send movimentation orders: 
   Function: mov1m( motor, speed)
   - motor: the motor to operate (1, 2 or 3);
   - speed: the desired speed for the movement (-100 to +100). 
   Ex: omni.mov1m(2,35);
   The motor runs at the desired speed, speed=0 stop the motor without holding torque.

 Important routines in this example:
 
      //setup routines
      void i2cConnect(byte omniAddress);
      void setI2cTimeout (byte timeout); //timeout x 100 miliseconds to receive i2C Commands
            
      //movement routines
      void mov1m(byte motor,int speed);

 Adjust the acceleration ramp if necessary:
   Function: setRamp( slope , kl )
   - slope: defines the acceleration of the motors (ramp inclination) varies from 0 to 100. Lower values slower acceleration.
   - kl: factor for the the necessary power to overcome the motor gearbox inertia, obtained during calibration from a system with encoders!
     Power*kl is loaded for the integral error of the PID control, under certain conditions, by the control algorithm.
     Positive values between 0 and 1000 for kl. The value 0 disables this function.
 
 
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
 on 27 September 2012
 Updated on 17 January 2017

 This code example is in the public domain. 
 http://www.botnroll.com
*/
 
#include <Wire.h>                   // required by BnrOmni.cpp
#include <BnrOmni.h>

//constants definitions
#define OMNI3MD_ADDRESS 0x30        // default factory address
#define M1  1    // Motor1
#define M2  2    // Motor2
#define M3  3    // Motor3

BnrOmni omni;    //declaration of object variable to control the Omni3MD


void setup()
{
    //setup routines
    delay(1500);                      // wait electronics to stabilize
    Serial.begin(57600);              // set baud rate to 57600bps for printing values on serial monitor. Press (ctrl+shift+m) after uploading
    omni.i2cConnect(OMNI3MD_ADDRESS); // set i2c connection
    omni.stop();                      // stop all motors
    omni.setI2cTimeout(0);            // safety parameter -> I2C communication must occur every [byte timeout] x 100 miliseconds for motor movement                             
    omni.setRamp(35,900);             // set acceleration ramp and limiar movemet parameter gain[int slope, int Kl] slope between 0 and 100 kl->gain for necessary motor power to start movement
    omni.setMinBat(9.7);              // Battery discharge protection voltage (Lithium 3S)
    
    Serial.print("Firmware:");  
    Serial.println(omni.readFirmware());      // print firmware value
    Serial.print("ControlRate:");
    Serial.println(omni.readControlRate());   // print control rate value
    Serial.print("Enc1max:");
    Serial.println(omni.readEnc1Max());       // print encoder1 maximum calibration value
    Serial.print("Enc2max:");
    Serial.println(omni.readEnc2Max());       // print encoder2 maximum calibration value 
    Serial.print("Enc3max:");
    Serial.println(omni.readEnc3Max());       // print encoder3 maximum calibration value
    Serial.print("Battery:");
    Serial.println(omni.readBattery());       // print battery value
    Serial.print("Temperature:");
    Serial.println(omni.readTemperature());   // print temperature value 
}


void loop()
{
    //Variables for motion control
    int speed1=65;
    int speed2=70;
    int speed3=80;
    
    //Move motors    
    omni.mov1m(M1,speed1);    // move motor1 at speed1
    omni.mov1m(M2,speed2);    // move motor2 at speed2
    omni.mov1m(M3,speed3);    // move motor3 at speed3
    delay(4000);
    
    omni.stop();              // stop all motors
    delay(2000);

    omni.mov1m(M1,-speed3);   // move motor1 at speed1
    omni.mov1m(M2,-speed3);   // move motor2 at speed2
    omni.mov1m(M3,-speed3);   // move motor3 at speed3
    delay(3000);
 
    omni.mov1m(M1,0);         // stop motor (using acceleration ramp)
    omni.mov1m(M2,0);         // stop motor (using acceleration ramp)
    omni.mov1m(M3,0);         // stop motor (using acceleration ramp)
    delay(2000); 
}
