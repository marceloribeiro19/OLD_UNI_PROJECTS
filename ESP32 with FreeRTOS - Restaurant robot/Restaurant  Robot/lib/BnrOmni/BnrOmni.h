/*
  BnrOmni.h - Library for interfacing with Omni-3MD (3 motor driver controller) from www.botnroll.com
  This library is compatible with Omni-3MD firmware 1.74 (released January 2017) and superior.
  Created by Nino Pereira, April 28, 2011.
  Updated by José Cruz on 17 January 2017
  Released into the public domain.
*/

#ifndef BnrOmni_h
#define BnrOmni_h

#include "Arduino.h"

#define KEY1 0xAA // key used in critical commands
#define KEY2 0x55 // key used in critical commands


    /*User Commands*/
    /*Read Firmware version*/
    #define COMMAND_FIRMWARE_INT		0xFE //Read firmware value (integer value)
    #define COMMAND_FIRMWARE_DEC		0xFD //Read firmware value (decimal value)
    /*Write Commands->Don't require response from Omni3MD */
    #define COMMAND_STOP				0xFC //Stop motors
    #define COMMAND_CALIBRATE           0xFB //Normal moving robot Calibration
    #define COMMAND_MOV_OMNI            0xFA //Omnidirectional movement
    #define COMMAND_MOV_LIN3M_PID       0xF8 //Linear independent motion of 3 motors with PID control
    #define COMMAND_MOV_LIN3M_NOPID     0xF7 //Linear independent motion of 3 motors without PID control
    #define COMMAND_MOV_LIN1M_PID       0xF6 //Linear independent motion of 1 motor with PID control
    #define COMMAND_MOV_LIN1M_NOPID     0xF5 //Linear independent motion of 1 motor without PID control
    #define COMMAND_GAIN_CFG_SPEED		0xF4 //Configuration of kp, ki and kd parameters for speed PID control
    #define COMMAND_I2C_ADD				0xF2 //Change Omni3MD I2C address
    #define COMMAND_TIMEOUT_I2C         0xF1 //Configuration of the break communication protection timeout
    #define COMMAND_PRESCALER_CFG       0xF0 //Set the encoders prescaler values
    #define COMMAND_POS_ENC_PRESET      0xEF //Preset/reset the encoders count
    #define COMMAND_MOV_POS				0xEE //Positional movement
    #define COMMAND_SAVE_POS			0xED //Save the encoders counting to Omni3MD eeprom
    #define COMMAND_RAMP_CFG            0xEC //Configuration of the acceleration ramp and limiar take off parameter
    #define COMMAND_MOV_DIF_SI          0xEB //Differential movement with SI units (m/s, rad/s)
    #define COMMAND_DIF_SI_CFG          0xEA //Configuration of the differential movement with SI units
    #define COMMAND_BAT_MIN_CFG         0xE9 //Configuration of the minimum battery voltage to stop motors
    #define COMMAND_FIND_HOME_POS       0xE7 //Positional system find home position
    #define COMMAND_GAIN_CFG_POS		0xE6 //Configuration of kp, ki and kd parameters position for PID control

/*Read Commands-> requests to Omni3MD */
    #define COMMAND_DUTY_LIM_M1         0xDB //Leitura do valor do PWM limiar para o Motor1
    #define COMMAND_DUTY_LIM_M2         0xDA //Leitura do valor do PWM limiar para o Motor2
    #define COMMAND_DUTY_LIM_M3         0xD9 //Leitura do valor do PWM limiar para o Motor3
    #define COMMAND_ENC1_INC			0xD8 //Read encoder1 positional value
    #define COMMAND_ENC2_INC            0xD7 //Read encoder2 positional value
    #define COMMAND_ENC3_INC			0xD6 //Read encoder3 positional value
    #define COMMAND_BAT                 0xD5 //Read battery voltage
    #define COMMAND_TEMP				0xD4 //Read Omni3MD temperature
    #define COMMAND_CTRL_RATE			0xD3 //Read the PID control rate
    #define COMMAND_ENC1_MAX			0xD2 //Read encoder1 maximum value at calibration
    #define COMMAND_ENC2_MAX			0xD1 //Read encoder1 maximum value at calibration
    #define COMMAND_ENC3_MAX            0xD0 //Read encoder1 maximum value at calibration


class BnrOmni
{
  public:
        //setup routines
        void i2cConnect(byte omniAddress);
        void setI2cAddress (byte newAddress);
        void setI2cTimeout (byte timeout); //timeout x 100 miliseconds to receive i2C Commands
        void calibrate(boolean way1,boolean way2,boolean way3);
        void setPid(int Kp, int Ki, int Kd);
        void setPositionalPid(int Kp, int Ki, int Kd);
		void setRamp(int slope, int Kl);
        void setEncValue(byte encoder, unsigned int encValue);
        void setEncPrescaler(byte encoder, byte value);
        void setDifferential(double axis_radius, double whell_radius, double gearbox_factor, double encoder_cpr);
        void setMinBat(float minbat);

        //reading routines
        float readTemperature();
        float readBattery();
        float readFirmware();
        void readFirmware(byte*,byte*,byte*);
        byte readControlRate();
        int readEnc1();
        int readEnc2();
        int readEnc3();
        int readEnc1Max();
        int readEnc2Max();
        int readEnc3Max();
        int readLim1();
        int readLim2();
        int readLim3();
        void readEncoders(int*,int*,int*);
        void readEncoders(unsigned int*,unsigned int*,unsigned int*);
        void readMovData(int*,int*,int*,float*,float*);
        void readAllData(int*,int*,int*,float*,float*,byte*,byte*,byte*,byte*,int*,int*,int*,int*,int*,int*);
        int readDBG(int);/*FOR botnroll.com DEBUG PURPOSES ONLY*/
        //movement routines
        void movOmni(byte linear_speed,int rotational_speed,int direction);
        void movDifSi(double linear_speed,double rotational_speed);
        void movPosition(byte motor,unsigned int speed,unsigned int encPosition);
        void mov3mPid(int speed1,int speed2,int speed3);
        void mov1mPid(byte motor,int speed);
        void mov3m(int speed1,int speed2,int speed3);
        void mov1m(byte motor,int speed);
        void stop();
        void savePosition();
        void findHome(byte motor);
    
  private:
        byte _omniAddress;
        byte i2cRequestByte(byte addressValue, byte command);
        int  i2cRequestWord(byte addressValue, byte command);
        void i2cRequestData(byte adreessValue, byte command);
        void i2cSendData(byte addressValue, byte command, byte buffer[], byte numBytes);
};
#endif


