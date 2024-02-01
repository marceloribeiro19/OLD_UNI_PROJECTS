#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#ifdef __AVR__
  #include <avr/power.h>            //LEDS1
#endif

#include <iostream>
#include <Arduino.h>
#include <math.h>
#include <BnrOmni.h>
#include <Wire.h>                   //required by BnrOmni.cpp
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <BluetoothSerial.h>
#include <Adafruit_NeoPixel.h>     //LEDS2  



//LEDS
#define PIN 27
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800);


//Bluetooth
BluetoothSerial ESP_BT;
int incoming;

//constants definitions
#define OMNI3MD_ADDRESS 0x30        //default factory address
#define M1  1  //Motor1
#define M2  2  //Motor2
#define M3  3  //Motor3
BnrOmni omni; 

const uint32_t Bit_Sensor = (1<<0); //0x01
EventGroupHandle_t EventGroupHandle = 0;

const int Button = 14;
const int FAN = 12;
const int trigPin = 16;
const int echoPin = 17;
#define SOUND_SPEED 0.034
//sensor distancia
long duration;
float distanceCm;
int return_distance = 0;
uint8_t obstaculodetetado=0;
uint8_t resume_mov = 0;
uint8_t return_mov = 0;
uint8_t P_FIRST_CYCLE=1;
uint8_t stop = 0;

int enc1old=0, enc2old=0, enc3old=0;
double RobotX = 0;
double RobotY = 0;
double RobotXcm = 0;
double RobotYcm = 0;
double RobotTeta = 0;
double PosicaoX=0;
double PosicaoY=0;
double diamRoda=5.95; // Diametro da roda
double d = 12.25;//8.9; // distancia duma roda ao centro do robot
double delta_t = 0.100;  // Periodo de amostragem em segundos
double KWheels  = (1/delta_t)*(M_PI*diamRoda)/220;

double RobotRelX    = 0.0;
double RobotRelY    = 0.0;
double RobotRelTeta = 0.0;

uint8_t RunningFlag; //Garante que nada acontece quando se encontra a 0
typedef enum {ROT_DEFAULT, ROT_START, ROT_STOP, ROT_TABLE1,ROT_TABLE2, ROT_TABLE3, ROT_TABLE4}states;
typedef enum {EV_DEFAULT, EV_Charging, EV_READY, EV_MOVING, EV_ROTATING_CLOCK, EV_ROTATING_COUNTERCLOCK, EV_OBJECT, EV_TABLE, EV_PAUSE}events;
states State = ROT_DEFAULT;
events event = EV_DEFAULT;

void Localizacao();
double NormalizeAngle(double angulo);
void Localizacao_Encoders();

void BlueToothControl(void *parameter);
void Movement(void *parameter);
void SensorObstaculos (void *parameter);
void SensorTemperatura(void *parameter);
void LEDcontroll(void *parameter);

void colorWipeCounterClockWise(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<=strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    vTaskDelay(wait);
  }
}
void colorWipeClockWise(uint32_t c, uint8_t wait) {
  for(uint16_t i=strip.numPixels(); i>=0; i--) {
    strip.setPixelColor(i, c);
    strip.show();
    vTaskDelay(wait);
    if(i==0)break;
  }
}
void SetColor(uint32_t c) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
}


void resetencoders(){
  omni.setEncValue(M1,0);
  omni.setEncValue(M2,0);
  omni.setEncValue(M3,0);
  RobotX = 0;
  RobotY = 0;
  RobotXcm = 0;
  RobotYcm = 0;
  RobotTeta = 0;
  enc1old=0;
  enc2old=0;
  enc3old=0;
}

void rotate(int Angulo){
  int lin_speed=0;
  int dir=0;
  int enc1=0,enc2=0,enc3=0;
  int new_enc1, new_enc2, new_enc3;

  new_enc1 = (abs(Angulo) * 807)/90;
  new_enc2 = (abs(Angulo) * 800)/90;
  new_enc3 = (abs(Angulo) * 800)/90;
  resetencoders();

  if(Angulo > 0)
  {
    int rot_speed=30;
    omni.movOmni(lin_speed,rot_speed,dir);
    while(enc1<new_enc1 || enc2<new_enc2 || enc3<new_enc3){
      event = EV_ROTATING_CLOCK;
      vTaskDelay(20/portTICK_PERIOD_MS);
      omni.readEncoders(&enc1,&enc2,&enc3);
    }
  omni.movOmni(0,0,0);
  resetencoders();
  vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  else
  {
    int rot_speed=-30;
    vTaskDelay(50/portTICK_PERIOD_MS);
    omni.setEncValue(M1,new_enc1);
    omni.setEncValue(M2,new_enc2);
    omni.setEncValue(M3,new_enc3);

    vTaskDelay(50/portTICK_PERIOD_MS);
    omni.movOmni(lin_speed,rot_speed,dir);
    vTaskDelay(50/portTICK_PERIOD_MS);

    while(enc1<new_enc1 || enc2<new_enc2 || enc3<new_enc3){
      event = EV_ROTATING_COUNTERCLOCK;
      vTaskDelay(50/portTICK_PERIOD_MS);
      omni.readEncoders(&enc1,&enc2,&enc3);
    }
  omni.movOmni(0,0,0);
  resetencoders();
  vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void movdistance(double distance){
  resetencoders();
  xEventGroupSetBits(EventGroupHandle,Bit_Sensor);
  int lin_speed=50;
  int rot_speed=0;
  int dir=0;
  omni.movOmni(lin_speed,rot_speed,dir);    //move motors (linspeed,rotspeed,direcao)
  while(RobotYcm>-distance ){
   if (!obstaculodetetado && stop == 0)event = EV_MOVING;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Localizacao();
    Serial.println(RobotXcm);
    Serial.println(RobotYcm);

    if(obstaculodetetado) {
      if(event == EV_MOVING) event = EV_OBJECT;
      omni.movOmni(0,0,0);
    }
    if(stop == 1){
      if(event == EV_MOVING) event = EV_PAUSE;
      omni.movOmni(0,0,0);
    }
    
   /* if(obstaculodetetado || stop == 1){
      omni.movOmni(0,0,0);
    }*/
    if(resume_mov)
    {
      resume_mov = 0;
      omni.movOmni(lin_speed,rot_speed,dir);    //move motors (linspeed,rotspeed,direcao)
    }
  }
  omni.movOmni(0,0,0);
  resetencoders();
  vTaskDelay(500 / portTICK_PERIOD_MS);
}
void Reverse_movdistance(double distance){
  resetencoders();
  xEventGroupSetBits(EventGroupHandle,Bit_Sensor);
  int lin_speed=50;
  int rot_speed=0;
  int dir=180;
  omni.movOmni(lin_speed,rot_speed,dir);    //move motors (linspeed,rotspeed,direcao)
  while(RobotYcm<distance ){
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Localizacao();
    Serial.println(RobotXcm);
    Serial.println(RobotYcm);
    if(stop == 1){
      omni.movOmni(0,0,0);
    }
    if(resume_mov)
    {
      resume_mov = 0;
      omni.movOmni(lin_speed,rot_speed,dir);    //move motors (linspeed,rotspeed,direcao)
    }
  }
  omni.movOmni(0,0,0);
  resetencoders();
  vTaskDelay(500 / portTICK_PERIOD_MS);
}

void setup() {
  Serial.begin(57600);             // set baud rate to 57600bps for printing values on serial monitor. Press (ctrl+shift+m) after uploading
  
  //BlueTooth
  ESP_BT.begin("ESP32_Control");

  //GPIO
  delay(1500);
  pinMode(FAN, OUTPUT);     // Sets the Fan pin as an Output
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
  pinMode(Button,INPUT);    // Sets the Button as an Input

  //OMNI INIT
  omni.i2cConnect(OMNI3MD_ADDRESS);// set i2c connection
  omni.stop();                     // stop all motors
  omni.setI2cTimeout(0);           // safety parameter -> I2C communication must occur every [byte timeout] x 100 miliseconds for motor movement                             
  omni.setPid(1000,400,500);       // adjust paramenters for Speed PID control [word Kp, word Ki, word Kd]
  omni.setRamp(35,900);            // set acceleration ramp and limiar movemet parameter gain[int slope, int Kl] slope between 0 and 100 kl->gain for necessary motor power to start movement
  omni.setMinBat(9.7);             // battery discharge protection voltage (Lithium 3S)
  omni.setEncPrescaler(M1, 1);     // set the prescaler to 1; encoder count will increment every 1 pulse [byte encoder, byte value]
  omni.setEncPrescaler(M2, 1);     // set the prescaler to 1; encoder count will increment every 1 pulse [byte encoder, byte value]
  omni.setEncPrescaler(M3, 1);     // set the prescaler to 1; encoder count will increment every 1 pulse [byte encoder, byte value]
  resetencoders();

  //LEDSTRIP INIT
  strip.begin();
  strip.setBrightness(50);
  strip.show();                             // Initialize all pixels to 'off'
  colorWipeClockWise(strip.Color(255, 165, 0), 50);  // orange wipe
  event = EV_Charging;
 

  //Task and Event group creation
  EventGroupHandle = xEventGroupCreate();

  xTaskCreatePinnedToCore(SensorObstaculos, "distancia de obstaculos", 1024*5, NULL, 3, NULL,ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(BlueToothControl, "Controlo do BT", 1024*5, NULL, 3, NULL,ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(Movement, "Movimento do Robô", 1024*5, NULL, 3, NULL,ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(LEDcontroll, "Controlo dos Leds", 1024*5, NULL, 3, NULL,ARDUINO_RUNNING_CORE);
  xTaskCreatePinnedToCore(SensorTemperatura, "Sensor de temperatura", 1024*5, NULL, 1, NULL,ARDUINO_RUNNING_CORE);
}

void BlueToothControl(void *parameter){
  (void) parameter;
  for(;;){
  
      if (ESP_BT.available()){

        incoming = ESP_BT.read();
        switch (incoming){
          case 0:         //STOP
            RunningFlag = 0;
            State = ROT_STOP;
            event = EV_PAUSE;
            stop = 1;
            break;
          case 1:         //START
            Serial.println("START");
            RunningFlag = 1;
            State = ROT_START;
            event = EV_READY;
            stop = 0;
            resume_mov = 1;
            break;
          case 2:         //TABLE1
            if(RunningFlag){
             State = ROT_TABLE1;
            }
            break;
          case 3:         //TABLE2
             if(RunningFlag){
              State = ROT_TABLE2;
            }
            break;
          case 4:         //TABLE3
            if(RunningFlag){
              State = ROT_TABLE3;
            }
            break;   
          case 5:         //TABLE4
            if(RunningFlag){
              State = ROT_TABLE4;
            }
            break;              
          default:

            break;
      }
    }
    vTaskDelay(200);  //?????????????????????????
  }
}

void Movement(void *parameter){
  (void) parameter;
  for(;;){
    switch (State){
     
      case ROT_TABLE1:
        movdistance(70);
        rotate(-90);
        movdistance(280);
        rotate(-90);
        movdistance(40);
        event = EV_TABLE;
        //---------DEBOUNCE-----------
        while(digitalRead(Button) == 0);
        while(digitalRead(Button) == 1);
        //----------------------------
        rotate(180);
        movdistance(40);
        rotate(90);
        movdistance(280);
        rotate(90);
        movdistance(40);
        rotate(180);
        Reverse_movdistance(30);
        event = EV_Charging;
        break;
      case ROT_TABLE2:
        movdistance(70);
        rotate(-90);
        movdistance(280);
        rotate(90);
        event = EV_TABLE;
        //---------DEBOUNCE-----------
        while(digitalRead(Button) == 0);
        while(digitalRead(Button) == 1);
        //-----------------------------
        rotate(90);
        movdistance(280);
        rotate(90);
        movdistance(40);
        rotate(180);
        Reverse_movdistance(30);
        event = EV_Charging;
        break;
      case ROT_TABLE3:
        movdistance(70);
        rotate(-90);
        movdistance(480);
        rotate(-90);
        movdistance(40);
        event = EV_TABLE;
        //---------DEBOUNCE-----------
        while(digitalRead(Button) == 0);
        while(digitalRead(Button) == 1);
        //----------------------------
        rotate(180);
        movdistance(40);
        rotate(90);
        movdistance(480);
        rotate(90);
        movdistance(40);
        rotate(180);
        Reverse_movdistance(30);
        event = EV_Charging;
        break;
      case ROT_TABLE4:
       /* movdistance(70);
        rotate(-90);
        movdistance(480);
        rotate(90);
        event = EV_TABLE;
        //---------DEBOUNCE-----------
        while(digitalRead(Button) == 0);
        while(digitalRead(Button) == 1);
        //----------------------------
        rotate(90);
        movdistance(480);
        rotate(90);
        movdistance(40);
        rotate(180);
        Reverse_movdistance(30);*/
        event = EV_Charging;
        break;
      default:

        break;
    }
    State = ROT_DEFAULT;
    xEventGroupClearBits(EventGroupHandle,Bit_Sensor);
    vTaskDelay(10);
  } 
}

void SensorObstaculos (void *parameter) {
  (void) parameter;
  for(;;){
    xEventGroupWaitBits(EventGroupHandle,Bit_Sensor,pdFALSE,pdFALSE,portMAX_DELAY);
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
  
    // Calculate the distance
    distanceCm = duration * SOUND_SPEED/2;

    // Prints the distance in the Serial Monitor
    Serial.print("Distance (cm): ");
    Serial.println(distanceCm);
    if(distanceCm<25 && distanceCm>1){
      obstaculodetetado=1;
      if(event == EV_MOVING) event = EV_OBJECT;
    }
    if(obstaculodetetado && distanceCm > 30){
      obstaculodetetado=0;
      resume_mov = 1;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

#define MaxTemp 45.0
void SensorTemperatura(void *parameter){
  (void) parameter;
  for(;;){
    xEventGroupWaitBits(EventGroupHandle,Bit_Sensor,pdFALSE,pdFALSE,portMAX_DELAY);
    float Temp = omni.readTemperature();
    vTaskDelay(50);

    if( Temp > MaxTemp) digitalWrite(FAN,1); // Se a temperatura for maior que 60 graus liga a ventoinha
    else digitalWrite(FAN,0);
    
    Serial.print("Temperatura: ");
    Serial.println(Temp);
    vTaskDelay(5000);
  }
}

void LEDcontroll(void *parameter){
  (void) parameter;
  volatile int Brightness = 50;
  for(;;){
    switch (event){
          case EV_Charging:         //EVENT Charging
            SetColor(strip.Color(255, 127, 0));
            if(Brightness == 0) Brightness=50;
            strip.setBrightness(Brightness);
            Brightness = Brightness - 1;
            vTaskDelay(20); //Demora 1segundo a dar DIM completamente
            break;
          case EV_READY:             //EVENT Ready
            SetColor(strip.Color(0, 0, 255));   //DIM BLUE
            if(Brightness == 0) Brightness=50;
            strip.setBrightness(Brightness);
            Brightness = Brightness - 1;
            vTaskDelay(20); //Demora 1segundo a dar DIM completamente
            break;
          case EV_MOVING:           //EVENT Moving
            SetColor(strip.Color(0, 0, 255));
            break;
          case EV_ROTATING_CLOCK:         //EVENT Rotating
            colorWipeClockWise(strip.Color(0, 0, 255), 50);  // blue wipe
            colorWipeClockWise(strip.Color(0, 0, 0), 50);  // off wipe
            break;
          case EV_ROTATING_COUNTERCLOCK:
            colorWipeCounterClockWise(strip.Color(0, 0, 255), 50);  // blue wipe
            colorWipeCounterClockWise(strip.Color(0, 0, 0), 50);  // off wipe
            break;
          case EV_OBJECT:           //EVENT Object detected
            SetColor(strip.Color(255, 0, 0));
            vTaskDelay(400);
            SetColor(strip.Color(0,0,0));
            vTaskDelay(400);
            break;
          case EV_PAUSE:            //EVENT Pause
            SetColor(strip.Color(255, 255, 0)); //YELLOW COLOR
            break;   
          case EV_TABLE:            //EVENT Charging
            colorWipeClockWise(strip.Color(57, 255, 20), 50);  // green wipe
            colorWipeClockWise(strip.Color(0, 0, 0), 50);  // off wipe
            break;              
          default:
          break;

  }
   // vTaskDelay(50);
  }
}

void loop() {
  
}


void Localizacao()
{
  Localizacao_Encoders();
  //Localizacao_Matriz_Linha();
  //Localizacao_Lidar();
  
 // RobotX = RobotX * cos (ANG_BUSSOLA*PIf/180);
 // RobotY = RobotY * sin (ANG_BUSSOLA*PI/180);
  
}

double NormalizeAngle(double angulo)
{
       while (angulo >  M_PI) angulo -= M_PI;
       while (angulo < -M_PI) angulo += M_PI;
return angulo;
}

void Localizacao_Encoders()

{

// Este bloco pode ser transformado em constantes, pus como static (calcula uma vez)

static double theta=M_PI/3;
static double st = 0;
static double ct = 0;
double v1, v2, v3, v, vn, w, halfTeta;    // Variaveis Temporarias
int enc1, enc2, enc3;
st = sin(theta);
ct = cos(theta);

  omni.readEncoders(&enc1,&enc2,&enc3); // reacd all encoders at once (in a single I2C request) //função da biblioteca BnrOmni
  if (enc1>32767)  enc1-=65536;
  if (enc2>32767)  enc2-=65536;
  if (enc3>32767)  enc3-=65536;

  v1 = (enc1-enc1old) * KWheels; //KWheels È a constante que passa ticks do encoder para velocidade em m/s
  v2 = (enc2-enc2old) * KWheels;
  v3 = (enc3-enc3old) * KWheels;

  v  = (- v1 + v3) / (2 * st);
  vn = (v1 - 2 * v2 + v3) / (2 *(ct + 1));
  w  = (v1 + 2 * ct * v2 + v3) / (2 * d * (ct + 1));

  halfTeta = NormalizeAngle(w * delta_t * 0.5); // funcao que poe o angulo entre -pi e pi

  // Integraca numerica com diferenca centrada para o angulo
  /*
  RobotRelX = (cos(halfTeta) * v - sin(halfTeta) * vn) * delta_t;
  RobotRelY = (sin(halfTeta) * v + cos(halfTeta) * vn) * delta_t;
  RobotRelTeta = NormalizeAngle(w * delta_t);

  RobotX += RobotRelX;
  RobotY += RobotRelY;
  RobotTeta = NormalizeAngle(RobotTeta + w * delta_t);
  */
  
  RobotRelY = (cos(halfTeta) * v - sin(halfTeta) * vn) * delta_t;
  RobotRelX = (sin(halfTeta) * v + cos(halfTeta) * vn) * delta_t;
  RobotRelTeta = NormalizeAngle(w * delta_t);

  RobotRelY*=-1;

  RobotX += RobotRelX;
  RobotY += RobotRelY; // ESTAVA ASSIM     RobotY += RobotRelY;
  RobotTeta = NormalizeAngle(RobotTeta + w * delta_t);
  RobotXcm = RobotX * 43 / 157.44;
  RobotYcm = RobotY * 43 / 157.44;
  
  enc1old=enc1; enc2old=enc2; enc3old=enc3;

//  if (DEBUGflag[GPS])    print_debug(GPS, 3, (int)RobotX, (int)RobotY, (int)RobotTeta);
}

