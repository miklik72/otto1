//----------------------------------------------------------------
//-- Otto DIY invests time and resources providing open source code and hardware,
//-- please support by purchasing kits from https://www.ottodiy.com/
//-- Make sure to have installed all libraries: https://github.com/OttoDIY/OttoDIYLib
//-----------------------------------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Otto.h> //-- Otto Library
Otto otto1;  //This is Otto!

#define LeftLeg 2
#define RightLeg 3
#define LeftFoot 4
#define RightFoot 5
#define Buzzer  13

double angle_rad = PI/180.0;
double angle_deg = 180.0/PI;
int YL;
int YR;
int RL;
int RR;

void printCalibration() {
  Serial.print("Y: ");
  Serial.print(YL);
  Serial.print(" ");
  Serial.print(YR);
  Serial.print(" ");
  Serial.print("R: ");
  Serial.print(RL);
  Serial.print(" ");
  Serial.println(RR);
}

void calib_homePos() {
  int servoPos[4];
  servoPos[0]=90;
  servoPos[1]=90;
  servoPos[2]=90;
  servoPos[3]=90;
  otto1._moveServos(500, servoPos);
  otto1.detachServos();
  printCalibration();
}



void setup(){
    otto1.init(LeftLeg, RightLeg, LeftFoot, RightFoot, true, Buzzer); //Set the servo pins and Buzzer pin
    Serial.begin(115200);
    YL = EEPROM.read(0);
    if (YL > 128) YL -= 256;
    YR = EEPROM.read(1);
    if (YR > 128) YR -= 256;
    RL = EEPROM.read(2);
    if (RL > 128) RL -= 256;
    RR = EEPROM.read(3);
    if (RR > 128) RR -= 256;
    otto1.sing(S_connection);
    otto1.home();
    Serial.println("Start");

    otto1.walk(3);
    otto1.home();
    otto1.sing(S_happy);
    delay(2000);
    otto1.turn(4,2000,LEFT);
    otto1.home();
    otto1.sing(S_happy);
    delay(2000);
    otto1.turn(4,1000,RIGHT);
    otto1.home();
    otto1.sing(S_happy);
    delay(2000);
    otto1.bend();
    otto1.home();
    otto1.sing(S_happy);
    delay(2000);
    otto1.shakeLeg();
    otto1.home();
    otto1.sing(S_happy);
    delay(2000);
    otto1.updown();
    otto1.home();
    otto1.sing(S_happy);

}

void loop(){
    
}