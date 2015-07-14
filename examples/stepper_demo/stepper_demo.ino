/*
PicoStepper demo

The PicoStepper uses the PCA9555 to drive each of the A4988 v2 boards.

The connections from the A4988 to the PCA9555 are as follows:

Motor 1:
========

A4988       PCA9555
-----       ---------
STEP        I/O 0
DIR         I/O 1
/EN         I/O 2
MS1         I/O 3
MS2         I/O 4
MS3         I/O 5

Motor 2:
========

A4988       PCA9555
-----       ---------
STEP        I/O 8
DIR         I/O 9
/EN         I/O 10
MS1         I/O 11
MS2         I/O 12
MS3         I/O 13

Pins for End-Stop/and/or/Switches are pulled high via a 10K resistor
and are mapped as follows:

E1 = I/O 6
E2 = I/O 7
E3 = I/O 14
E4 = I/O 15

See Piccolino_STEP.h for available functions and documentation.

*/   

#include <Wire.h>
#include <Piccolino_STEP.h>
#include <Piccolino_OLED.h>

Piccolino_STEP X;
Piccolino_STEP Y;

Piccolino_OLED oled;

void setup()
{
  pinMode(3,INPUT);
  pinMode(4,INPUT);
  pinMode(8,INPUT);

  oled.begin();
  oled.println("Initializing ...");
  oled.update();
  
  X.begin(0x20,MOTOR1); // 0x20 is the default address (no solder-jumpers on A0,A1,A2)
  Y.begin(0x20,MOTOR2); // I2C address, MOTOR1 or MOTOR2 (2 stepper motors per board)
  
  X.setSpeed(700); // 700 = Max Step Speed via I2C is about 700
  X.setAcceleration(200000);  // basically, no acceleration
  X.setCurrentPosition(0);  //start from a known position
  X.setStep(STEP_FULL);

  Y.setSpeed(700);
  Y.setAcceleration(200000);  
  Y.setCurrentPosition(0);  
  Y.setStep(STEP_FULL);
}



void updateOLED()
{
 oled.setTextSize(2);
 oled.clear(); 
 oled.setCursor(0,0);
 oled.print("X: ");
 oled.print(X.currentPosition());
 oled.setCursor(0,20);
 oled.print("Y: ");
 oled.print(Y.currentPosition());
 oled.update();
}


void moveX(int pos)
{
  X.enable(); 
  X.moveTo(pos); 
  X.runToPosition();
  X.disable();
  updateOLED();
}

void moveY(int pos)
{
  Y.enable(); 
  Y.moveTo(pos); 
  Y.runToPosition();
  Y.disable();
  updateOLED();
}

void moveBoth(int posX, int posY)
{
  char done=0;
  
  X.enable(); 
  X.moveTo(posX); 
  Y.enable(); 
  Y.moveTo(posY); 
  
  while(X.currentPosition()!=posX||Y.currentPosition()!=posY) {
    if(X.currentPosition()!=posX)
      X.run();
        
    if(Y.currentPosition()!=posY)
      Y.run();
  }
  X.disable();
  Y.disable();
  updateOLED();
}

void loop()
{

   X.setStep(STEP_FULL);
   Y.setStep(STEP_FULL);

   moveY(500);
   moveX(500);
   delay(100);

   moveY(0);
   moveX(0);
   delay(100);
 
   moveBoth(500,500);
   delay(100);
   
   moveBoth(0,0);
   delay(100);
   
 
   X.setStep(STEP_HALF);
   Y.setStep(STEP_HALF);
   moveY(500);
   moveX(500);
   delay(100);
   moveY(0);
   moveX(0);
   delay(100);
  
   X.setStep(STEP_QUARTER);
   Y.setStep(STEP_QUARTER);
   moveY(1000);
   moveX(1000);
   delay(100);
   moveY(0);
   moveX(0);
   delay(100);

   X.setStep(STEP_EIGHT);
   Y.setStep(STEP_EIGHT);
   moveY(2000);
   moveX(2000);
   delay(100);
   moveY(0);
   moveX(0);
   delay(100);
  
   X.setStep(STEP_SIXTEEN);
   Y.setStep(STEP_SIXTEEN);
   moveY(4000);
   moveX(4000);
   delay(100);
   moveY(0);
   moveX(0);
   delay(100);
  
}
