// Arduino requires inclusion of all files used in a project in the ino file.
#include "JJ_MPU6050_DMP_6Axis.h"    
#include <Wire.h>                    
#include <I2Cdev.h>                
#include "Mpu.hpp"                   
#include "NMbot.hpp"                 
#include <PID_v1.h>                 

// Direction and pulse pins for the stepper motors.
// Dir high = clockwise, low = counter-clockwise.
// Pulse should be PWM, 0-255.

#define DIR_PIN_L 8
#define PUL_PIN_L 9
#define DIR_PIN_R 6
#define PUL_PIN_R 7

NMbot bot;

// volatile because changed in interrupts
volatile bool pullRight;
volatile bool pullLeft;

void setup() {
  // Begins Serial comms, initializes and calibrates MPU,
  // Starts PID control.
  bot.setup();
  Wire.begin();

  // Initialize pins
  pinMode(PUL_PIN_L, OUTPUT);
  pinMode(PUL_PIN_R, OUTPUT);
  pinMode(DIR_PIN_L, OUTPUT);
  pinMode(DIR_PIN_R, OUTPUT);

  digitalWrite(DIR_PIN_L, LOW);
  digitalWrite(DIR_PIN_R, HIGH);
}

void loop() {
  // Updates the MPU, feeds new MPU
  //  value to PID, which sets speed and direction of motors.
  bot.loop();
}

// Interrupts on the due fire through TC handlers,
// Called at a frequency specified by setmotorspeed in nmbot.cpp.
// Pulses the left motor.
void TC3_Handler(){
  TC_GetStatus(TC1,0);

  pullLeft = !pullLeft;
  digitalWrite(PUL_PIN_L, pullLeft);
}

// Interrupts on the due fire through TC handlers,
// Called at a frequency specified by setmotorspeed in nmbot.cpp.
// Pulses the left motor.
void TC4_Handler(){
  TC_GetStatus(TC1,1);
  
  pullRight = !pullRight;
  digitalWrite(PUL_PIN_R, pullRight);
}
