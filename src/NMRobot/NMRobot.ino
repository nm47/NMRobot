//#include <digitalWriteFast.h>

#include "JJ_MPU6050_DMP_6Axis.h"    // |
#include <Wire.h>                    // |
#include <I2Cdev.h>                  // | Arduino (stupidly) demands explicit references to all files used in a project in the ino file...
#include "Mpu.hpp"                   // |
#include "NMbot.hpp"                 // |

#define DIR_PIN_L 9
#define PUL_PIN_L 8
#define DIR_PIN_R 6
#define PUL_PIN_R 7

NMbot bot;

volatile bool pullRight;
volatile bool pullLeft;

void setup() {
  bot.setup();
  Wire.begin();

  pinMode(PUL_PIN_L, OUTPUT);
  pinMode(PUL_PIN_R, OUTPUT);
  pinMode(DIR_PIN_L, OUTPUT);
  pinMode(DIR_PIN_R, OUTPUT);
  
  delay(1000);
}

void loop() {
  bot.loop();
}

void TC3_Handler(){
  TC_GetStatus(TC1,0);

  pullLeft = !pullLeft;
  digitalWrite(PUL_PIN_L, pullLeft);
}

void TC4Handler(){
  TC_GetStatus(TC1,1);
  pullRight = !pullRight;
  digitalWrite(PUL_PIN_R, pullRight);
}
