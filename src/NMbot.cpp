#include "NMbot.hpp"
#include "MPU6050.h"
#include "mpu.hpp"
#include <PID_v1.h>

mpu gyro;
double kp = 11;
double ki = 5;
double kd = 1;
double setPoint = 0;
double currentSpeed = 0;
double pidOutputSpeed;
double input;
PID pid = PID (&input, &pidOutputSpeed, &setPoint, kp, ki, kd, DIRECT);
void NMbot::setup(){
    Serial.begin(115200);
    gyro.setup();
    
    pid.SetOutputLimits(-1000,1000);
    startTimer(TC1, 0, TC3_IRQn, 1000); //TC1 channel 0, the IRQ for that channel and the desired frequency
    startTimer(TC1, 1, TC4_IRQn, 1000);
}

void NMbot::loop(){
    if(motion.tick()){
    }
}
void NMbot::CalculateSpeed(){
    input = gyro.angle;
    pid.Compute();
    currentSpeed = pidOutputSpeed;

    Serial.print(",");
    Serial.println(pidOutputSpeed);

    SetMotorSpeed(0,pidOutputSpeed);
    pidOutputSpeed = map(mspeed, -300,300, -1000,1000);
    pidOutputSpeed = constrain(mspeed, -1000,1000);
    
}

void NMbot::SetMotorSpeed(int motor, double mSpeed){  //0 is left, 1 is right, motors are mounted in opposite directions, therefore rotate opposite dirs.
  
  if (motor == 0){
    startTimer(TC1, 0, TC3_IRQn, &pidOutputSpeed);
  }
  else {
    startTimer(TC1, 1, TC4_IRQn, pidOutputSpeed);
  }
}

//Initializes SAM3X8E wave generator at 50% duty cycle, triggers interupt on rising edge of the wave
void NMbot::startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
    pmc_set_writeprotect(false);
    pmc_enable_periph_clk((uint32_t)irq);
    TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
    uint32_t rc = VARIANT_MCK / 128 / frequency; //128 because we selected TIMER_CLOCK4 above
    TC_SetRA(tc, channel, rc / 2); //50% high, 50% low
    TC_SetRC(tc, channel, rc);
    TC_Start(tc, channel);
    tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
    tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
    NVIC_EnableIRQ(irq);
}
