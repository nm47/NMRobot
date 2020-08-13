#include "NMbot.hpp"
#include "MPU6050.h"
#include "mpu.hpp"
#include <PID_v1.h>

#define DIR_PIN_L 8  //Due Pins for the drivers
#define PUL_PIN_L 9
#define DIR_PIN_R 6
#define PUL_PIN_R 7

//Uses an MPU6050 Accel/Gyro.
mpu motion;

//Microstepping at 1/32 to allow fine movement of the motors
int microstep = 32;
double kp = 3.7;
double ki = 0;
double kd = .4;
double setPoint = -1.22; //The balancing point is not an exact 90 degrees.
double currentSpeed = 0;
double pidOutputSpeed;
double input;

//Initialize PID to control motor pulse speed
PID pid = PID(&input, &pidOutputSpeed, &setPoint, kp, ki, kd, DIRECT);

void NMbot::setup(){
    //Begin Serial comms and initialize the motion processor.
    Serial.begin(115200);
    motion.setup();
    
    //Limit the PID to outputs of +/- 500.
    pid.SetOutputLimits(-500,500);
    //Begin PID, options are auto or manual, auto=PID controls output
    //manual = PID effectively off, you must calculate correct speed.
    pid.SetMode(AUTOMATIC);
}

void NMbot::loop(){
    if(motion.tick()){
        Serial.println(motion.angle);
    }
}

void NMbot::CalculateSpeed(){
    //Input to the PID is the current angle of the accelerometer.
    input = motion.angle;
    //Compute the speed we need to move to keep the robot upright (minimize error between motion.angle and setpoint).
    pid.Compute();
    //Set our desired speed to the PID output.
    currentSpeed = pidOutputSpeed;
    
    SetMotorSpeed(0,currentSpeed);
    SetMotorSpeed(1,currentSpeed);
}
void NMbot::SetMotorSpeed(int motor, double mSpeed){  
  //0 is left, 1 is right, motors are mounted in opposite directions, therefore rotate opposite dirs.
  // If we want to move forward, set the motor direction to forwards.  
  if(mSpeed > 0){
    if(motor == 0)digitalWrite(DIR_PIN_L, LOW);
    else digitalWrite(DIR_PIN_R, HIGH);
  }
  // If we want to move backwards, set the motor direction to backwards.  
  else if (mSpeed < 0){
    if(motor == 0) digitalWrite(DIR_PIN_L, HIGH );
    else digitalWrite(DIR_PIN_R, LOW);
  }
    
  // Start software interrupt to pulse motor at mSpeed
  if (motor == 0)
    startTimer(TC1, 0, TC3_IRQn, abs(mSpeed *microstep)); //Microstepping at 1/32
  else
    startTimer(TC1, 1, TC4_IRQn, abs(mSpeed *mircrostep));
}

//Initializes SAM3X8E wave generator at 50% duty cycle, triggers interupt on rising edge of the wave
void NMbot::startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
    pmc_set_writeprotect(false); //allow writing to timer registers
    pmc_enable_periph_clk((uint32_t)irq); // Enable clock for the timer
    TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4); //configure timer in waveform mode
    uint32_t rc = VARIANT_MCK / 128 / frequency; //128 because we selected TIMER_CLOCK4 above
    TC_SetRA(tc, channel, rc / 2); //50% high, 50% low
    TC_SetRC(tc, channel, rc);
    TC_Start(tc, channel);
    tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
    tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
    NVIC_EnableIRQ(irq);
}
