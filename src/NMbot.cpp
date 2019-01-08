#include "NMbot.hpp"
#include "MPU6050.h"
#include "mpu.hpp"

mpu motion;

void NMbot::setup(){
    Serial.begin(115200);
    motion.setup();
}

void NMbot::loop(){
    if(motion.tick()){

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
