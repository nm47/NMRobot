#include "NMbot.hpp"
#include <MPU6050.h>
#include "mpu.hpp"

mpu motion;

void NMbot::setup(){
    motion.setup();
}

void NMbot::loop(){
    if(motion.tick()){

    }
}
