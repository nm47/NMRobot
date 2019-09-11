#ifndef MPU_HPP
#define MPU_HPP

#include "JJ_MPU6050_DMP_6Axis.h"

class mpu
{
    public:
      static bool dmpOnline;
      static double angle;
      void setup();
      bool tick();
    
    private:
      Quaternion q;
      MPU6050 gyro;
      uint16_t fifocount;
      uint8_t packetSize;
      
};

#endif
