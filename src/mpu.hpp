#include "JJ_MPU6050_DMP_6Axis.h"

class mpu
{
    public:
    MPU6050 gyro;
    bool dmpOnline;
    Quaternion q;
    uint16_t fifocount;
    uint8_t packetSize;
    float angle;
    void setup();
    bool tick();
};
