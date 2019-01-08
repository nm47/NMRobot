#include "JJ_MPU6050_DMP_6Axis.h"

class mpu
{
    public:
    MPU6050 gyro;
    bool dmpOnline;
    
    void setup();
    bool tick();
};
