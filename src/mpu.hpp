#include <MPU6050.h>

class mpu
{
    public:
    MPU6050 gyro;

    void setup();
    bool tick();
};
