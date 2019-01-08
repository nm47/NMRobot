#include <MPU6050.h>

class mpu
{
    public:
    MPU6050 gyro;
    bool dmpOnline;
    
    void setup();
    bool tick();
};
