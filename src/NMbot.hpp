#include <stdint.h>
#include <arduino.h>

class NMbot{
    public:
      void setup();
      void loop();
      void SetMotorSpeed(int motor, double mspeed);
      void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);
      void CalculateSpeed();
};
