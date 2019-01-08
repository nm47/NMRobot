#include <stdint.h>
#include <arduino.h>

class NMbot{
    public:
      int16_t ay;
      int16_t gy;
      int16_t my;
  
      void setup();
      void loop();
      void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);

};
