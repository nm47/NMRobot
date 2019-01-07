#include "mpu.hpp"
#include <SimpleKalmanFilter.h>

#define OUTPUT_READABLE_ACCELGYRO

//Kalman parameters
float e_mea = 4;  //measurement uncertainty
float e_est = 4;  //estimation uncertainty
float proccessVariance = .01; //how fast the value will change (0.001 - .1)

SimpleKalmanFilter kalman(e_mea,e_est,proccessVariance);

 void mpu::setup(){
    Wire.begin();
    Serial.begin(115200);
    gyro.setSleepEnabled(false);
    gyro.initialize();
    Serial.println(gyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
 
}

bool mpu::tick(){
  int16_t rawValue = gyro.getRotationX();
  double estimated = kalman.updateEstimate(rawValue);
  Serial.print(rawValue);
  Serial.print(",");
  Serial.print(estimated);
  Serial.println(",");
}
