#include "mpu.hpp"

#define OUTPUT_READABLE_ACCELGYRO

 void mpu::setup(){
    Wire.begin();
    Serial.begin(115200);
    gyro.setSleepEnabled(false);
    gyro.initialize();
    Serial.println(gyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
 
}

bool mpu::tick(){
  Serial.println(gyro.getRotationX());
}
