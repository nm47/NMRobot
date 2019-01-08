#include "mpu.hpp"

#define OUTPUT_READABLE_ACCELGYRO

//Kalman parameters

 void mpu::setup(){
    dmpOnline = false;

    Wire.begin();

    Serial.println("Initializing I2C devices...");
    gyro.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    gyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    gyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    gyro.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
    gyro.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
    gyro.setSleepEnabled(false);
    
    int dmpStatus = gyro.dmpInitialize();
    Serial.println("Initializing DMP...");
    
    if (dmpStatus == 0){
      gyro.setDMPEnabled(true);
      dmpOnline = true;
      Serial.println("DMP Ready");
    }
    else{
      Serial.print(F("DMP initialization failed... (code "));
      Serial.print(dmpStatus);
      Serial.println(F(")"));
    }
    delay(2000);
}

bool mpu::tick(){
  int16_t rawValue = gyro.getRotationX();
  Serial.println(rawValue);
}
