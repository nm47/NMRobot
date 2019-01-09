#include "mpu.hpp"

#define OUTPUT_READABLE_QUATERNION
uint8_t fifoBuffer[18];

 void mpu::setup(){
    dmpOnline = false;

    Wire.begin();

    Serial.println("Initializing I2C devices...");
    gyro.initialize();
    gyro.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
    gyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    gyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    gyro.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
    gyro.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
    gyro.setSleepEnabled(false);
    Serial.println(F("Testing device connections..."));
    Serial.println(gyro.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    delay(2000);
    Serial.println("Initializing DMP...");
    int dmpStatus = gyro.dmpInitialize();

    
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
   gyro.resetFIFO();
}

bool mpu::tick(){
  fifocount = gyro.getFIFOCount();
  //packetSize = gyro.dmpGetFIFOPacketSize(); 
  if (fifocount < 18){
    return false;
  }
  /*else if (packetSize < fifocount)
  {
    gyro.resetFIFO();
    return false;
  }*/
  
  gyro.getFIFOBytes(fifoBuffer, 16);
  gyro.dmpGetQuaternion(&q, fifoBuffer);
  gyro.resetFIFO();
  angle = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z) * 57.2957795; //we want yaw, other formulas for reference

  Serial.println(angle);
  // yaw = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
  // pitch = asin(-2.0*(q.x*q.z - q.w*q.y));
  // roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);

  return true;
}
