#include "mpu.hpp"

#define OUTPUT_READABLE_QUATERNION
uint8_t fifoBuffer[18];

double mpu::angle = 0.;
bool mpu::dmpOnline = false;

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
    //gyro.setSleepEnabled(false);
    
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
   delay(1000);
   gyro.resetFIFO();
}

bool mpu::tick(){
  fifocount = gyro.getFIFOCount();
  if (fifocount >= 18){
    if (fifocount > 18){
      gyro.resetFIFO();
      return false;
    }
    gyro.getFIFOBytes(fifoBuffer, 16);
    gyro.dmpGetQuaternion(&q, fifoBuffer);
  
    
   //angle = atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    angle =  asin(-2.0*(q.x*q.z - q.w*q.y));
   //roll = atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
  // Serial.print(angle);
    gyro.resetFIFO();
    return true;
  }
  return false;
}
