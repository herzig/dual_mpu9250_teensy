#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>


const uint32_t I2C_FREQ = 50000;

const int32_t mpu_base_accel_bias[] = {-238, 16, 41};
const int32_t mpu_base_gyro_bias[] = {188, 64, -1166};
const float mpu_base_mag_bias[] = { 77.97, 232.41, -424.33 };
const float mpu_base_mag_scale[] = { 0.96, 1.06, 0.99 };
MPU9250 mpu_base(MPU9250_ADDRESS_AD1, Wire, I2C_FREQ);

const int32_t mpu_wrist_accel_bias[] = {-7, 160, 19};
const int32_t mpu_wrist_gyro_bias[] = {-964, 425, -276};
const float mpu_wrist_mag_bias[] = {128.95, -34.49, -131.95};
const float mpu_wrist_mag_scale[] = {1.03, 1.02, 0.96};
MPU9250 mpu_wrist(MPU9250_ADDRESS_AD0, Wire, I2C_FREQ);

MPU9250* imus[] = {&mpu_base, &mpu_wrist};
size_t num_imus = sizeof(imus)/sizeof(imus[0]);

void calibrate_acc_gyro(MPU9250* imu) {

  Serial.print("recording accel & gyro bias");
  int32_t accel_bias[3], gyro_bias[3];
  imu->calibrateMPU9250(accel_bias, gyro_bias);

  Serial.println("accel bias: ");
  Serial.println(accel_bias[0]);
  Serial.println(accel_bias[1]);
  Serial.println(accel_bias[2]);

  Serial.println("gyro bias: ");
  Serial.println(gyro_bias[0]);
  Serial.println(gyro_bias[1]);
  Serial.println(gyro_bias[2]);
}

void calibrate_mag(MPU9250* imu) {

  float mag_bias[3], mag_scale[3];
  imu->magCalMPU9250(mag_bias, mag_scale);
  Serial.println("mag bias: ");
  Serial.println(mag_bias[0]);
  Serial.println(mag_bias[1]);
  Serial.println(mag_bias[2]);

  Serial.println("mag scale: ");
  Serial.println(mag_scale[0]);
  Serial.println(mag_scale[1]);
  Serial.println(mag_scale[2]);
}

void setup() 
{
  Serial.begin(115200);
  while(!Serial.available()) {
    delay(100);
  }

  for (size_t i = 0; i < num_imus; i++) 
  {
    
    MPU9250* imu = imus[i];
    bool check = imu->checkWhoAmI();

    if (!check) 
    {
      Serial.print("Could not connect to MPU9250 at: 0x"); Serial.print(imu->i2cAddr, HEX);
      Serial.println("WHO AM I check failed");
      continue;
    }

    Serial.print("MPU9250 0x"); Serial.print(imu->i2cAddr, HEX); Serial.println(" is online...");

    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    imu->initMPU9250();
    Serial.println("\tinitialized for active data mode....");

    // check mag communication
    check = imu->checkWhoAmIAK8963();
    if (!check) 
    {
      Serial.println("\tAK8963 WHO AM I check failed");
      //continue;
    }
    imu->initAK8963Slave();
    Serial.println("\tAK8963 online as EXT_SENS");
    imu->isEnabled = true;
  }

  if (mpu_base.isEnabled)
  {
    mpu_base.setAccGyroBias(mpu_base_gyro_bias, mpu_base_accel_bias);
    mpu_base.setMagBiasScale(mpu_base_mag_bias, mpu_base_mag_scale);
  }
  if (mpu_wrist.isEnabled)
  {
    mpu_wrist.setAccGyroBias(mpu_wrist_gyro_bias, mpu_wrist_accel_bias);
    mpu_wrist.setMagBiasScale(mpu_wrist_mag_bias, mpu_wrist_mag_scale);
  }

  // calibtration functions, do this to find bias and scale values.
  // WARNING: only do either calibrate_acc_gyro, or calibrate_mag in one go 
  // (calibrate_acc_gyro changes some state and the mag readings will nto work anymore)
  //calibrate_acc_gyro(&mpu_base);
  //mpu_base.update();
  //mpu_base.update();
  //calibrate_mag(&mpu_base);
}

uint32_t ts = 0;
void loop() 
{

  mpu_base.update();
  mpu_wrist.update();

  MPU9250* imu = &mpu_wrist;
  uint32_t now = millis();

  if (now-ts > 100)
  {
    Serial.println(mpu_base.deltat,5);
    Serial.println(mpu_wrist.deltat,5);
    Serial.println();
    
    Serial.print("ypr: "); Serial.print(imu->yaw); Serial.print(" "); Serial.print(imu->pitch); Serial.print(" "); Serial.println(imu->roll);
    Serial.print("acc: "); Serial.print(imu->ax); Serial.print(" "); Serial.print(imu->ay); Serial.print(" "); Serial.println(imu->az);
    Serial.print("gyro:"); Serial.print(imu->gx); Serial.print(" "); Serial.print(imu->gy); Serial.print(" "); Serial.println(imu->gz);
    Serial.print("mag:"); Serial.print(imu->mx); Serial.print(" "); Serial.print(imu->my); Serial.print(" "); Serial.println(imu->mz);
    //Serial.print("temp: "); Serial.println(imu->temperature);
    //Serial.println();
    ts = now;

  }

  //delay(10);
}