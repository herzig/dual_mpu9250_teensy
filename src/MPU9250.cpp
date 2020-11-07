#include "MPU9250.h"

//==============================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer,
//====== and temperature data
//==============================================================================

MPU9250::MPU9250( int8_t csPin, SPIClass &spiInterface, uint32_t spi_freq )
{
	// Use hardware SPI communication
  	// If used with sparkfun breakout board
  	// https://www.sparkfun.com/products/13762 , change the pre-soldered JP2 to
  	// enable SPI (solder middle and left instead of middle and right) pads are
  	// very small and re-soldering can be very tricky. I2C highly recommended.

	_csPin = csPin;
	spi_bus = &spiInterface;
	i2c_bus = NULL;

	_interfaceSpeed = spi_freq;

    spi_bus->begin();
    pinMode(_csPin, OUTPUT);
    deselect();

}
MPU9250::MPU9250( uint8_t address, TwoWire &wirePort, uint32_t clock_frequency )
{
	i2cAddr = address;
	i2c_bus = &wirePort;
	spi_bus = NULL;

	_interfaceSpeed = clock_frequency;

	_csPin = NOT_SPI;	// Used to tell the library that the sensor is using I2C

	i2c_bus->begin();
	i2c_bus->setClock(_interfaceSpeed);

  getAres();
  getGres();
  getMres();
}

bool MPU9250::checkWhoAmI() 
{
  byte c = readByte(i2cAddr, WHO_AM_I_MPU9250);  
  return c == 0x71;
}

bool MPU9250::checkWhoAmIAK8963() 
{
  //  uint8_t c = readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for MPU-9250
  writeByte(i2cAddr, USER_CTRL, 0x20);    // Enable I2C Master mode  
  writeByte(i2cAddr, I2C_MST_CTRL, 0x0D); // I2C configuration multi-master I2C 400KHz

  writeByte(i2cAddr, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
  writeByte(i2cAddr, I2C_SLV0_REG, WHO_AM_I_AK8963);           // I2C slave 0 register address from where to begin data transfer
  writeByte(i2cAddr, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
  delay(10);
  uint8_t c = readByte(i2cAddr, EXT_SENS_DATA_00);             // Read the WHO_AM_I byte

  Serial.print("\t AK8963 WhoAmI is 0x"); Serial.print(c, HEX); Serial.println(", should be 0x48");

  return c == 0x48;
}

void MPU9250::setupMagForSPI()
{
  // Use slave 4 for talking to the magnetometer
  writeByteSPI(49, ((1 << 7) | AK8963_ADDRESS));    // Set the SLV_4_ADDR register to the magnetometer's address
  writeByteSPI(52, 0b00000000);                     // Setup SLV_4 control as needed (but not set to do an operation yet)

  writeByteSPI(36, 0b10000000);   // Enable the multi-master mode
}

void MPU9250::getMres()
{
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.0f * 4912.0f / 8190.0f; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
      break;
  }
}

void MPU9250::getGres()
{
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case GFS_250DPS:
      gRes = 250.0f / 32768.0f;
      break;
    case GFS_500DPS:
      gRes = 500.0f / 32768.0f;
      break;
    case GFS_1000DPS:
      gRes = 1000.0f / 32768.0f;
      break;
    case GFS_2000DPS:
      gRes = 2000.0f / 32768.0f;
      break;
  }
}

void MPU9250::getAres()
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case AFS_2G:
      aRes = 2.0f / 32768.0f;
      break;
    case AFS_4G:
      aRes = 4.0f / 32768.0f;
      break;
    case AFS_8G:
      aRes = 8.0f / 32768.0f;
      break;
    case AFS_16G:
      aRes = 16.0f / 32768.0f;
      break;
  }
}

void MPU9250::readMPU9250Data(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  readBytes(i2cAddr, ACCEL_XOUT_H, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;  
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ; 
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;   
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;  
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;  
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ; 
}

void MPU9250::readAccelData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  // Read the six raw data registers into data array
  readBytes(i2cAddr, ACCEL_XOUT_H, 6, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}


void MPU9250::readGyroData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  // Read the six raw data registers sequentially into data array
  readBytes(i2cAddr, GYRO_XOUT_H, 6, &rawData[0]);

  // Turn the MSB and LSB into a signed 16-bit value
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
}

void MPU9250::readMagData(int16_t * destination)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  //  readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);  // Read the six raw data and ST2 registers sequentially into data array
  //writeByte(i2cAddr, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
  //writeByte(i2cAddr, I2C_SLV0_REG, AK8963_XOUT_L);             // I2C slave 0 register address from where to begin data transfer
  writeByte(i2cAddr, I2C_SLV0_CTRL, 0x87);                     // Enable I2C and read 7 bytes
  readBytes(i2cAddr, EXT_SENS_DATA_00, 7, &rawData[0]);        // Read the x-, y-, and z-axis mag data
  uint8_t c = rawData[6]; // End data read by reading ST2 register
  if(!(c & 0x08)) 
  { // Check if magnetic sensor overflow set, if not then report data
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
  }
}

int16_t MPU9250::readTempData()
{
  uint8_t rawData[2]; // x/y/z gyro register data stored here
  // Read the two raw data registers sequentially into data array
  readBytes(i2cAddr, TEMP_OUT_H, 2, &rawData[0]);
  // Turn the MSB and LSB into a 16-bit value
  return ((int16_t)rawData[0] << 8) | rawData[1];
}

void MPU9250::update() {

  if (!isEnabled)
    return;
    
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (readByte(i2cAddr, INT_STATUS) & 0x01) 
  {
    int16_t data[7];
    //readMPU9250Data(data);

    int16_t accel_data[3];
    readAccelData(accel_data);
    // to m/s
    ax = ((float)accel_data[0])*aRes;
    ay = ((float)accel_data[1])*aRes;
    az = ((float)accel_data[2])*aRes;
    delay(1);

    // to rad/s
    int16_t gyro_data[3];
    readGyroData(gyro_data);
    gx = ((float)gyro_data[0])*gRes*DEG_TO_RAD;
    gy = ((float)gyro_data[1])*gRes*DEG_TO_RAD;  
    gz = ((float)gyro_data[2])*gRes*DEG_TO_RAD; 
    delay(1);

    int16_t mag_data[3] = {0,0,0};
    readMagData(mag_data); 
    
    // to milliGauss
    mx = ((float)mag_data[0]) * mRes * magCalibration[0] - magBias[0];
    my = ((float)mag_data[1]) * mRes * magCalibration[1] - magBias[1];
    mz = ((float)mag_data[2]) * mRes * magCalibration[2] - magBias[2];
    mx *= magScale[0];
    my *= magScale[1];
    mz *= magScale[2];
    //temperature = data[3]/333.87f+21.0f;
  }

  // Must be called before updating quaternions!
  updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. 
  // Pass gyro rate as rad/s
  // MahonyQuaternionUpdate(myIMU->ax, myIMU->ay, myIMU->az, myIMU->gx * DEG_TO_RAD,
  //                        myIMU->gy * DEG_TO_RAD, myIMU->gz * DEG_TO_RAD, myIMU->my,
  //                        myIMU->mx, myIMU->mz, myIMU->deltat);
  fusion_filter.update(ax, ay, az, 
                       gx, gy, gz, 
                       my, mx, -mz, deltat);

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.


float* q = fusion_filter.q;
yaw   = atan2(2.0f * q[1]*q[2] + q[0]*q[3], 
              q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);

pitch = -asin(2.0f *q[1]*q[3] - q[0]*q[2]);

roll  = atan2(2.0f * q[0]*q[1] + q[2]*q[3],
              q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]);

// skip this, we don't really care about absolute north
// Declination of ZSR Glattbrug Office (47.434704, 8.561447) is
// 	Magnetic Field Declination	2.75 °	8.07 °/year on 2020-08-09
// - https://www.geomatrix.co.uk/tools/magnetic-field-calculator/
//myIMU->yaw  -= mag_corr_ZSR;

}

// Calculate the time the last update took for use in the quaternion filters
// TODO: This doesn't really belong in this class.
void MPU9250::updateTime()
{
  now = micros();

  // Set integration time by time elapsed since last filter update
  deltat = ((now - lastUpdate) / 1000000.0f);
  lastUpdate = now;
}

void MPU9250::initAK8963Slave()
{
   // First extract the factory calibration for each magnetometer axis
   uint8_t rawData[3];  // x/y/z gyro calibration data stored here

   writeByte(i2cAddr, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(i2cAddr, I2C_SLV0_REG, AK8963_CNTL2);              // I2C slave 0 register address from where to begin data transfer
   writeByte(i2cAddr, I2C_SLV0_DO, 0x01);                       // Reset AK8963
   writeByte(i2cAddr, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and write 1 byte
   delay(50);
   writeByte(i2cAddr, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(i2cAddr, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
   writeByte(i2cAddr, I2C_SLV0_DO, 0x00);                       // Power down magnetometer  
   writeByte(i2cAddr, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and write 1 byte
   delay(50);
   writeByte(i2cAddr, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(i2cAddr, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
   writeByte(i2cAddr, I2C_SLV0_DO, 0x0F);                       // Enter fuze mode
   writeByte(i2cAddr, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and write 1 byte
   delay(50);
   writeByte(i2cAddr, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
   writeByte(i2cAddr, I2C_SLV0_REG, AK8963_ASAX);               // I2C slave 0 register address from where to begin data transfer
   writeByte(i2cAddr, I2C_SLV0_CTRL, 0x83);                     // Enable I2C and read 3 bytes
   delay(50);
   readBytes(i2cAddr, EXT_SENS_DATA_00, 3, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
   magCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;        // Return x-axis sensitivity adjustment values, etc.
   magCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
   magCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 

   Serial.print("\t AK8963 mag sensitivity adj. values: ");
   Serial.print(magCalibration[0]); Serial.print(" ");
   Serial.print(magCalibration[1]); Serial.print(" ");
   Serial.println(magCalibration[2]); Serial.print(" ");
   
   writeByte(i2cAddr, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(i2cAddr, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer
   writeByte(i2cAddr, I2C_SLV0_DO, 0x00);                       // Power down magnetometer  
   writeByte(i2cAddr, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte
   delay(50);

   writeByte(i2cAddr, I2C_SLV0_ADDR, AK8963_ADDRESS);           // Set the I2C slave address of AK8963 and set for write.
   writeByte(i2cAddr, I2C_SLV0_REG, AK8963_CNTL);               // I2C slave 0 register address from where to begin data transfer 
   // Configure the magnetometer for continuous read and highest resolution
   // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
   // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
   writeByte(i2cAddr, I2C_SLV0_DO, Mscale << 4 | Mmode);        // Set magnetometer data resolution and sample ODR
   writeByte(i2cAddr, I2C_SLV0_CTRL, 0x81);                     // Enable I2C and transfer 1 byte

  delay(50);
  writeByte(i2cAddr, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
  writeByte(i2cAddr, I2C_SLV0_REG, AK8963_XOUT_L);             // I2C slave 0 register address from where to begin data transfer
  delay(50);
}

void MPU9250::initMPU9250()
{

  resetMPU9250();

  // wake up device
  // Clear sleep mode bit (6), enable all sensors
  writeByte(i2cAddr, PWR_MGMT_1, 0x00);
  delay(100); // Wait for all registers to reset

  // Get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  writeByte(i2cAddr, PWR_MGMT_1, 0x01);
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz,
  // respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion
  // update rates cannot be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!),
  // 8 kHz, or 1 kHz
  writeByte(i2cAddr, CONFIG, 0x03);

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  // Use a 200 Hz rate; a rate consistent with the filter update rate
  // determined inset in CONFIG above.
  writeByte(i2cAddr, SMPLRT_DIV, 0x04);

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are
  // left-shifted into positions 4:3

  // get current GYRO_CONFIG register value
  uint8_t c = readByte(i2cAddr, GYRO_CONFIG);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
  // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of
  // GYRO_CONFIG
  // c =| 0x00;
  // Write new GYRO_CONFIG value to register
  writeByte(i2cAddr, GYRO_CONFIG, c );

  // Set accelerometer full-scale range configuration
  // Get current ACCEL_CONFIG register value
  c = readByte(i2cAddr, ACCEL_CONFIG);
  // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  // Write new ACCEL_CONFIG register value
  writeByte(i2cAddr, ACCEL_CONFIG, c);

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by
  // choosing 1 for accel_fchoice_b bit [3]; in this case the bandwidth is
  // 1.13 kHz
  // Get current ACCEL_CONFIG2 register value
  c = readByte(i2cAddr, ACCEL_CONFIG2);
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  // Write new ACCEL_CONFIG2 register value
  writeByte(i2cAddr, ACCEL_CONFIG2, c);
  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because
  // of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH
  // until interrupt cleared, clear on read of INT_STATUS.
  writeByte(i2cAddr, INT_PIN_CFG, 0x10);
  // Enable data ready (bit 0) interrupt
  writeByte(i2cAddr, INT_ENABLE, 0x01);
  delay(100);

  writeByte(i2cAddr, USER_CTRL, 0x20);          // Enable I2C Master mode  
  writeByte(i2cAddr, I2C_MST_CTRL, 0x1D);       // I2C configuration STOP after each transaction, master I2C bus at 400 KHz
  writeByte(i2cAddr, I2C_MST_DELAY_CTRL, 0x81); // Use blocking data retreival and enable delay for mag sample rate mismatch
  writeByte(i2cAddr, I2C_SLV4_CTRL, 0x01);      // Delay mag data retrieval to once every other accel/gyro data sample



  if(_csPin != NOT_SPI)
  {
    setupMagForSPI();
  }
}


// Function which accumulates gyro and accelerometer data after device
// initialization. It calculates the average of the at-rest readings and then
// loads the resulting offsets into accelerometer and gyro bias registers.
void MPU9250::calibrateMPU9250(int32_t* gyro_bias, int32_t* accel_bias)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;

  gyro_bias[0] = 0; gyro_bias[1] = 0; gyro_bias[2] = 0;
  accel_bias[0] = 0; accel_bias[1] = 0; accel_bias[2] = 0;

  // reset device
  // Write a one to bit 7 reset bit; toggle reset device
  writeByte(i2cAddr, PWR_MGMT_1, 0x80);
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope
  // reference if ready else use the internal oscillator, bits 2:0 = 001
  writeByte(i2cAddr, PWR_MGMT_1, 0x01);
  writeByte(i2cAddr, PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  // Disable all interrupts
  writeByte(i2cAddr, INT_ENABLE, 0x00);
  // Disable FIFO
  writeByte(i2cAddr, FIFO_EN, 0x00);
  // Turn on internal clock source
  writeByte(i2cAddr, PWR_MGMT_1, 0x00);
  // Disable I2C master
  writeByte(i2cAddr, I2C_MST_CTRL, 0x00);
  // Disable FIFO and I2C master modes
  writeByte(i2cAddr, USER_CTRL, 0x00);
  // Reset FIFO and DMP
  writeByte(i2cAddr, USER_CTRL, 0x0C);
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  // Set low-pass filter to 188 Hz
  writeByte(i2cAddr, CONFIG, 0x01);
  // Set sample rate to 1 kHz
  writeByte(i2cAddr, SMPLRT_DIV, 0x00);
  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  writeByte(i2cAddr, GYRO_CONFIG, 0x00);
  // Set accelerometer full-scale to 2 g, maximum sensitivity
  writeByte(i2cAddr, ACCEL_CONFIG, 0x00);

  //uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384; // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  writeByte(i2cAddr, USER_CTRL, 0x40);  // Enable FIFO
  // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
  // MPU-9150)
  writeByte(i2cAddr, FIFO_EN, 0x78);
  delay(40);  // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  // Disable gyro and accelerometer sensors for FIFO
  writeByte(i2cAddr, FIFO_EN, 0x00);
  // Read FIFO sample count
  readBytes(i2cAddr, FIFO_COUNTH, 2, &data[0]);
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  // How many sets of full gyro and accelerometer data for averaging
  packet_count = fifo_count/12;

  for (ii = 0; ii < packet_count; ii++)
  {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    // Read data for averaging
    readBytes(i2cAddr, FIFO_R_W, 12, &data[0]);
    // Form signed 16-bit integer for each sample in FIFO
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  );
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  );
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  );
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  );
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  );
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]);

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit
    // biases.
    accel_bias[0] += (int32_t) accel_temp[0];
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
  }
  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  accel_bias[0] /= (int32_t) packet_count;
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
  if (accel_bias[2] > 0L)
  {
    accel_bias[2] -= (int32_t) accelsensitivity;
  }
  else
  {
    accel_bias[2] += (int32_t) accelsensitivity;
  }

  setAccGyroBias(gyro_bias, accel_bias);
}


void MPU9250::setAccGyroBias(const int32_t* gyro_bias, const int32_t* accel_bias) 
{
  uint8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data

  // Construct the gyro biases for push to the hardware gyro bias registers,
  // which are reset to zero upon device startup.
  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
  // format.
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF;
  // Biases are additive, so change sign on calculated average gyro biases
  data[1] = (-gyro_bias[0]/4)       & 0xFF;
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

  // Push gyro biases to hardware registers
  writeByte(i2cAddr, XG_OFFSET_H, data[0]);
  writeByte(i2cAddr, XG_OFFSET_L, data[1]);
  writeByte(i2cAddr, YG_OFFSET_H, data[2]);
  writeByte(i2cAddr, YG_OFFSET_L, data[3]);
  writeByte(i2cAddr, ZG_OFFSET_H, data[4]);
  writeByte(i2cAddr, ZG_OFFSET_L, data[5]);

  // Construct the accelerometer biases for push to the hardware accelerometer
  // bias registers. These registers contain factory trim values which must be
  // added to the calculated accelerometer biases; on boot up these registers
  // will hold non-zero values. In addition, bit 0 of the lower byte must be
  // preserved since it is used for temperature compensation calculations.
  // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  // A place to hold the factory accelerometer trim biases
  int32_t accel_bias_reg[3] = {0, 0, 0};
  // Read factory accelerometer trim values
  readBytes(i2cAddr, XA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(i2cAddr, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  readBytes(i2cAddr, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  // Define mask for temperature compensation bit 0 of lower byte of
  // accelerometer bias registers
  uint32_t mask = 1uL;
  // Define array to hold mask bit for each accelerometer bias axis
  uint8_t mask_bit[3] = {0, 0, 0};

  for (size_t ii = 0; ii < 3; ii++)
  {
    // If temperature compensation bit is set, record that fact in mask_bit
    if ((accel_bias_reg[ii] & mask))
    {
      mask_bit[ii] = 0x01;
    }
  }

  // Construct total accelerometer bias, including calculated average
  // accelerometer bias from above
  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
  // (16 g full scale)
  accel_bias_reg[0] -= (accel_bias[0]/8);
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  // preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[1] = data[1] | mask_bit[0];
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  // Preserve temperature compensation bit when writing back to accelerometer
  // bias registers
  data[5] = data[5] | mask_bit[2];

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  writeByte(i2cAddr, XA_OFFSET_H, data[0]);
  writeByte(i2cAddr, XA_OFFSET_L, data[1]);
  writeByte(i2cAddr, YA_OFFSET_H, data[2]);
  writeByte(i2cAddr, YA_OFFSET_L, data[3]);
  writeByte(i2cAddr, ZA_OFFSET_H, data[4]);
  writeByte(i2cAddr, ZA_OFFSET_L, data[5]);
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or less
// deviation is a pass.
void MPU9250::MPU9250SelfTest(float * destination)
{
  uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = GFS_250DPS;
   
  writeByte(i2cAddr, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
  writeByte(i2cAddr, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  writeByte(i2cAddr, GYRO_CONFIG, FS<<3);  // Set full scale range for the gyro to 250 dps
  writeByte(i2cAddr, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  writeByte(i2cAddr, ACCEL_CONFIG, FS<<3); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
    readBytes(i2cAddr, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    readBytes(i2cAddr, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }

  // Get average of 200 values and store as average current readings
  for (int ii =0; ii < 3; ii++)
  {
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

  // Configure the accelerometer for self-test
  // Enable self test on all three axes and set accelerometer range to +/- 2 g
  writeByte(i2cAddr, ACCEL_CONFIG, 0xE0);
  // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  writeByte(i2cAddr, GYRO_CONFIG,  0xE0);
  delay(25);  // Delay a while to let the device stabilize

  // Get average self-test values of gyro and acclerometer
  for (int ii = 0; ii < 200; ii++)
  {
    // Read the six raw data registers into data array
    readBytes(i2cAddr, ACCEL_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    // Read the six raw data registers sequentially into data array
    readBytes(i2cAddr, GYRO_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  // Get average of 200 values and store as average self-test readings
  for (int ii =0; ii < 3; ii++)
  {
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  writeByte(i2cAddr, ACCEL_CONFIG, 0x00);
  writeByte(i2cAddr, GYRO_CONFIG,  0x00);
  delay(25);  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  // X-axis accel self-test results
  selfTest[0] = readByte(i2cAddr, SELF_TEST_X_ACCEL);
  // Y-axis accel self-test results
  selfTest[1] = readByte(i2cAddr, SELF_TEST_Y_ACCEL);
  // Z-axis accel self-test results
  selfTest[2] = readByte(i2cAddr, SELF_TEST_Z_ACCEL);
  // X-axis gyro self-test results
  selfTest[3] = readByte(i2cAddr, SELF_TEST_X_GYRO);
  // Y-axis gyro self-test results
  selfTest[4] = readByte(i2cAddr, SELF_TEST_Y_GYRO);
  // Z-axis gyro self-test results
  selfTest[5] = readByte(i2cAddr, SELF_TEST_Z_GYRO);

  // Retrieve factory self-test value from self-test code reads
  // FT[Xa] factory trim calculation
  factoryTrim[0] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[0] - 1.0) ));
  // FT[Ya] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[1] - 1.0) ));
  // FT[Za] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[2] - 1.0) ));
  // FT[Xg] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[3] - 1.0) ));
  // FT[Yg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[4] - 1.0) ));
  // FT[Zg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow(1.01 ,((float)selfTest[5] - 1.0) ));

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
  // of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++)
  {
    // Report percent differences
    destination[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]
      - 100.;
    // Report percent differences
    destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]
      - 100.;
  }
}

// Function which accumulates magnetometer data after device initialization.
// It calculates the bias and scale in the x, y, and z axes.
void MPU9250::magCalMPU9250(float * bias, float * scale) 
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
  uint8_t rawData[7] = {0, 0, 0, 0, 0, 0, 0}, magCalibration[3] = {0, 0, 0};

  Serial.print("Mag Calibration: Wave device in a figure eight until done!");
  Serial.println(Mmode, HEX);
  delay(2000);
  
// shoot for ~fifteen seconds of mag data
  if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
  if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
  for(ii = 0; ii < sample_count; ii++) {

    readMagData(mag_temp);
    
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }

    if (Mmode == 0x02) delay(125);
    if (Mmode == 0x06) delay(10);
  }

  Serial.println("mag x max/min:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  Serial.println("mag y max/min:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  Serial.println("mag z max/min:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

  writeByte(i2cAddr, I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);    // Set the I2C slave address of AK8963 and set for read.
  writeByte(i2cAddr, I2C_SLV0_REG, AK8963_ASAX);               // I2C slave 0 register address from where to begin data transfer
  writeByte(i2cAddr, I2C_SLV0_CTRL, 0x83);                     // Enable I2C and read 3 bytes
  delay(50);
  readBytes(i2cAddr, EXT_SENS_DATA_00, 3, &rawData[0]);        // Read the x-, y-, and z-axis calibration values
  magCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;        // Return x-axis sensitivity adjustment values, etc.
  magCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
  magCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 

  bias[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
  bias[1] = (float) mag_bias[1]*mRes*magCalibration[1];   
  bias[2] = (float) mag_bias[2]*mRes*magCalibration[2];  
      
  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  scale[0] = avg_rad/((float)mag_scale[0]);
  scale[1] = avg_rad/((float)mag_scale[1]);
  scale[2] = avg_rad/((float)mag_scale[2]);

  Serial.println("Mag Calibration done!");
}

void MPU9250::setMagBiasScale(const float bias[3], const float scale[3]) {
  
  for (size_t i = 0; i < 3; i++)
  {
    this->magBias[i] = bias[i];
    this->magScale[i] = scale[i];
  }
}

void MPU9250::resetMPU9250() {
  writeByte(i2cAddr, PWR_MGMT_1, 0x80);
}

// Wire.h read and write protocols
uint8_t MPU9250::writeByte(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t data)
{
  if (_csPin != NOT_SPI)
  {
    return writeByteSPI(registerAddress, data);
  }
  else
  {
    return writeByteWire(deviceAddress,registerAddress, data);
  }
}

uint8_t MPU9250::writeByteSPI(uint8_t registerAddress, uint8_t writeData)
{
  uint8_t returnVal;

  spi_bus->beginTransaction(SPISettings(_interfaceSpeed, MSBFIRST, SPI_MODE));
  select();

  spi_bus->transfer(registerAddress);
  returnVal = spi_bus->transfer(writeData);

  deselect();
  spi_bus->endTransaction();
// #ifdef SERIAL_DEBUG
//   Serial.print("MPU9250::writeByteSPI slave returned: 0x");
//   Serial.println(returnVal, HEX);
// #endif
  return returnVal;
}

uint8_t MPU9250::writeByteWire(uint8_t deviceAddress, uint8_t registerAddress,
                            uint8_t data)
{
	i2c_bus->setClock(_interfaceSpeed);			// Reset to the desired speed, in case other devices required a slowdown
  	i2c_bus->beginTransmission(deviceAddress);  	// Initialize the Tx buffer
  	i2c_bus->write(registerAddress);      		// Put slave register address in Tx buffer
  	i2c_bus->write(data);                 		// Put data in Tx buffer
  	i2c_bus->endTransmission();           		// Send the Tx buffer
  	// TODO: Fix this to return something meaningful
  	// return NULL; // In the meantime fix it to return the right type
  	return 0;
}

// Read a byte from given register on device. Calls necessary SPI or I2C
// implementation. This was configured in the constructor.
uint8_t MPU9250::readByte(uint8_t deviceAddress, uint8_t registerAddress)
{
  if (_csPin != NOT_SPI)
  {
    if(deviceAddress == AK8963_ADDRESS)
    {
      return readMagByteSPI(registerAddress);
    }
    else
    {
      return readByteSPI(registerAddress);
    } 
  }
  else
  {
    return readByteWire(deviceAddress, registerAddress);
  }
}

uint8_t MPU9250::readMagByteSPI(uint8_t registerAddress)
{
  setupMagForSPI();

  writeByteSPI(49, ((1 << 7) | AK8963_ADDRESS));
  writeByteSPI(50, registerAddress);
  writeByteSPI(52, 0b11000000);         // Command the read into I2C_SLV4_DI register, cause an interrupt when complete

  // Wait for the data to be ready
  uint8_t I2C_MASTER_STATUS = readByteSPI(54);

  uint32_t count = 0;
  while(((I2C_MASTER_STATUS & 0b01000000) == 0) && (count++ < 100000))            // Checks against the I2C_SLV4_DONE bit in the I2C master status register
  {
    I2C_MASTER_STATUS = readByteSPI(54);  
  }
  if(count > 10000)
  {
    Serial.println(F("Timed out"));
  }
  
  


  return readByteSPI(53);   // Read the data that is in the SLV4_DI register 
}

uint8_t MPU9250::writeMagByteSPI(uint8_t registerAddress, uint8_t data)
{
  setupMagForSPI();

  writeByteSPI(49, ((1 << 7) | AK8963_ADDRESS));
  writeByteSPI(50, registerAddress);
  writeByteSPI(51, data);
  writeByteSPI(52, 0b11000000);         // Command the read into I2C_SLV4_DI register, cause an interrupt when complete

  uint8_t I2C_MASTER_STATUS = readByteSPI(54);
  uint32_t count = 0;
  while(((I2C_MASTER_STATUS & 0b01000000) == 0) && (count++ < 10000))            // Checks against the I2C_SLV4_DONE bit in the I2C master status register
  {
    I2C_MASTER_STATUS = readByteSPI(54);  
  }
  if(count > 10000)
  {
    Serial.println(F("Timed out"));
  }
  return 0x00;
}

// Read a byte from the given register address from device using I2C
uint8_t MPU9250::readByteWire(uint8_t deviceAddress, uint8_t registerAddress)
{
  uint8_t data; // `data` will store the register data

  // Initialize the Tx buffer
  i2c_bus->beginTransmission(deviceAddress);
  // Put slave register address in Tx buffer
  i2c_bus->write(registerAddress);
  // Send the Tx buffer, but send a restart to keep connection alive
  i2c_bus->endTransmission(false);
  // Read one byte from slave register address
  i2c_bus->requestFrom(deviceAddress, (uint8_t) 1);
  // Fill Rx buffer with result
  data = i2c_bus->read();
  // Return data read from slave register
  return data;
}

// Read a byte from the given register address using SPI
uint8_t MPU9250::readByteSPI(uint8_t registerAddress)
{
  return writeByteSPI(registerAddress | READ_FLAG, 0xFF /*0xFF is arbitrary*/);
}

// Read 1 or more bytes from given register and device using I2C
uint8_t MPU9250::readBytesWire(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t count, uint8_t * dest)
{
  // Initialize the Tx buffer
  i2c_bus->beginTransmission(deviceAddress);
  // Put slave register address in Tx buffer
  i2c_bus->write(registerAddress);
  // Send the Tx buffer, but send a restart to keep connection alive
  i2c_bus->endTransmission(false);

  uint8_t i = 0;
  // Read bytes from slave register address
  uint8_t resp = i2c_bus->requestFrom(deviceAddress, count);
  if (resp != count) {
    Serial.print("I2C ERR requested didn't return correct number of bytes. addr: 0x"); Serial.print(deviceAddress,HEX);
    Serial.print(" requested: "); Serial.print(count); Serial.print(" recv: "); Serial.print(resp);
    Serial.println();
    
  }
  while (i2c_bus->available() && i <= count)
  {
    // Put read results in the Rx buffer
    dest[i++] = i2c_bus->read();
  }

  return i; // Return number of bytes written
}

// Select slave IC by asserting CS pin
void MPU9250::select()
{
  digitalWrite(_csPin, LOW);
}

// Select slave IC by deasserting CS pin
void MPU9250::deselect()
{
  digitalWrite(_csPin, HIGH);
}

uint8_t MPU9250::readBytesSPI(uint8_t registerAddress, uint8_t count,
                           uint8_t * dest)
{
  spi_bus->beginTransaction(SPISettings(SPI_DATA_RATE, MSBFIRST, SPI_MODE));
  select();

  spi_bus->transfer(registerAddress | READ_FLAG);

  uint8_t i;

  for (i = 0; i < count; i++)
  {
    dest[i] = spi_bus->transfer(0x00);
// #ifdef SERIAL_DEBUG
//     Serial.print("readBytesSPI::Read byte: 0x");
//     Serial.println(dest[i], HEX);
// #endif
  }

  spi_bus->endTransaction();
  deselect();

  delayMicroseconds(50);

  return i; // Return number of bytes written

  /*
#ifdef SERIAL_DEBUG
  Serial.print("MPU9250::writeByteSPI slave returned: 0x");
  Serial.println(returnVal, HEX);
#endif
  return returnVal;
  */

  /*
  // Set slave address of AK8963 and set AK8963 for read
  writeByteSPI(I2C_SLV0_ADDR, AK8963_ADDRESS | READ_FLAG);

Serial.print("\nBHW::I2C_SLV0_ADDR set to: 0x");
Serial.println(readByte(_I2Caddr, I2C_SLV0_ADDR), HEX);

  // Set address to start read from
  writeByteSPI(I2C_SLV0_REG, registerAddress);
  // Read bytes from magnetometer
  //
Serial.print("\nBHW::I2C_SLV0_CTRL gets 0x");
Serial.println(READ_FLAG | count, HEX);

  // Read count bytes from registerAddress via I2C_SLV0
  Serial.print("BHW::readBytesSPI: return value test: ");
  Serial.println(writeByteSPI(I2C_SLV0_CTRL, READ_FLAG | count));
  */
}

uint8_t MPU9250::readBytes(uint8_t deviceAddress, uint8_t registerAddress,
                        uint8_t count, uint8_t * dest)
{
  if (_csPin == NOT_SPI)  // Read via I2C
  {
    return readBytesWire(deviceAddress, registerAddress, count, dest);
  }
  else  // Read using SPI
  {
    return readBytesSPI(registerAddress, count, dest);
  }
}

bool MPU9250::magInit()
{
  // Reset registers to defaults, bit auto clears
  writeByteSPI(0x6B, 0x80);
  // Auto select the best available clock source
  writeByteSPI(0x6B, 0x01);
  // Enable X,Y, & Z axes of accel and gyro
  writeByteSPI(0x6C, 0x00);
  // Config disable FSYNC pin, set gyro/temp bandwidth to 184/188 Hz
  writeByteSPI(0x1A, 0x01);
  // Self tests off, gyro set to +/-2000 dps FS
  writeByteSPI(0x1B, 0x18);
  // Self test off, accel set to +/- 8g FS
  writeByteSPI(0x1C, 0x08);
  // Bypass DLPF and set accel bandwidth to 184 Hz
  writeByteSPI(0x1D, 0x09);
  // Configure INT pin (active high / push-pull / latch until read)
  writeByteSPI(0x37, 0x30);
  // Enable I2C master mode
  // TODO Why not do this 11-100 ms after power up?
  writeByteSPI(0x6A, 0x20);
  // Disable multi-master and set I2C master clock to 400 kHz
  //https://developer.mbed.org/users/kylongmu/code/MPU9250_SPI/ calls says
  // enabled multi-master... TODO Find out why
  writeByteSPI(0x24, 0x0D);
  // Set to write to slave address 0x0C
  writeByteSPI(0x25, 0x0C);
  // Point save 0 register at AK8963's control 2 (soft reset) register
  writeByteSPI(0x26, 0x0B);
  // Send 0x01 to AK8963 via slave 0 to trigger a soft restart
  writeByteSPI(0x63, 0x01);
  // Enable simple 1-byte I2C reads from slave 0
  writeByteSPI(0x27, 0x81);
  // Point save 0 register at AK8963's control 1 (mode) register
  writeByteSPI(0x26, 0x0A);
  // 16-bit continuous measurement mode 1
  writeByteSPI(0x63, 0x12);
  // Enable simple 1-byte I2C reads from slave 0
  writeByteSPI(0x27, 0x81);

  // TODO: Remove this code
  uint8_t ret = ak8963WhoAmI_SPI();
#ifdef SERIAL_DEBUG
  Serial.print("MPU9250::magInit to return ");
  Serial.println((ret == 0x48) ? "true" : "false");
#endif
  return ret == 0x48;
}

// Write a null byte w/o CS assertion to get SPI hardware to idle high (mode 3)
void MPU9250::kickHardware()
{
  spi_bus->beginTransaction(SPISettings(SPI_DATA_RATE, MSBFIRST, SPI_MODE));
  spi_bus->transfer(0x00); // Send null byte
  spi_bus->endTransaction();
}

bool MPU9250::begin()
{
  kickHardware();
  return magInit();
}

// Read the WHOAMI (WIA) register of the AK8963
// TODO: This method has side effects
uint8_t MPU9250::ak8963WhoAmI_SPI()
{
  uint8_t response, oldSlaveAddress, oldSlaveRegister, oldSlaveConfig;
  // Save state
  oldSlaveAddress  = readByteSPI(I2C_SLV0_ADDR);
  oldSlaveRegister = readByteSPI(I2C_SLV0_REG);
  oldSlaveConfig   = readByteSPI(I2C_SLV0_CTRL);
#ifdef SERIAL_DEBUG
  Serial.print("Old slave address: 0x");
  Serial.println(oldSlaveAddress, HEX);
  Serial.print("Old slave register: 0x");
  Serial.println(oldSlaveRegister, HEX);
  Serial.print("Old slave config: 0x");
  Serial.println(oldSlaveConfig, HEX);
#endif

  // Set the I2C slave addres of AK8963 and set for read
  response = writeByteSPI(I2C_SLV0_ADDR, AK8963_ADDRESS|READ_FLAG);
  // I2C slave 0 register address from where to begin data transfer
  response = writeByteSPI(I2C_SLV0_REG, 0x00);
  // Enable 1-byte reads on slave 0
  response = writeByteSPI(I2C_SLV0_CTRL, 0x81);
  delayMicroseconds(1);
  // Read WIA register
  response = writeByteSPI(WHO_AM_I_AK8963|READ_FLAG, 0x00);

  // Restore state
  writeByteSPI(I2C_SLV0_ADDR, oldSlaveAddress);
  writeByteSPI(I2C_SLV0_REG, oldSlaveRegister);
  writeByteSPI(I2C_SLV0_CTRL, oldSlaveConfig);

  return response;
}
