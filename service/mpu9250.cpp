#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "I2Cdev/I2Cdev.h"
#include "mpu9250.h"

#define PI 3.1415926

#define i2c_write   writeBytes
#define i2c_read(a,b,c,d)    (readBytes(a,b,c,d)!=-1?0:1)
#define delay_ms(a)    usleep(a*1000)


MPU9250::MPU9250()
    : mAccScale(AFS_16G), mGyroScale(GFS_2000DPS), mMagScale(MFS_16BITS), mMode(0x06)
{

    buildMagRes();
    buildAccRes();
    buildGyroRes();

}

void MPU9250::buildMagRes(){
  switch (mMagScale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mMagRes = 10.0*4912.0/8190.0; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mMagRes = 48.0f/32767.0f;//10.0*4912.0/32760.0; // Proper scale to return milliGauss
          break;
  }
}
void MPU9250::buildAccRes(){
  switch (mAccScale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          mAccRes = 2.0/32768.0;
          break;
    case AFS_4G:
          mAccRes = 4.0/32768.0;
          break;
    case AFS_8G:
          mAccRes = 8.0/32768.0;
          break;
    case AFS_16G:
          mAccRes = 16.0/32768.0;
          break;
  }
  mAccRes *= 9.80665f;
}
void MPU9250::buildGyroRes(){
  switch (mGyroScale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          mGyroRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          mGyroRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          mGyroRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          mGyroRes = 2000.0/32768.0;
          break;
  }
}

status_t MPU9250::readAccelData(int16_t * destination, nsecs_t *timestamp)
{
  uint8_t rawData[6];  // x/y/z accel register data stored here
  int status = i2c_read(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawData);
  *timestamp = systemTime();
  if(status){// Read the six raw data registers into data array
    return FAIL;
  }  
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

  return OK;
}

status_t MPU9250::readGyroData(int16_t * destination, nsecs_t *timestamp)
{
  uint8_t rawData[6];  // x/y/z gyro register data stored here
  int status = i2c_read(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
  *timestamp = systemTime();
  if(status){// Read the six raw data registers sequentially into data array
    return FAIL;
  }  
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 

  return OK;
}

status_t MPU9250::readMagData(int16_t * destination, nsecs_t *timestamp)
{
  uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
  uint8_t meta;
  int status = 0;
  if(i2c_read(AK8963_ADDRESS, AK8963_ST1, 1, &meta)){
  	return FAIL;
  }
  if(meta & 0x01){ // wait for magnetometer data ready bit to be set
  	status = i2c_read(AK8963_ADDRESS, AK8963_XOUT_L, 7, &rawData[0]);
  	*timestamp = systemTime();
  	if(status){// Read the six raw data and ST2 registers sequentially into data array
  		return FAIL;
  	}
  	uint8_t c = rawData[6];// End data read by reading ST2 register

  	if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
      destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
      destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
      destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ; 
      return OK;
    }
    return FAIL;
  }

  return FAIL;
  
}

status_t MPU9250::readTempData(int16_t * destination, nsecs_t *timestamp)
{
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  int status = i2c_read(MPU9250_ADDRESS, TEMP_OUT_H, 2, rawData);
  *timestamp = systemTime();
  if(status )  // Read the two raw data registers sequentially into data array 
  {
  	return FAIL;
  }
  *destination = (int16_t)(((int16_t)rawData[0]) << 8 | rawData[1]) ;  // Turn the MSB and LSB into a 16-bit value
  return OK;
}

void MPU9250::resetMPU9250() {
  // reset device
  uint8_t meta = 0x80;
  // Write a one to bit 7 reset bit; toggle reset device
  i2c_write(MPU9250_ADDRESS,PWR_MGMT_1, 1 , &meta);
  delay_ms(1);
}

void MPU9250::initAK8963()
{
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here
  uint8_t meta;
  meta = 0x00;
  i2c_write(AK8963_ADDRESS, AK8963_CNTL, 1, &meta); // Power down magnetometer  
  delay_ms(1);
  meta = 0x0F;
  i2c_write(AK8963_ADDRESS, AK8963_CNTL, 1, &meta); // Enter Fuse ROM access mode
  delay_ms(1);
  i2c_read(AK8963_ADDRESS, AK8963_ASAX, 3, rawData);  // Read the x-, y-, and z-axis calibration values
  mMagCalibration[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  mMagCalibration[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f;  
  mMagCalibration[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f; 
  meta = 0x00;
  i2c_write(AK8963_ADDRESS, AK8963_CNTL,1,  &meta); // Power down magnetometer  
  delay_ms(1);
  // Configure the magnetometer for continuous read and highest resolution
  // set mMagScale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition mMode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  meta = mMagScale << 4 | mMode;
  i2c_write(AK8963_ADDRESS, AK8963_CNTL, 1, &meta); // Set magnetometer data resolution and sample ODR
  delay_ms(1);
}

void MPU9250::initMPU9250()
{  
 // Initialize MPU9250 device
 // wake up device
  uint8_t meta = 0x00;
  i2c_write(MPU9250_ADDRESS, PWR_MGMT_1, 1, &meta); // Clear sleep mode bit (6), enable all sensors 
  delay_ms(1); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

 // get stable time source
  meta = 0x01;
  i2c_write(MPU9250_ADDRESS, PWR_MGMT_1, 1, &meta);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  delay_ms(1);
 // Configure Gyro and Thermometer
 // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
 // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
 // be higher than 1 / 0.0059 = 170 Hz
 // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
 // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  meta = 0x06;//changed, to 5Hz
  i2c_write(MPU9250_ADDRESS, CONFIG, 1, &meta);
  delay_ms(1);
 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  meta = 0x04;
  i2c_write(MPU9250_ADDRESS, SMPLRT_DIV, 1, &meta);  // Use a 200 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 delay_ms(1);
// Set gyroscope full scale range
 // Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = 0;
  i2c_read(MPU9250_ADDRESS, GYRO_CONFIG, 1, &c); // get current GYRO_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x03; // Clear Fchoice bits [1:0] 
  c = c & ~0x18; // Clear GFS bits [4:3]
  c = c | mGyroScale << 3; // Set full scale range for the gyro
 // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
  i2c_write(MPU9250_ADDRESS, GYRO_CONFIG, 1, &c); // Write new GYRO_CONFIG value to register
  delay_ms(1);
 // Set accelerometer full-scale range configuration
  c = 0;
  i2c_read(MPU9250_ADDRESS, ACCEL_CONFIG, 1, &c); // get current ACCEL_CONFIG register value
 // c = c & ~0xE0; // Clear self-test bits [7:5] 
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | mAccScale << 3; // Set full scale range for the accelerometer 
  i2c_write(MPU9250_ADDRESS, ACCEL_CONFIG, 1, &c); // Write new ACCEL_CONFIG register value
 delay_ms(1);
 // Set accelerometer sample rate configuration
 // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
 // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = 0 ;
  i2c_read(MPU9250_ADDRESS, ACCEL_CONFIG2, 1, &c); // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  i2c_write(MPU9250_ADDRESS, ACCEL_CONFIG2, 1, &c); // Write new ACCEL_CONFIG2 register value
delay_ms(1);
 // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
 // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
  meta = 0x22;
   i2c_write(MPU9250_ADDRESS, INT_PIN_CFG, 1, &meta);   
   delay_ms(1);
   meta = 0x01; 
   i2c_write(MPU9250_ADDRESS, INT_ENABLE, 1, &meta);   // Enable data ready (bit 0) interrupt
   delay_ms(1);
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU9250::accelgyrocal()
{  
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  
 // reset device
  uint8_t meta = 0x80;
  i2c_write(MPU9250_ADDRESS, PWR_MGMT_1, 1, &meta); // Write a one to bit 7 reset bit; toggle reset device
  delay_ms(1);
   
 // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
 // else use the internal oscillator, bits 2:0 = 001
  meta = 0x01;
  i2c_write(MPU9250_ADDRESS, PWR_MGMT_1, 1, &meta); 
  meta = 0x00;
  i2c_write(MPU9250_ADDRESS, PWR_MGMT_2, 1, &meta);
  delay_ms(1);                                    

// Configure device for bias calculation
  meta = 0x00;
  i2c_write(MPU9250_ADDRESS, INT_ENABLE, 1, &meta);   // Disable all interrupts
  i2c_write(MPU9250_ADDRESS, FIFO_EN, 1, &meta);     // Disable FIFO
  i2c_write(MPU9250_ADDRESS, PWR_MGMT_1, 1, &meta);  // Turn on internal clock source
  i2c_write(MPU9250_ADDRESS, I2C_MST_CTRL, 1, &meta); // Disable I2C master
  i2c_write(MPU9250_ADDRESS, USER_CTRL, 1, &meta);    // Disable FIFO and I2C master modes
  meta = 0x0C;
  i2c_write(MPU9250_ADDRESS, USER_CTRL, 1, &meta);    // Reset FIFO and DMP
  delay_ms(1);
  
// Configure MPU6050 gyro and accelerometer for bias calculation
  meta = 0x01;
  i2c_write(MPU9250_ADDRESS, CONFIG, 1, &meta);      // Set low-pass filter to 188 Hz
  meta = 0x00;
  i2c_write(MPU9250_ADDRESS, SMPLRT_DIV, 1, &meta);  // Set sample rate to 1 kHz
  i2c_write(MPU9250_ADDRESS, GYRO_CONFIG, 1, &meta);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  i2c_write(MPU9250_ADDRESS, ACCEL_CONFIG, 1, &meta); // Set accelerometer full-scale to 2 g, maximum sensitivity
 
  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
  meta = 0x40;
  i2c_write(MPU9250_ADDRESS, USER_CTRL, 1, &meta);   // Enable FIFO  
  meta = 0x78;
  i2c_write(MPU9250_ADDRESS, FIFO_EN, 1, &meta);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay_ms(1); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  meta = 0x00;
  i2c_write(MPU9250_ADDRESS, FIFO_EN, 1, &meta);        // Disable gyro and accelerometer sensors for FIFO
  i2c_read(MPU9250_ADDRESS, FIFO_COUNTH, 2, data); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
  
  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    i2c_read(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
            
  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;
    
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}
   
// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
// Push gyro biases to hardware registers
  i2c_write(MPU9250_ADDRESS, XG_OFFSET_H, 1, data);
  i2c_write(MPU9250_ADDRESS, XG_OFFSET_L, 1, data+1);
  i2c_write(MPU9250_ADDRESS, YG_OFFSET_H, 1, data+2);
  i2c_write(MPU9250_ADDRESS, YG_OFFSET_L, 1, data+3);
  i2c_write(MPU9250_ADDRESS, ZG_OFFSET_H, 1, data+4);
  i2c_write(MPU9250_ADDRESS, ZG_OFFSET_L, 1, data+5);
  
// Output scaled gyro biases for display in the main program
  mGyroBias[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
  mGyroBias[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  mGyroBias[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  i2c_read(MPU9250_ADDRESS, XA_OFFSET_H, 2, data); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  i2c_read(MPU9250_ADDRESS, YA_OFFSET_H, 2, data);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  i2c_read(MPU9250_ADDRESS, ZA_OFFSET_H, 2, data);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  
  for(ii = 0; ii < 3; ii++) {
    if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }
  
  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);
  
  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for display in the main program
   mAccelBias[0] = (float)accel_bias[0]/(float)accelsensitivity; 
   mAccelBias[1] = (float)accel_bias[1]/(float)accelsensitivity;
   mAccelBias[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

void MPU9250::magcal() 
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0}, mag_temp[3] = {0, 0, 0};
  nsecs_t timestamp;
  
   sample_count = 64;
   for(ii = 0; ii < sample_count; ii++) {
    readMagData(mag_temp, &timestamp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay_ms(1);  // at 8 Hz ODR, new mag data is available every 125 ms
   }
   // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    mMagbias[0] = (float) mag_bias[0]*mMagRes*mMagCalibration[0];  // save mag biases in G for main program
    mMagbias[1] = (float) mag_bias[1]*mMagRes*mMagCalibration[1];   
    mMagbias[2] = (float) mag_bias[2]*mMagRes*mMagCalibration[2];  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    magScale[0] = avg_rad/((float)mag_scale[0]);
    magScale[1] = avg_rad/((float)mag_scale[1]);
    magScale[2] = avg_rad/((float)mag_scale[2]);         

}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250::selfTest() // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
   float factoryTrim[6];
   uint8_t FS = 0;

   uint8_t meta = 0x00;
   
  i2c_write(MPU9250_ADDRESS, SMPLRT_DIV, 1, &meta);    // Set gyro sample rate to 1 kHz
  meta = 0x02;
  i2c_write(MPU9250_ADDRESS, CONFIG, 1, &meta);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
  meta = FS<<3;
  i2c_write(MPU9250_ADDRESS, GYRO_CONFIG,1, &meta );  // Set full scale range for the gyro to 250 dps
  meta = 0x02;
  i2c_write(MPU9250_ADDRESS, ACCEL_CONFIG2, 1, &meta); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
  meta = FS<<3;
  i2c_write(MPU9250_ADDRESS, ACCEL_CONFIG, 1, &meta); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
  
  i2c_read(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawData);        // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
  i2c_read(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawData);       // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }
  
// Configure the accelerometer for self-test
  meta = 0xE0;
   i2c_write(MPU9250_ADDRESS, ACCEL_CONFIG, 1, &meta); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   i2c_write(MPU9250_ADDRESS, GYRO_CONFIG,  1, &meta); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   delay_ms(1);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
  
  i2c_read(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawData);  // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  
    i2c_read(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawData);  // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;  
  gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ; 
  }
  
  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }   
  
 // Configure the gyro and accelerometer for normal operation
  meta = 0x00;
   i2c_write(MPU9250_ADDRESS, ACCEL_CONFIG, 1, &meta);  
   i2c_write(MPU9250_ADDRESS, GYRO_CONFIG,  1, &meta);  
   delay_ms(1);  // Delay a while to let the device stabilize
   
   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   i2c_read(MPU9250_ADDRESS, SELF_TEST_X_ACCEL, 1 , selfTest); // X-axis accel self-test results
   i2c_read(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL, 1 , selfTest+1); // Y-axis accel self-test results
   i2c_read(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL, 1 , selfTest+2); // Z-axis accel self-test results
   i2c_read(MPU9250_ADDRESS, SELF_TEST_X_GYRO, 1 , selfTest+3);  // X-axis gyro self-test results
   i2c_read(MPU9250_ADDRESS, SELF_TEST_Y_GYRO, 1 , selfTest+4);  // Y-axis gyro self-test results
   i2c_read(MPU9250_ADDRESS, SELF_TEST_Z_GYRO, 1 , selfTest+5);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation
 
 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (int i = 0; i < 3; i++) {
     mSelfTest[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.;   // Report percent differences
     mSelfTest[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
   }

   printf("self test (acc, gyro):%.2f %.2f %.2f %.2f %.2f %.2f\n", 
                                mSelfTest[0],mSelfTest[1],mSelfTest[2],
                                mSelfTest[3],mSelfTest[4],mSelfTest[5]);
   
}

void MPU9250::setup(){
    uint8_t c = 0;
    i2c_read(MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, &c);  // Read WHO_AM_I register for MPU-9250
    if (c == 0x71 || c == 0x73){
        printf("MPU9250 is online...\n");
        selfTest();
        // get sensor resolutions, only need to do this once
        buildMagRes();
        buildAccRes();
        buildGyroRes();
        // Calibrate gyro and accelerometers, load biases in bias registers
        //accelgyrocal(); 
        delay_ms(1); 
        initMPU9250();
        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        uint8_t d = 0;
        i2c_read(AK8963_ADDRESS, WHO_AM_I_AK8963, 1, &d);  // Read WHO_AM_I register for AK8963 
        // Get magnetometer calibration from AK8963 ROM
        initAK8963();
        magcal();
        delay_ms(1);
        printf("chip setup done.\n");
  

    }else{
        printf("Could not connect to MPU9250\n");
    }
}

status_t MPU9250::process(sensors_event_t *accData, sensors_event_t *gyroData, sensors_event_t *magData){
    // If intPin goes high, all data registers have new data
  uint8_t data = 0;
  int64_t timestamp = 0;

  while(1){
  	i2c_read(MPU9250_ADDRESS, INT_STATUS, 1, &data);
  	if ( data & 0x01) {  // check if data ready interrupt
 // if (digitalRead(intPin)) {  // On interrupt, read data
       readAccelData(mAccelCount, &timestamp);  // Read the x/y/z adc values
 
    // Now we'll calculate the accleration value into actual g's
       // accData->data[0] = ((float)mAccelCount[0]*mAccRes - mAccelBias[0])*9.8f;  // get actual g value, this depends on scale being set
       // accData->data[1] = ((float)mAccelCount[1]*mAccRes - mAccelBias[1])*9.8f;   
       // accData->data[2] = ((float)mAccelCount[2]*mAccRes - mAccelBias[2])*9.8f;  
       accData->data[0] = ((float)mAccelCount[0]*mAccRes);  // get actual g value, this depends on scale being set
       accData->data[1] = ((float)mAccelCount[1]*mAccRes);   
       accData->data[2] = ((float)mAccelCount[2]*mAccRes); 
       accData->timestamp = timestamp;
       accData->type = SENSOR_TYPE_ACCELEROMETER;
   
       readGyroData(mGyroCount, &timestamp);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
       gyroData->data[0] = (float)(mGyroCount[0])*mGyroRes * PI / 180.0f;  // get actual gyro value, this depends on scale being set
       gyroData->data[1] = (float)(mGyroCount[1])*mGyroRes * PI / 180.0f;  
       gyroData->data[2] = (float)(mGyroCount[2])*mGyroRes * PI / 180.0f;  
       gyroData->timestamp = timestamp;
       gyroData->type = SENSOR_TYPE_GYROSCOPE;
  
       readMagData(mMagCount, &timestamp);  // Read the x/y/z adc values
    
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
       magData->data[0] = ((float)mMagCount[0]*mMagRes*mMagCalibration[0] - mMagbias[0])*magScale[0];  // get actual magnetometer value, this depends on scale being set
       magData->data[1] = ((float)mMagCount[1]*mMagRes*mMagCalibration[1] - mMagbias[1])*magScale[1];  
       magData->data[2] = ((float)mMagCount[2]*mMagRes*mMagCalibration[2] - mMagbias[2])*magScale[2];   
       magData->timestamp = timestamp;
       magData->type = SENSOR_TYPE_MAGNETIC_FIELD;
       break;
    }   
  }
  
  return OK;
}