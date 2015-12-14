#ifndef _MyMPU9150_H_
#define _MyMPU9150_H_

#include <math.h>

#define MPU9150_SMPLRT_DIV         0x19   // R/W
#define MPU9150_CONFIG             0x1A   // R/W
#define MPU9150_GYRO_CONFIG        0x1B   // R/W
#define MPU9150_ACCEL_CONFIG       0x1C   // R/W
#define MPU9150_I2C_MST_CTRL       0x24   // R/W
#define MPU9150_I2C_SLV0_ADDR      0x25   // R/W
#define MPU9150_I2C_SLV0_REG       0x26   // R/W
#define MPU9150_I2C_SLV0_CTRL      0x27   // R/W
#define MPU9150_I2C_SLV1_ADDR      0x28   // R/W
#define MPU9150_I2C_SLV1_REG       0x29   // R/W
#define MPU9150_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU9150_I2C_SLV4_CTRL      0x34   // R/W
#define MPU9150_ACCEL_XOUT_H       0x3B   // R  
#define MPU9150_ACCEL_XOUT_L       0x3C   // R  
#define MPU9150_ACCEL_YOUT_H       0x3D   // R  
#define MPU9150_ACCEL_YOUT_L       0x3E   // R  
#define MPU9150_ACCEL_ZOUT_H       0x3F   // R  
#define MPU9150_ACCEL_ZOUT_L       0x40   // R  
#define MPU9150_GYRO_XOUT_H        0x43   // R  
#define MPU9150_GYRO_XOUT_L        0x44   // R  
#define MPU9150_GYRO_YOUT_H        0x45   // R  
#define MPU9150_GYRO_YOUT_L        0x46   // R  
#define MPU9150_GYRO_ZOUT_H        0x47   // R  
#define MPU9150_GYRO_ZOUT_L        0x48   // R  
#define MPU9150_I2C_SLV1_DO        0x64   // R/W
#define MPU9150_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU9150_USER_CTRL          0x6A   // R/W
#define MPU9150_PWR_MGMT_1         0x6B   // R/W

//MPU9150 Compass
#define MPU9150_CMPS_XOUT_L        0x4A   // R
#define MPU9150_CMPS_XOUT_H        0x4B   // R
#define MPU9150_CMPS_YOUT_L        0x4C   // R
#define MPU9150_CMPS_YOUT_H        0x4D   // R
#define MPU9150_CMPS_ZOUT_L        0x4E   // R
#define MPU9150_CMPS_ZOUT_H        0x4F   // R

// CALIBREATION AND FILTERING PARAMETERS
#define SGA_MAX_LENGTH 9
#define SGA_MID 5
#define SGA_INDEX 3
#define SGA_HIST_LENGTH 9

#define ACC_SENS 2048 // AccelSensitivity --- 2g=16384, 4g=8192, 8g=4096, 16g=2048, 32g=1024, 64g=512, 128g=256, 256g=128, 512g=64, 1024g=32
#define GYR_SENS 131 // GyroSensitivity --- 250dps=131(131.072), 500dps=65.5(65.536), 1000dps=32.8(32.768), 2000dps=16.4(16.384)

// I2C address 0x69 could be 0x68 depends on your wiring. 
int MPU9150_I2C_ADDRESS = 0x68;

class MyMPU9150 
{

private: 
  uint8_t L;
  uint8_t H;
  float COMPASS_OFFSET;
  float head_hist[9];

public:

  MyMPU9150 ()
  {
    COMPASS_OFFSET = 0.0;
  }
  void MPU9150_setupAccGyr()
  {
    MPU9150_writeSensor(0x6B, 0x08); //disable sleep and temp-sens
    MPU9150_writeSensor(0x1A, 0x01); //enable DLPF value 1
    MPU9150_writeSensor(0x1B, 0x00); //set gyro range to 0 - 250dps
    MPU9150_writeSensor(0x1C, 0x08); //set accel range to 1 - 4g 
    MPU9150_writeSensor(0x19, 0x05); // Set SMPL_DIV  
  
  }

  void MPU9150_setupCompass()
  {
    MPU9150_I2C_ADDRESS = 0x0C;      //change Adress to Compass

    MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode
    MPU9150_writeSensor(0x0A, 0x0F); //SelfTest
    MPU9150_writeSensor(0x0A, 0x00); //PowerDownMode

    MPU9150_I2C_ADDRESS = 0x68;      //change Adress to MPU

    MPU9150_writeSensor(0x24, 0x40); //Wait for Data at Slave0
    MPU9150_writeSensor(0x25, 0x8C); //Set i2c address at slave0 at 0x0C
    MPU9150_writeSensor(0x26, 0x02); //Set where reading at slave 0 starts
    MPU9150_writeSensor(0x27, 0x88); //set offset at start reading and enable
    MPU9150_writeSensor(0x28, 0x0C); //set i2c address at slv1 at 0x0C
    MPU9150_writeSensor(0x29, 0x0A); //Set where reading at slave 1 starts
    MPU9150_writeSensor(0x2A, 0x81); //Enable at set length to 1
    MPU9150_writeSensor(0x64, 0x01); //overvride register
    MPU9150_writeSensor(0x67, 0x03); //set delay rate
    MPU9150_writeSensor(0x01, 0x80);

    MPU9150_writeSensor(0x34, 0x04); //set i2c slv4 delay
    MPU9150_writeSensor(0x64, 0x00); //override register
    MPU9150_writeSensor(0x6A, 0x00); //clear usr setting
    MPU9150_writeSensor(0x64, 0x01); //override register
    MPU9150_writeSensor(0x6A, 0x20); //enable master i2c mode
    MPU9150_writeSensor(0x34, 0x13); //disable slv4
  }

  ////////////////////////////////////////////////////////////
  ///////// I2C functions to get easier all values ///////////
  ////////////////////////////////////////////////////////////

  float MPU9150_readSensor(int addrL, int addrH)
  {
    Wire.beginTransmission(MPU9150_I2C_ADDRESS);
    Wire.write(addrL);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
    L = Wire.read();

    Wire.beginTransmission(MPU9150_I2C_ADDRESS);
    Wire.write(addrH);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
    H = Wire.read();

    return (int16_t)((H<<8)+L);
  }

  int MPU9150_readSensor(int addr)
  {
    Wire.beginTransmission(MPU9150_I2C_ADDRESS);
    Wire.write(addr);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU9150_I2C_ADDRESS, 1, true);
    return Wire.read();
  }

  bool MPU9150_writeSensor(int addr,int data)
  {
    Wire.beginTransmission(MPU9150_I2C_ADDRESS);
    Wire.write(addr);
    Wire.write(data);
    Wire.endTransmission(true);

    return 1;
  }
  
  void set_accel_sleep()
  {
    MPU9150_writeSensor(0x6B, 0x48); //Enable sleep
    
  }
    
  void wake_accel()
  {
    MPU9150_writeSensor(0x6B, 0x08); //disable sleep and temp-sens 
    MPU9150_writeSensor(0x1A, 0x01); //enable DLPF value 1
    MPU9150_writeSensor(0x1C, 0x08); //set accel range to 1 - 4g 
     
  } 
  
  void setup_hist()
  {
    /* get Heading history vector */
    for(int i=0;i<5;++i)
    { 
      float head=0.0;
      get_Heading(&head);
      head_hist[i] = head;
    }

  }
  void get_Heading(float* heading)
  {
    *heading = ((atan2((float)MPU9150_readSensor(MPU9150_CMPS_YOUT_L,MPU9150_CMPS_YOUT_H), (float)MPU9150_readSensor(MPU9150_CMPS_XOUT_L,MPU9150_CMPS_XOUT_H)))* 180.0/3.14159265 + 180) - COMPASS_OFFSET;
    if (*heading < 0) 
      *heading += 360;
    else if (*heading > 360) 
      *heading -= 360; 
  }

  float get_SGAfilt(float val)
  {
    /* SGA Coefficients*/
    const float sga_coefficients[SGA_MAX_LENGTH]=
    {
      // 0, -2, 3, 6, 7, 6, 3, -2, 0 
     -0.090909091,
      0.060606061,
      0.168831169,
      0.233766234,
      0.255411255,
      0.233766234,
      0.168831169,
      0.060606061,
      -0.090909091
    };
    float sum=0;
    int8_t i=0;

    //Right Shift the hist values to update the current value
    for(i=1;i < SGA_HIST_LENGTH; ++i)
      head_hist[i-1] = head_hist[i];

    head_hist[SGA_HIST_LENGTH-1] = val;

    // Savitzky Golay Filtering
    for(i= -SGA_MID; i<SGA_MID;++i)
    {
      sum+= head_hist[i+SGA_MID]*sga_coefficients[i+SGA_MID];
    }

    //hist[SGA_MID] = (float)sum/norm;
    if(head_hist[SGA_MID] < 0)
      head_hist[SGA_MID] = 0;

    val = head_hist[SGA_MID];
    return val;

  }

  void set_CompassOffset(float value)
  {
    COMPASS_OFFSET = value;
  }


 

};
#endif








