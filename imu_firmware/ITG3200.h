#ifndef ITG3200_H
#define ITG3200_H

#include <Arduino.h>

#define ITG3200_ADDRESS 0x68 // gyro address, binary = 11101001 when AD0 is connected to Vcc (see schematics of your breakout board)
#define ITG3200_FIRST_AXIS 0x1B

#define ITG3200_SMPLRT_DIV 0x15
#define ITG3200_DLPF_FS 0x16
#define ITG3200_INT_CFG 0x17
#define ITG3200_PWR_MGM 0x3E

#define ITG3200_GAIN_X  14.375f
#define ITG3200_GAIN_Y  14.375f
#define ITG3200_GAIN_Z  14.375f

#define ITG3200_BYTES_TO_READ 8 // 2 bytes for each axis x, y, z, temp

// To Remove:
//#define Gyro_Gain 2.5 //2.5Gyro gain
//#define Gyro_Scaled(x) x* ((Gyro_Gain*PI)/360) //Return the scaled ADC raw data of the gyro in radians for second


#define ITG3200_MAGIC_NUMBER_X 2
#define ITG3200_MAGIC_NUMBER_Y 0
#define ITG3200_MAGIC_NUMBER_Z 0.5

class ITG3200 
{

public:
    ITG3200() 
    { 
        init();
    }

    void readData(int16_t* result);

protected:
    void init();

};

#endif