#ifndef ADXL345_H
#define ADXL345_H

#include <Arduino.h>

#define ADXL345_ADDRESS 0x53
#define ADXL345_BYTES_TO_READ 2 * 3 // 2 byets for each axis
#define ADXL345_FIRST_AXIS 0x32

#define ADXL345_MAGIC_NUMBER_X -23
#define ADXL345_MAGIC_NUMBER_Y -11
#define ADXL345_MAGIC_NUMBER_Z 18

class ADXL345 
{

public:
    ADXL345() 
    {
        init();
    }

    void readData(int16_t* result);

protected: 
    void init();    

};

#endif
