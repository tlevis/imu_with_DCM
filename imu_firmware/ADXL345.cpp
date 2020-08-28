#include "ADXL345.h"

#include "functions.h"

void ADXL345::init()
{
    // Turning on the ADXL345
    Funcs::writeTo(ADXL345_ADDRESS, 0x2D, 0);      
    Funcs::writeTo(ADXL345_ADDRESS, 0x2D, 16);
    Funcs::writeTo(ADXL345_ADDRESS, 0x2D, 8);
}	

void ADXL345::readData(int16_t* result) 
{
    byte buff[ADXL345_BYTES_TO_READ];
	
    Funcs::readFrom(ADXL345_ADDRESS, ADXL345_FIRST_AXIS, ADXL345_BYTES_TO_READ, buff); 
    result[0] = (((int16_t)buff[1]) << 8) | buff[0] + ADXL345_MAGIC_NUMBER_X;
    result[1] = (((int16_t)buff[3]) << 8) | buff[2] + ADXL345_MAGIC_NUMBER_Y;
    result[2] = (((int16_t)buff[5]) << 8) | buff[4] + ADXL345_MAGIC_NUMBER_Z;

}
