#include "ITG3200.h"

#include "functions.h"

void ITG3200::init() 
{
	/*****************************************
	* ITG 3200
	* power management set to:
	* clock select = internal oscillator
	*     no reset, no sleep mode
	*   no standby mode
	* sample rate to = 125Hz
	* parameter to +/- 2000 degrees/sec
	* low pass filter = 5Hz
	* no interrupt
	******************************************/
	Funcs::writeTo(ITG3200_ADDRESS, ITG3200_PWR_MGM    , 0x00);
	Funcs::writeTo(ITG3200_ADDRESS, ITG3200_SMPLRT_DIV , 0x07);
	Funcs::writeTo(ITG3200_ADDRESS, ITG3200_DLPF_FS    , 0x1E);
	Funcs::writeTo(ITG3200_ADDRESS, ITG3200_INT_CFG    , 0x00);
}

void ITG3200::readData(int16_t* result) 
{
	byte  buff[ITG3200_BYTES_TO_READ];

	Funcs::readFrom(ITG3200_ADDRESS, ITG3200_FIRST_AXIS, ITG3200_BYTES_TO_READ, buff); 
	result[0] = ((int16_t(buff[2] << 8) | buff[3]) / ITG3200_GAIN_X) + ITG3200_MAGIC_NUMBER_X;
	result[1] = ((int16_t(buff[4] << 8) | buff[5]) / ITG3200_GAIN_Y) + ITG3200_MAGIC_NUMBER_Y;
	result[2] = ((int16_t(buff[6] << 8) | buff[7]) / ITG3200_GAIN_Z) + ITG3200_MAGIC_NUMBER_Z;
	result[3] = (35 + (int16_t(buff[0] << 8 | buff[1]) + 13200) / 280.0); // Temperature in C
}
