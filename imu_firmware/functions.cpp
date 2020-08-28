#include "functions.h"
#include <Wire.h>

void Funcs::writeTo(int device, byte address, byte val)  
{
	Wire.beginTransmission(device);
	Wire.write(address);
	Wire.write(val);
	Wire.endTransmission();
}

void Funcs::readFrom(int device, byte address, int num, byte buff[]) 
{
	Wire.beginTransmission(device);
	Wire.write(address);
	Wire.endTransmission();

	Wire.beginTransmission(device);
	Wire.requestFrom(device, num);

	int i = 0;
	while (Wire.available()) 
    {
		buff[i] = Wire.read();
		i++;
	}
	Wire.endTransmission();
}

long Funcs::convert2dec(float x) 
{
    return x * 1000;
}

//Computes the dot product of two vectors
float Funcs::vectorDotProduct(float vector1[3],float vector2[3])
{
    float op = 0;
    for (int c = 0; c < 3; c++)
    {
        op += vector1[c] * vector2[c];
    }
    return op; 
}

//Computes the cross product of two vectors
void Funcs::vectorCrossProduct(float vectorOut[3], float v1[3],float v2[3])
{
    vectorOut[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
    vectorOut[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
    vectorOut[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

//Multiply the vector by a scalar. 
void Funcs::vectorScale(float vectorOut[3], float vectorIn[3], float scale2)
{
    for (int c = 0; c < 3; c++)
    {
        vectorOut[c] = vectorIn[c] * scale2; 
    }
}

void Funcs::vectorAdd(float vectorOut[3], float vectorIn1[3], float vectorIn2[3])
{
    for (int c = 0; c < 3; c++)
    {
        vectorOut[c] = vectorIn1[c] + vectorIn2[c];
    }
}

void Funcs::MatrixMultiply(float a[3][3], float b[3][3], float mat[3][3])
{
    float op[3]; 
    for (int x = 0; x < 3; x++)
    {
        for (int y = 0; y < 3; y++)
        {
            for (int w = 0; w < 3; w++)
            {
                op[w] = a[x][w] * b[w][y];
            } 
            mat[x][y] = 0;
            mat[x][y] = op[0] + op[1] + op[2];
        }
    }
}