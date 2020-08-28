#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>

class Funcs 
{
public:
    static float ToDeg(float x) { return (x * 180.0f) / PI; }
    static float ToRad(float x) { return (x * PI) / 180.0f; }
    static float ToDeg(int16_t x) { return (x * 180.0f) / PI; }
    static float ToRad(int16_t x) { return  (x * PI) / 180.0f; }
    static void writeTo(int device, byte address, byte val);
    static void readFrom(int device, byte address, int num, byte buff[]);
    static long convert2dec(float x);
    static float vectorDotProduct(float vector1[3],float vector2[3]);
    static void vectorCrossProduct(float vectorOut[3], float v1[3],float v2[3]);
    static void vectorScale(float vectorOut[3], float vectorIn[3], float scale2);
    static void vectorAdd(float vectorOut[3], float vectorIn1[3], float vectorIn2[3]);
    static void MatrixMultiply(float a[3][3], float b[3][3], float mat[3][3]);
};


#endif