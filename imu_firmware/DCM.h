#ifndef DCM_H
#define DCM_H

#include <Arduino.h>

class DCM
{

public:
    DCM() 
    {
        _gdt = 0.005f;

    }


    void collect(int16_t* acc, int16_t* gyro);
    void normalize();
    void driftCorrection();
    void accelAdjust();
    void matrixUpdate();
    void eulerAngles();
    void calculate(int16_t* acc, int16_t* gyro);
    void updateGDT(float value);
    void getDataString(char* str);

protected: 
    float _accelVector[3]  = {0,0,0}; 
    float _gyroVector[3]  = {0,0,0}; 
    float _omegaVector[3]  = {0,0,0}; 

    float _errorRollPitch[3]  = {0,0,0}; 
    float _errorYaw[3]  = {0,0,0}; 

    float _omegaP[3] = {0,0,0};  //Omega Proportional correction
    float _omegaI[3] = {0,0,0}; //Omega Integrator
    float _omega[3] = {0,0,0}; 

    float _DCMMatrix[3][3]       = {{1,0,0},{0,1,0},{0,0,1}}; 
    float _updateMatrix[3][3]    = {{0,1,2},{3,4,5},{6,7,8}}; 
    float _temporaryMatrix[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

    int16_t  _acc[3];
    int16_t  _gyro[4];  

    float _roll;
    float _pitch;
    float _yaw;

    float _gdt;
};

#endif