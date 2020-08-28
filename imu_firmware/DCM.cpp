#include "DCM.h"
#include "functions.h"


#define GRAVITY 256

#define Kp_ROLLPITCH 0.015 //.015 Pitch&Roll Proportional Gain
#define Ki_ROLLPITCH 0.000010 //0.000005Pitch&Roll Integrator Gain
#define ACCEL_SCALE(x) x * (GRAVITY / 9.81)

#define G_Dt(x) x * 0.05 // DT .02 = 20 miliseconds, value used in derivations and integrations

#define OMEGAA 0

void DCM::normalize()
{
    float error = 0;
    float temporary[3][3];
    float renorm = 0;
    bool  problem = false;
  
    error = -Funcs::vectorDotProduct(&_DCMMatrix[0][0], &_DCMMatrix[1][0])*.5; //eq.19

    Funcs::vectorScale(&temporary[0][0], &_DCMMatrix[1][0], error); //eq.19
    Funcs::vectorScale(&temporary[1][0], &_DCMMatrix[0][0], error); //eq.19
  
    Funcs::vectorAdd(&temporary[0][0], &temporary[0][0], &_DCMMatrix[0][0]);//eq.19
    Funcs::vectorAdd(&temporary[1][0], &temporary[1][0], &_DCMMatrix[1][0]);//eq.19
  
    Funcs::vectorCrossProduct(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b //eq.20
  
    renorm = Funcs::vectorDotProduct(&temporary[0][0], &temporary[0][0]); 
    if (renorm < 1.5625f && renorm > 0.64f) 
    {
        renorm = .5 * (3 - renorm);                                                 //eq.21
    } 
    else if (renorm < 100.0f && renorm > 0.01f) 
    {
        renorm = 1. / sqrt(renorm);  
    } 
    else 
    {
        problem = true;
    }
    Funcs::vectorScale(&_DCMMatrix[0][0], &temporary[0][0], renorm);
  
    renorm = Funcs::vectorDotProduct(&temporary[1][0], &temporary[1][0]); 
    if (renorm < 1.5625f && renorm > 0.64f)
    {
        renorm = .5 * (3 - renorm);                                                 //eq.21
    } 
    else if (renorm < 100.0f && renorm > 0.01f) 
    {
        renorm = 1. / sqrt(renorm);    
    } 
    else 
    {
        problem = true;
    }
    Funcs::vectorScale(&_DCMMatrix[1][0], &temporary[1][0], renorm);
  
    renorm = Funcs::vectorDotProduct(&temporary[2][0],&temporary[2][0]); 
    if (renorm < 1.5625f && renorm > 0.64f) 
    {
        renorm = .5 * (3 - renorm);                                                 //eq.21
    } 
    else if (renorm < 100.0f && renorm > 0.01f) 
    {
        renorm = 1. / sqrt(renorm);  
    } 
    else 
    {
        problem = true;
    }
    Funcs::vectorScale(&_DCMMatrix[2][0], &temporary[2][0], renorm);
  
    if (problem) {                // Our solution is blowing up and we will force back to initial condition.  Hope we are not upside down!
        _DCMMatrix[0][0]= 1.0f;
        _DCMMatrix[0][1]= 0.0f;
        _DCMMatrix[0][2]= 0.0f;
        _DCMMatrix[1][0]= 0.0f;
        _DCMMatrix[1][1]= 1.0f;
        _DCMMatrix[1][2]= 0.0f;
        _DCMMatrix[2][0]= 0.0f;
        _DCMMatrix[2][1]= 0.0f;
        _DCMMatrix[2][2]= 1.0f;
        problem = false;  
    }
}


void DCM::driftCorrection()
{
    float mag_heading_x;
    float mag_heading_y;
    float errorCourse;
    //Compensation the Roll, Pitch and Yaw drift. 
    static float scaledOmegaI[3];
    float accelMagnitude, accelWeight;
    float Integrator_magnitude;
    float mag_projection;
  
    // Calculate the magnitude of the accelerometer vector
    accelMagnitude = sqrt(_accelVector[0] * _accelVector[0] + _accelVector[1] * _accelVector[1] + _accelVector[2] * _accelVector[2]);
    accelMagnitude = accelMagnitude / GRAVITY; // Scale to gravity.
    
    // Dynamic weighting of accelerometer info (reliability filter)
    // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
    accelWeight = constrain(1 - 2 * abs(1 - accelMagnitude), 0, 1);

    Funcs::vectorCrossProduct(&_errorRollPitch[0], &_accelVector[0], &_DCMMatrix[2][0]); //adjust the ground of reference
    Funcs::vectorScale(&_omegaP[0], &_errorRollPitch[0], Kp_ROLLPITCH * accelWeight);
  
    Funcs::vectorScale(&scaledOmegaI[0], &_errorRollPitch[0] ,Ki_ROLLPITCH * accelWeight);
    Funcs::vectorAdd(_omegaI, _omegaI, scaledOmegaI);     
}


void DCM::accelAdjust()
{
    _accelVector[1] = _accelVector[1] - (ACCEL_SCALE(0) * _omega[2]); 
    _accelVector[2] = _accelVector[2] + (ACCEL_SCALE(0) * _omega[1]); 
}

void DCM::matrixUpdate()
    {
    _gyroVector[0] = Funcs::ToRad(_gyro[0]); // gyro y roll
    _gyroVector[1] = Funcs::ToRad(_gyro[1]); // gyro x pitch
    _gyroVector[2] = Funcs::ToRad(_gyro[2]); // gyro Z yaw

    _accelVector[0] = _acc[0]; // x
    _accelVector[1] = _acc[1]; // y
    _accelVector[2] = _acc[2]; // z

    Funcs::vectorAdd(&_omega[0], &_gyroVector[0], &_omegaI[0]); //adding proportional
    Funcs::vectorAdd(&_omegaVector[0], &_omega[0], &_omegaP[0]); //adding Integrator
  
    accelAdjust(); //adjusting centrifugal acceleration.

    _updateMatrix[0][0]= 0;
    _updateMatrix[0][1]= -_gdt * _omegaVector[2];  //-z
    _updateMatrix[0][2]= _gdt * _omegaVector[1];   //y
    _updateMatrix[1][0]= _gdt * _omegaVector[2];   //z
    _updateMatrix[1][1]= 0;
    _updateMatrix[1][2]= -_gdt * _omegaVector[0];  //-x
    _updateMatrix[2][0]= -_gdt * _omegaVector[1];  //-y
    _updateMatrix[2][1]= _gdt * _omegaVector[0];   //x
    _updateMatrix[2][2]= 0;


    Funcs::MatrixMultiply(_DCMMatrix, _updateMatrix, _temporaryMatrix); //a*b=c

    for (int x = 0; x < 3; x++) //Matrix Addition (update)
    {
        for (int y = 0; y < 3; y++)
        {
            _DCMMatrix[x][y] += _temporaryMatrix[x][y];
        } 
    }
}

void DCM::eulerAngles(void)
{
    _pitch = -asin(_DCMMatrix[2][0]);
    _roll = atan2(_DCMMatrix[2][1], _DCMMatrix[2][2]);
    _yaw = atan2(_DCMMatrix[1][0], _DCMMatrix[0][0]);
}

void DCM::calculate(int16_t* acc, int16_t* gyro)
{
    for (int i = 0; i < 4; i++) 
    {
        if (i < 3)
            _acc[i] = acc[i];
        
        _gyro[i] = gyro[i];
    }

    matrixUpdate();
    normalize();
    driftCorrection();
    eulerAngles();
}

void DCM::updateGDT(float value)
{
    _gdt = value;
}

void DCM::getDataString(char* str)
{
    sprintf(str, "%d,%d,%d,%d,%d,%d,%d,%f,%f,%f\n", _acc[0], _acc[1], _acc[2], _gyro[0], _gyro[1], _gyro[2], _gyro[3], Funcs::ToDeg(_roll), Funcs::ToDeg(_pitch), Funcs::ToDeg(_yaw));  
}