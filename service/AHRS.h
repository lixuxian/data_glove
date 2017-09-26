#ifndef _AHRS_H
#define _AHRS_H

#include "utils/timers.h"

class AHRS{
private:
     
    float eInt[3];
    static float PI;
    float GyroMeasError;
    float GyroMeasDrift;
    float beta;
    float zeta;
    float deltat;
    int inited;

    nsecs_t lastUpdate;

public:
	float q[4];
	AHRS();
    void QuaternionInit(float *a, float *m);
    void MadgwickUpdate(float *a, float *g, float *m, nsecs_t now);
    void buildEulerAngles(float *yaw, float *pitch, float *roll );
    void MahonyUpdate(float *a, float *g, float *m, nsecs_t now);
    void MahonyUpdateIMU(float *a, float *g, nsecs_t now);

};



#endif
