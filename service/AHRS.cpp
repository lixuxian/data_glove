#include "AHRS.h"
#include <math.h>
#include <stdio.h>

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, 
#define Ki 0.0f        //Kp for proportional feedback, Ki for integral

#define twoKpDef    (2.0f * 5.0f)   // 2 * proportional gain
#define twoKiDef    (2.0f * 0.0f)   // 2 * integral gain

float twoKp = twoKpDef;                                            // 2 * proportional gain (Kp)
float twoKi = twoKiDef;                                            // 2 * integral gain (Ki)
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f; // integral error terms scaled by Ki

AHRS::AHRS() //beta = 
    :GyroMeasError(PI * (40.0f / 180.0f)), GyroMeasDrift(PI * (1.0f  / 180.0f)),
    beta(sqrt(3.0f / 4.0f) * GyroMeasError), zeta(sqrt(3.0f / 4.0f) * GyroMeasDrift),
    deltat(0.0f), lastUpdate(0)
{
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;

    eInt[0] = 0.0f;
    eInt[1] = 0.0f;
    eInt[2] = 0.0f;

    inited = 0;

}

float AHRS::PI = 3.14159265358979323846f;

void AHRS::QuaternionInit(float *a, float *m){
    if (inited == 0){
        float ax = a[0], ay = a[1], az = a[2];
        float mx = m[0], my = m[1], mz = m[2];
        float roll = atan2f(-ay, -az);
        float pitch = asinf(ax / sqrt(ax * ax + ay * ay + az * az));
        float yaw = atan2f(-my * cos(roll) + mz * sin(roll), 
            mx * cos(pitch) + my * sin(pitch) * sin(roll) + mz * sin(pitch) * cos(roll));
        yaw /= 2;
        pitch /= 2;
        roll /= 2;        
        q[0] = cos(yaw)*cos(pitch)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll);
        q[1] = sin(yaw)*cos(pitch)*cos(roll) - cos(yaw)*sin(pitch)*sin(roll);
        q[2] = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*cos(pitch)*sin(roll);
        q[3] = cos(yaw)*cos(pitch)*sin(roll) - sin(yaw)*sin(pitch)*cos(roll);

        // normalise quaternion
        float norm = 1.0 / (float)sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (norm == 0.0f) return;
        q[0] *= norm;
        q[1] *= norm;
        q[2] *= norm;
        q[3] *= norm;

        printf("quaternion:%f %f %f %f \ninited:%d \n", q[0], q[1], q[2], q[3], inited);

        inited = 1;
    }else {
        // has been inited;
    }
    
 }

void AHRS::buildEulerAngles(float *yaw, float *pitch, float *roll ){
    float  localYaw = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), 1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]);
   float localPitch = asinf(2.0f * (q[0] * q[2] - q[1] * q[3]));
   float localRoll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * q[1] * q[1] - 2 * q[2] * q[2] );
   localPitch *= 180.0f / PI;
   localYaw   *= 180.0f / PI; 
   //yaw   += 13.8f; // 
   // if(localYaw < 0) localYaw   += 360.0f; // Ensure yaw stays between 0 and 360
   localRoll  *= 180.0f / PI;

   *yaw = localYaw;
   *pitch = localPitch;
   *roll = localRoll;
}


// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void AHRS::MadgwickUpdate(float *a, float *g, float *m, nsecs_t now)
{
    if(lastUpdate == 0){
        lastUpdate = now;
       return;
    }
    deltat = (now - lastUpdate)/1000000000.0f;
    lastUpdate = now;

    //printf("deltat %f\n",deltat );

    float ax = a[0];
    float ay = a[1];
    float az = a[2];
    float gx = g[0];
    float gy = g[1];
    float gz = g[2];
    float mx = m[0];
    float my = m[1];
    float mz = m[2];

    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = (float)sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1 / norm;        // use reciprocal for division
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = (float)sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1 / norm;        // use reciprocal for division
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = (float)sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = 1.0f / (float)sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = 1.0f / (float)sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}

void AHRS::MahonyUpdate(float *a, float *g, float *m, nsecs_t now) {

    if(lastUpdate == 0){
        lastUpdate = now;
       return;
    }
    deltat = (now - lastUpdate)/1000000000.0f;
    lastUpdate = now;

    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    // if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    //     MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
    //     return;
    // }

    float ax = a[0];
    float ay = a[1];
    float az = a[2];
    float gx = g[0];
    float gy = g[1];
    float gz = g[2];
    float mx = m[0];
    float my = m[1];
    float mz = m[2];

    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];   // short name local variable for readability

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = (float)sqrt(ax * ax + ay * ay + az * az);
        if (recipNorm == 0.0f) return;
        recipNorm = 1 / recipNorm;
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;     

        // Normalise magnetometer measurement
        recipNorm = (float)sqrt(mx * mx + my * my + mz * mz);
        if (recipNorm == 0.0f) return;
        recipNorm = 1 / recipNorm;
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
    
        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * deltat;    // integral error scaled by Ki
            integralFBy += twoKi * halfey * deltat;
            integralFBz += twoKi * halfez * deltat;
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    
    // Integrate rate of change of quaternion
    gx *= (0.5f * deltat);     // pre-multiply common factors
    gy *= (0.5f * deltat);
    gz *= (0.5f * deltat);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx); 
    
    // Normalise quaternion
    recipNorm = (float)sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    recipNorm = 1 / recipNorm;
    if (recipNorm == 0.0f) return;
    q[0] = q0 * recipNorm;
    q[1] = q1 * recipNorm;
    q[2] = q2 * recipNorm;
    q[3] = q3 * recipNorm;
}

void AHRS::MahonyUpdateIMU(float *a, float *g, nsecs_t now) {

    if(lastUpdate == 0){
        lastUpdate = now;
        return;
    }
    deltat = (now - lastUpdate)/1000000000.0f;
    lastUpdate = now;

    float ax = a[0], ay = a[1], az = a[2];
    float gx = g[0], gy = g[1], gz = g[2];

    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = 1.0 / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;        

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;
    
        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * deltat;    // integral error scaled by Ki
            integralFBy += twoKi * halfey * deltat;
            integralFBz += twoKi * halfez * deltat;
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f; // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    
    // Integrate rate of change of quaternion
    gx *= (0.5f * deltat);     // pre-multiply common factors
    gy *= (0.5f * deltat);
    gz *= (0.5f * deltat);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx); 
    
    // Normalise quaternion
    recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    q[0] = q0;
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}
