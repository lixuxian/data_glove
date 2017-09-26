#ifndef _ROTATION_VECTOR_SENSOR_H
#define _ROTATION_VECTOR_SENSOR_H

#include "utils/timers.h"
#include "utils/vec.h"
#include "utils/mat.h"
#include "sensors.h"
#include "Fusion.h"


class RotationVectorSensor{

private:

    float mEstimatedGyroRate;
    nsecs_t mGyroTime;
    vec4_t mAttitude;
    Fusion mFusion;

    void build_inv_acc(float *accData);
    void build_inv_gyro(float *gyroData, int64_t timestamp);
    void build_inv_mag(float *magData);
    
public:
    RotationVectorSensor();
    void process(sensors_event_t* outEvent, float *accData, float *gyroData, float *magData, int64_t gyroTimestamp);

    bool hasEstimate() const { return mFusion.hasEstimate(); }
    mat33_t getRotationMatrix() const { return mFusion.getRotationMatrix(); }
    vec4_t getAttitude() const { return mAttitude; }
    vec3_t getGyroBias() const { return mFusion.getBias(); }
    float getEstimatedRate() const { return mEstimatedGyroRate; }



};



#endif // _ROTATION_VECTOR_SENSOR_H