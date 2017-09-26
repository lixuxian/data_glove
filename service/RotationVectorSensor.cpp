#include "RotationVectorSensor.h"

RotationVectorSensor::RotationVectorSensor()
    : mGyroTime(0)
{
    // 200 Hz for gyro events is a good compromise between precision
    // and power/cpu usage.
    mEstimatedGyroRate = 200;
    mFusion.init();

}

void RotationVectorSensor::process(sensors_event_t* outEvent, float *accData, float *gyroData, float *magData, int64_t gryoTimestamp){
	build_inv_gyro(gyroData, gryoTimestamp);
	build_inv_mag(magData);
	build_inv_acc(accData);

    if(hasEstimate()){
        outEvent->data[0] = mAttitude.x;
        outEvent->data[1] = mAttitude.y;
        outEvent->data[2] = mAttitude.z;
        outEvent->data[3] = mAttitude.w;
        outEvent->type = SENSOR_TYPE_ROTATION_VECTOR;
        outEvent->timestamp = systemTime();
    }

}

void RotationVectorSensor::build_inv_acc(float *accData){
        const vec3_t acc(accData);
        mFusion.handleAcc(acc);
        mAttitude = mFusion.getAttitude();
}

void RotationVectorSensor::build_inv_gyro(float *gyroData, int64_t timestamp){

        if (mGyroTime != 0) {
            const float dT = (timestamp - mGyroTime) / 1000000000.0f;
            mFusion.handleGyro(vec3_t(gyroData), dT);
            // here we estimate the gyro rate (useful for debugging)
            const float freq = 1 / dT;
            if (freq >= 100 && freq<1000) { // filter values obviously wrong
                const float alpha = 1 / (1 + dT); // 1s time-constant
                mEstimatedGyroRate = freq + (mEstimatedGyroRate - freq)*alpha;
            }
        }
        mGyroTime = timestamp;
}
void RotationVectorSensor::build_inv_mag(float *magData){
        const vec3_t mag(magData);
        mFusion.handleMag(mag);
}