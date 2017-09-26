#ifndef _SENSOR_SERVICE_H
#define _SENSOR_SERVICE_H

#include "utils/timers.h"
#include "SensorListener.h"
#include "sensors.h"

class SensorSerivce{
private:
    nsecs_t mCurrNanosec;
    SensorListener mListener;
    int *mChannels;
    int mNumberOfChannels;
    int mCurrentChannelIndex;

    sensors_event_t mAccData;
    sensors_event_t mGyroData;
    sensors_event_t mMagData;
    sensors_event_t mRotationVector;
    sensors_event_t mEulerAngles;
    sensors_event_t multiEulerAngles[N_SENSORS];


    void do_service();

public:
    SensorSerivce(int *channels, int nChannel);
    void start_service(nsecs_t nanoseconds);
    void open_device();
};



#endif