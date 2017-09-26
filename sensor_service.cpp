#include "sensor_service.h"
#include <stdio.h>

SensorSerivce::SensorSerivce(int *channels, int nChannel) 
    : mCurrNanosec(0), mNumberOfChannels(nChannel), mCurrentChannelIndex(0) {
    mChannels = channels;
}

void SensorSerivce::do_service(){
    
    // store_euler_angles(&mEulerAngles, mChannels[mCurrentChannelIndex]);

    // mListener.onSensorChanged(mEulerAngles);

    store_multiple_euler_angles(multiEulerAngles, mChannels, mNumberOfChannels);
    mListener.onMultiSensorChanged(multiEulerAngles, mNumberOfChannels);

    //mCurrentChannelIndex = (mCurrentChannelIndex + 1) % mNumberOfChannels;
}

void SensorSerivce::open_device(){
    setup_sensors(mChannels, mNumberOfChannels);
}

void SensorSerivce::start_service(nsecs_t nanoseconds){

    nsecs_t local_nanosec = 0;

    while(1){
        local_nanosec = systemTime();
        if( local_nanosec - mCurrNanosec >= nanoseconds ){
            do_service();
            mCurrNanosec = local_nanosec;
        }
    }
}
