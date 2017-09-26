#ifndef _SENSOR_LISTENER_H
#define _SENSOR_LISTENER_H

#include <stdio.h>
#include "sensors.h"


class SensorListener{
public:
    void onSensorChanged(sensors_event_t& event);
    void onMultiSensorChanged(sensors_event_t *events, int nEvents);
};



#endif