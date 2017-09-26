#include "SensorListener.h"
#include "data_sender.h"
#include "data_dumper.h"  // add by lixuxian 2017/06/27
#include "utils/timers.h" // add by lixuxian 2017/06/29
#include "bt_send.h"
#include <stdio.h>
#include <sstream>
#include <string>
#include <string.h>
using namespace std;

void SensorListener::onSensorChanged(sensors_event_t& event){
	char sensor_data[128];
	sprintf(sensor_data,"%d#%.2f,%.2f,%.2f",event.channel, event.data[0], event.data[1], event.data[2]);
    //printf("sensor listener:%.2f, %.2f, %.2f\n", event.data[0], event.data[1], event.data[2]);
    send(sensor_data);
    //printf("%s\n", sensor_data);
}
char sensor_data_string[1024];
void SensorListener::onMultiSensorChanged(sensors_event_t *events, int nEvents){
    stringstream ss;
    string sensor_data;
    sensors_event_t *event;
    int i=0;
    for(; i<nEvents; i++){
        event = events+i;
        ss<< event->channel << "#" << event->data[0] << "," << event->data[1] << ","<< event->data[2] << ";";
    }

    ss >> sensor_data;
    const int len = sensor_data.length();
    strncpy(sensor_data_string, sensor_data.c_str(), len);
    sensor_data_string[len] = '\0';
    //printf("%s len=%d\n", sensor_data_string,strlen(sensor_data_string));
    send(sensor_data_string);
    //send_bt(sensor_data_string);
    dump("data/data_dumper_test.txt", sensor_data_string); //add by lixuxian 2017/06/27
} 
