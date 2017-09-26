/*different chip need to impliments the sensors interface*/
#include "mpu9250.h"
#include "sensors.h"
#include "tca9548.h"
// #include "RotationVectorSensor.h"
#include "AHRS.h"
#include <stdio.h>

MPU9250 g_mpu_chips[N_SENSORS];
// RotationVectorSensor g_rv_sensors[N_SENSORS];
AHRS  g_ahrs[N_SENSORS];



void setup_sensors(int *channels, int n_channels){
    int i = 0;
    init_channel();
    for(; i<n_channels; i++){
       select_channel( channels[i]);
       g_mpu_chips[ channels[i] ].setup();
    }

}

status_t store_sensor_data(sensors_event_t *accData, sensors_event_t * gyroData, sensors_event_t *magData,  int channel){
	select_channel(channel);
	g_mpu_chips[channel].process(accData, gyroData, magData);
	accData->channel = channel;
	magData->channel = channel;
	gyroData->channel = channel;

	return OK;
}

sensors_event_t g_acc_data_tmp;
sensors_event_t g_mag_data_tmp;
sensors_event_t g_gyro_data_tmp;

status_t store_rotation_vector(sensors_event_t *rotation_vector, int channel){
    // select_channel(channel);
    // store_sensor_data(&g_acc_data_tmp, &g_gyro_data_tmp, &g_mag_data_tmp, channel);
    // rotation_vector->channel = channel;
    // rotation_vector->type = SENSOR_TYPE_ROTATION_VECTOR;
    
    // g_rv_sensors[channel].process(rotation_vector,g_acc_data_tmp.data, g_gyro_data_tmp.data, g_mag_data_tmp.data, g_gyro_Data_tmp.timestamp);
    return OK;
}

status_t store_euler_angles(sensors_event_t *euler, int channel){
    select_channel(channel);
    store_sensor_data(&g_acc_data_tmp, &g_gyro_data_tmp, &g_mag_data_tmp, channel);
    euler->channel = channel;
    euler->type = SENSOR_TYPE_EULER_ANGLE;
    // printf("acc:%.2f %.2f %.2f\t gyro:%.2f %.2f %.2f\t mag:%.2f %.2f %.2f time=%lld\n",
    //     g_acc_data_tmp.data[0],g_acc_data_tmp.data[1],g_acc_data_tmp.data[2],
    //     g_gyro_data_tmp.data[0],g_gyro_data_tmp.data[1],g_gyro_data_tmp.data[2],
    //     g_mag_data_tmp.data[0],g_mag_data_tmp.data[1],g_mag_data_tmp.data[2],
    //     g_mag_data_tmp.timestamp);
    g_ahrs[channel].QuaternionInit(g_acc_data_tmp.data, g_mag_data_tmp.data);
    // g_ahrs[channel].MadgwickUpdate(g_acc_data_tmp.data,  g_gyro_data_tmp.data, g_mag_data_tmp.data, g_mag_data_tmp.timestamp);
    g_ahrs[channel].MahonyUpdate(g_acc_data_tmp.data, g_gyro_data_tmp.data, g_mag_data_tmp.data, g_mag_data_tmp.timestamp);
    // g_ahrs[channel].MahonyUpdateIMU(g_acc_data_tmp.data, g_gyro_data_tmp.data, g_mag_data_tmp.timestamp);
    g_ahrs[channel].buildEulerAngles(&(euler->data[0]), &(euler->data[1]), &(euler->data[2]));
    //printf("quat:%.2f %.2f %.2f %.2f\n", g_ahrs[channel].q[0],g_ahrs[channel].q[1],g_ahrs[channel].q[2],g_ahrs[channel].q[3]);
    return OK;
}

status_t store_multiple_euler_angles(sensors_event_t *euler, int *channels, int n_channels){
    int ii = 0;
    for(ii=0; ii<n_channels; ii++){
        store_euler_angles(euler+ii, channels[ii]);
    }
    return OK;
}
