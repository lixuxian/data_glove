#ifndef _SENSORS_H
#define _SENSORS_H

#include <stdint.h>

#define SENSOR_TYPE_ACCELEROMETER                    (1)
#define SENSOR_TYPE_MAGNETIC_FIELD                   (2)
#define SENSOR_TYPE_GYROSCOPE                        (3)
#define SENSOR_TYPE_ROTATION_VECTOR                  (4)
#define SENSOR_TYPE_EULER_ANGLE                      (5)
#define SENSOR_MULTIPLE_EULER_ANGLE                  (6)

#define N_SENSORS 16

enum _status_t
{
    OK = 0,
    FAIL = 1,
};

typedef enum _status_t status_t;

typedef struct sensors_event_t {

    /* sensor type */
    int32_t type;
    /* time is in nanosecond */
    int64_t timestamp;

    float           data[4];

    int channel;

} sensors_event_t;

void setup_sensors(int *channels, int n_channels);

status_t store_sensor_data(sensors_event_t *accData, sensors_event_t * gyroData, sensors_event_t *magData,  int channel);
status_t store_rotation_vector(sensors_event_t *rotation_vector, int channel);
status_t store_euler_angles(sensors_event_t *euler, int channel);
status_t store_multiple_euler_angles(sensors_event_t *euler, int *channels, int n_channels);

#endif