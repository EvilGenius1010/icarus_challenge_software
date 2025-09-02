#ifndef SPACE_ADCS_APP_H
#define SPACE_ADCS_APP_H

#include "cfe.h"

typedef struct {
    float gyro[3];
    float magnetometer[3];
    float sun_angle;
    uint32_t timestamp;
    uint8_t valid;
} SensorData_t;

typedef struct {
    float wheel_torques[3];
    float magnetorquer[3];
    uint32_t timestamp;
} ActuatorCommands_t;

void SpaceADCS_AppMain(void);

#endif // SPACE_ADCS_APP_H
