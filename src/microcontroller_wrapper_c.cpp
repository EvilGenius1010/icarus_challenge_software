#include "microcontroller.h"
#include <cstring>

static Microcontroller* g_microcontroller = nullptr;

extern "C" {

// Initialize the microcontroller instance
void Microcontroller_Init() {
    if (!g_microcontroller) {
        g_microcontroller = new Microcontroller();
    }
}

// Sensor data struct matching cFS telemetry payload
typedef struct {
    float gyro[3];
    float magnetometer[3];
    float sun_angle;
    unsigned int timestamp;
    unsigned char valid;
} SensorData_t;

// Actuator command struct for output
typedef struct {
    float wheel_torques[3];
    float magnetorquer[3];
    unsigned int timestamp;
} ActuatorCommands_t;

// Process incoming sensor data (called from cFS app)
void Microcontroller_ProcessSensor(const SensorData_t* sensor) {
    if (g_microcontroller && sensor) {
        // Convert to Microcontroller::SensorData
        auto sensorData = Microcontroller::SensorData{
            {sensor->gyro[0], sensor->gyro[1], sensor->gyro[2]},
            {sensor->magnetometer[0], sensor->magnetometer[1], sensor->magnetometer[2]},
            sensor->sun_angle,
            sensor->timestamp,
            sensor->valid != 0
        };
        g_microcontroller->processSensorData(sensorData);
    }
}

// Get actuator commands from controller
void Microcontroller_GetActuatorCommands(ActuatorCommands_t* commands) {
    if (g_microcontroller && commands) {
        Microcontroller::ActuatorCommands actuators;
        g_microcontroller->getActuatorCommands(actuators);
        std::memcpy(commands->wheel_torques, actuators.wheel_torques, sizeof(actuators.wheel_torques));
        std::memcpy(commands->magnetorquer, actuators.magnetorquer, sizeof(actuators.magnetorquer));
        commands->timestamp = actuators.timestamp;
    }
}

// Cleanup microcontroller instance
void Microcontroller_Cleanup() {
    delete g_microcontroller;
    g_microcontroller = nullptr;
}

}
