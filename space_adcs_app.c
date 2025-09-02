#include "cfe.h"
#include "space_adcs.h"

// Interface to C++ controllers
extern "C" {
    void* create_microcontroller();
    void process_sensor_data(void* mc, float gyro[3], float altitude);
    void get_actuator_commands(void* mc, float torques[3]);
}

static void* microcontroller_instance = NULL;

int32 SpaceADCS_AppMain(void) {
    // Initialize microcontroller
    microcontroller_instance = create_microcontroller();
    
    // Main processing loop
    while (CFE_ES_RunLoop(&SpaceADCS_Data.RunStatus)) {
        // Process sensor messages
        SpaceADCS_ProcessData();
        OS_TaskDelay(100); // 10 Hz
    }
    
    CFE_ES_ExitApp(SpaceADCS_Data.RunStatus);
}

void SpaceADCS_ProcessData(void) {
    CFE_SB_Msg_t *MsgPtr;
    
    // Receive sensor data
    int32 status = CFE_SB_RcvMsg(&MsgPtr, SpaceADCS_Data.SensorPipe, CFE_SB_POLL);
    if (status == CFE_SUCCESS) {
        SensorData_t *SensorMsg = (SensorData_t *)MsgPtr;
        
        float gyro_rates[3] = {SensorMsg->gyro_x, SensorMsg->gyro_y, SensorMsg->gyro_z};
        
        // Call C++ microcontroller
        process_sensor_data(microcontroller_instance, gyro_rates, SensorMsg->altitude);
        
        // Get actuator commands
        float torques[3];
        get_actuator_commands(microcontroller_instance, torques);
        
        // Send actuator commands
        SpaceADCS_SendActuatorCommands(torques);
        
        SpaceADCS_Data.control_cycles++;
    }
}
