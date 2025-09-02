#include "cfe.h"
#include "space_adcs_app.h"
#include <string.h>

// External C wrapper calls to C++ microcontroller
extern void Microcontroller_Init(void);
extern void Microcontroller_ProcessSensor(const SensorData_t* sensor);
extern void Microcontroller_GetActuatorCommands(ActuatorCommands_t* commands);

// Global app data
typedef struct {
    CFE_SB_PipeId_t SensorPipe;
    uint32 RunStatus;
} SpaceADCS_Data_t;

SpaceADCS_Data_t SpaceADCS_Data;

void SpaceADCS_AppMain(void) {
    CFE_Status_t Status;
    
    // Send startup message
    CFE_EVS_SendEvent(1, CFE_EVS_EventType_INFORMATION, 
                     "=== SPACE ADCS APP STARTING ===");
    
    // Initialize EVS
    Status = CFE_EVS_Register(NULL, 0, CFE_EVS_EventFilter_BINARY);
    if (Status != CFE_SUCCESS) {
        CFE_ES_WriteToSysLog("Space ADCS: Error registering EVS, RC = 0x%08lX\n", 
                            (unsigned long)Status);
        return;
    }
    
    CFE_EVS_SendEvent(2, CFE_EVS_EventType_INFORMATION, 
                     "SPACE ADCS: EVS Registration Complete");
    
    // Create SB pipe
    Status = CFE_SB_CreatePipe(&SpaceADCS_Data.SensorPipe, 16, "ADCS_SENSOR_PIPE");
    if (Status != CFE_SUCCESS) {
        CFE_EVS_SendEvent(3, CFE_EVS_EventType_ERROR, 
                         "SPACE ADCS: Error creating pipe, RC = 0x%08lX", 
                         (unsigned long)Status);
        return;
    }
    
    CFE_EVS_SendEvent(4, CFE_EVS_EventType_INFORMATION, 
                     "SPACE ADCS: Software Bus Pipe Created Successfully");
    
    // Subscribe to sensor data
    CFE_SB_MsgId_t SensorMsgId = CFE_SB_ValueToMsgId(0x0800);
    Status = CFE_SB_Subscribe(SensorMsgId, SpaceADCS_Data.SensorPipe);
    if (Status != CFE_SUCCESS) {
        CFE_EVS_SendEvent(5, CFE_EVS_EventType_ERROR, 
                         "SPACE ADCS: Error subscribing to sensor MsgId 0x0800, RC = 0x%08lX", 
                         (unsigned long)Status);
        return;
    }
    
    CFE_EVS_SendEvent(6, CFE_EVS_EventType_INFORMATION, 
                     "SPACE ADCS: Subscribed to Sensor Data MsgId 0x0800");

    // Initialize C++ microcontroller
    Microcontroller_Init();
    CFE_EVS_SendEvent(7, CFE_EVS_EventType_INFORMATION, 
                     "SPACE ADCS: Microcontroller & PID Controller Initialized");

    SpaceADCS_Data.RunStatus = CFE_ES_RunStatus_APP_RUN;

    // Send ready message
    CFE_EVS_SendEvent(8, CFE_EVS_EventType_INFORMATION, 
                     "=== SPACE ADCS APP READY - WAITING FOR SENSOR DATA ===");

    // Main processing loop with enhanced logging
    uint32 total_messages = 0;
    while (CFE_ES_RunLoop(&SpaceADCS_Data.RunStatus)) {
        CFE_SB_Buffer_t* SBBufPtr = NULL;
        
        Status = CFE_SB_ReceiveBuffer(&SBBufPtr, SpaceADCS_Data.SensorPipe, CFE_SB_PEND_FOREVER);
        if (Status == CFE_SUCCESS && SBBufPtr != NULL) {
            total_messages++;
            
            // Process sensor data message
            SensorData_t SensorMsg;
            CFE_MSG_Message_t* MsgPtr = &SBBufPtr->Msg;
            void* DataPtr = CFE_SB_GetUserData(MsgPtr);
            
            if (DataPtr != NULL) {
                memcpy(&SensorMsg, DataPtr, sizeof(SensorData_t));
                
                // Call C++ microcontroller processing
                Microcontroller_ProcessSensor(&SensorMsg);
                
                // Get actuator commands
                ActuatorCommands_t ActCmds;
                Microcontroller_GetActuatorCommands(&ActCmds);
                
                // Log every 10 messages for first 100, then every 50
                if ((total_messages <= 100 && total_messages % 10 == 0) || 
                    (total_messages % 50 == 0)) {
                    CFE_EVS_SendEvent(9, CFE_EVS_EventType_INFORMATION, 
                                    "SPACE ADCS: Processed %lu sensor msgs - "
                                    "Gyro:[%.3f,%.3f,%.3f] Torques:[%.3f,%.3f,%.3f]", 
                                    (unsigned long)total_messages,
                                    SensorMsg.gyro[0], SensorMsg.gyro[1], SensorMsg.gyro[2],
                                    ActCmds.wheel_torques[0], ActCmds.wheel_torques[1], ActCmds.wheel_torques[2]);
                }
            }
        }
    }
    
    CFE_EVS_SendEvent(10, CFE_EVS_EventType_INFORMATION, 
                     "=== SPACE ADCS APP SHUTTING DOWN ===");
    CFE_ES_ExitApp(SpaceADCS_Data.RunStatus);
}
