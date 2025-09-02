#include "cfe.h"
#include "space_obc.h"
#include <string.h>
#include <math.h>

// Space-rated OBC Data Structure
typedef struct {
    CFE_SB_PipeId_t CommandPipe;
    CFE_SB_PipeId_t SensorPipe;
    
    // Sensor readings
    float gyro_rates[3];        // rad/s [x,y,z]
    float magnetometer[3];      // gauss [x,y,z]
    float sun_angle;            // degrees
    uint32 sensor_timestamp;
    
    // Actuator outputs
    float wheel_torques[3];     // Nm [x,y,z]
    float magnetorquer[3];      // Am² [x,y,z]
    
    // OBC state
    uint8 adcs_mode;            // 0=Safe, 1=Detumble, 2=Point, 3=Science
    uint8 fault_flags;          // Fault detection bits
    uint8 power_mode;           // 0=Eclipse, 1=Daylight, 2=Peak
    uint32 control_cycles;
    uint32 uptime_seconds;
    
    // Control parameters (space-rated defaults)
    float kp_detumble;          // Proportional gain for detumble
    float kp_point;             // Proportional gain for pointing
    float fault_threshold;      // Fault detection threshold
    float max_torque;           // Maximum allowed torque
    
    // Health monitoring
    float cpu_temperature;      // °C
    float voltage_3v3;          // V
    float voltage_5v;           // V
    uint32 memory_usage;        // bytes
    uint32 fault_count;
    uint32 reset_count;
    
    // Mission parameters
    float target_pointing[3];   // Target vector in body frame
    uint32 mission_elapsed_time;
    
} SpaceOBC_Data_t;

SpaceOBC_Data_t SpaceOBC_Data;

// Message definitions
#define SENSOR_DATA_MID         0x0800
#define ACTUATOR_CMD_MID        0x0801
#define OBC_HK_MID             0x0890
#define OBC_CMD_MID            0x1890

// Commands
#define OBC_NOOP_CC            0
#define OBC_RESET_COUNTERS_CC  1
#define OBC_SET_ADCS_MODE_CC   2
#define OBC_SET_GAINS_CC       3
#define OBC_SAFE_MODE_CC       4

// Sensor data from 42/bridge
typedef struct {
    uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
    float gyro_x, gyro_y, gyro_z;      // rad/s
    float mag_x, mag_y, mag_z;         // gauss
    float sun_angle;                   // degrees
    uint32 timestamp;
    uint8 sensor_valid;
} SensorData_t;

// Actuator commands to 42/bridge
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    float wheel_x, wheel_y, wheel_z;   // Nm
    float mtq_x, mtq_y, mtq_z;         // Am²
    uint32 timestamp;
} ActuatorCmd_t;

// Housekeeping telemetry
typedef struct {
    uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
    
    // Sensor data
    float gyro_rates[3];
    float magnetometer[3];
    float sun_angle;
    
    // Actuator outputs
    float wheel_torques[3];
    float magnetorquer[3];
    
    // OBC state
    uint8 adcs_mode;
    uint8 fault_flags;
    uint8 power_mode;
    uint32 control_cycles;
    uint32 uptime_seconds;
    
    // Health data
    float cpu_temperature;
    float voltage_3v3;
    float voltage_5v;
    uint32 memory_usage;
    uint32 fault_count;
    uint32 reset_count;
    
    // Mission data
    float target_pointing[3];
    uint32 mission_elapsed_time;
    
} OBC_HK_t;

// Command structure
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
} OBC_NoArgsCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint8 Mode;
} OBC_SetModeCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    float Kp_Detumble;
    float Kp_Point;
    float FaultThreshold;
    float MaxTorque;
} OBC_SetGainsCmd_t;

// App initialization
int32 SpaceOBC_AppInit(void) {
    // Create pipes
    CFE_SB_CreatePipe(&SpaceOBC_Data.CommandPipe, 16, "OBC_CMD");
    CFE_SB_CreatePipe(&SpaceOBC_Data.SensorPipe, 16, "OBC_SENSOR");
    
    // Subscribe to messages
    CFE_SB_Subscribe(OBC_CMD_MID, SpaceOBC_Data.CommandPipe);
    CFE_SB_Subscribe(SENSOR_DATA_MID, SpaceOBC_Data.SensorPipe);
    
    // Initialize space-rated defaults
    SpaceOBC_Data.kp_detumble = 0.1;     // Conservative for stability
    SpaceOBC_Data.kp_point = 0.05;       // Precise pointing
    SpaceOBC_Data.fault_threshold = 0.5;  // rad/s
    SpaceOBC_Data.max_torque = 0.1;      // Nm (safe limit)
    
    SpaceOBC_Data.adcs_mode = 0;         // Start in safe mode
    SpaceOBC_Data.power_mode = 0;        // Assume eclipse initially
    
    // Initialize health monitoring
    SpaceOBC_Data.cpu_temperature = 25.0;
    SpaceOBC_Data.voltage_3v3 = 3.3;
    SpaceOBC_Data.voltage_5v = 5.0;
    SpaceOBC_Data.memory_usage = 65536;  // 64KB baseline
    
    // Initialize target pointing (nadir)
    SpaceOBC_Data.target_pointing[0] = 0.0;
    SpaceOBC_Data.target_pointing[1] = 0.0;
    SpaceOBC_Data.target_pointing[2] = -1.0;
    
    CFE_EVS_SendEvent(1, CFE_EVS_INFORMATION, "Space OBC: Initialized - Safe Mode");
    return CFE_SUCCESS;
}

// Main processing function
void SpaceOBC_ProcessData(void) {
    CFE_SB_Msg_t *MsgPtr;
    CFE_SB_MsgId_t MsgId;
    
    // Process commands
    int32 status = CFE_SB_RcvMsg(&MsgPtr, SpaceOBC_Data.CommandPipe, CFE_SB_POLL);
    if (status == CFE_SUCCESS) {
        MsgId = CFE_SB_GetMsgId(MsgPtr);
        if (MsgId == OBC_CMD_MID) {
            SpaceOBC_ProcessCommand(MsgPtr);
        }
    }
    
    // Process sensor data
    status = CFE_SB_RcvMsg(&MsgPtr, SpaceOBC_Data.SensorPipe, CFE_SB_POLL);
    if (status == CFE_SUCCESS) {
        MsgId = CFE_SB_GetMsgId(MsgPtr);
        if (MsgId == SENSOR_DATA_MID) {
            SensorData_t *SensorMsg = (SensorData_t *)MsgPtr;
            
            // Update sensor readings
            SpaceOBC_Data.gyro_rates[0] = SensorMsg->gyro_x;
            SpaceOBC_Data.gyro_rates[1] = SensorMsg->gyro_y;
            SpaceOBC_Data.gyro_rates[2] = SensorMsg->gyro_z;
            
            SpaceOBC_Data.magnetometer[0] = SensorMsg->mag_x;
            SpaceOBC_Data.magnetometer[1] = SensorMsg->mag_y;
            SpaceOBC_Data.magnetometer[2] = SensorMsg->mag_z;
            
            SpaceOBC_Data.sun_angle = SensorMsg->sun_angle;
            SpaceOBC_Data.sensor_timestamp = SensorMsg->timestamp;
            
            // Run control algorithm
            SpaceOBC_RunControl();
            
            // Send actuator commands
            SpaceOBC_SendActuatorCommands();
            
            SpaceOBC_Data.control_cycles++;
        }
    }
    
    // Update health monitoring
    SpaceOBC_UpdateHealth();
    
    // Send housekeeping every 10 cycles
    if (SpaceOBC_Data.control_cycles % 10 == 0) {
        SpaceOBC_SendHousekeeping();
    }
}

// Space-rated control algorithm
void SpaceOBC_RunControl(void) {
    // Fault detection first (critical for space operations)
    SpaceOBC_FaultDetection();
    
    // Mode-specific control
    switch (SpaceOBC_Data.adcs_mode) {
        case 0: // Safe Mode
            // Zero all actuators
            memset(SpaceOBC_Data.wheel_torques, 0, sizeof(SpaceOBC_Data.wheel_torques));
            memset(SpaceOBC_Data.magnetorquer, 0, sizeof(SpaceOBC_Data.magnetorquer));
            break;
            
        case 1: // Detumble Mode
            SpaceOBC_DetumbleControl();
            break;
            
        case 2: // Pointing Mode
            SpaceOBC_PointingControl();
            break;
            
        case 3: // Science Mode
            SpaceOBC_ScienceControl();
            break;
            
        default:
            // Invalid mode - enter safe
            SpaceOBC_Data.adcs_mode = 0;
            CFE_EVS_SendEvent(2, CFE_EVS_ERROR, "Space OBC: Invalid mode, entering safe");
            break;
    }
    
    // Apply torque limits (space-rated safety)
    for (int i = 0; i < 3; i++) {
        if (SpaceOBC_Data.wheel_torques[i] > SpaceOBC_Data.max_torque) {
            SpaceOBC_Data.wheel_torques[i] = SpaceOBC_Data.max_torque;
        }
        if (SpaceOBC_Data.wheel_torques[i] < -SpaceOBC_Data.max_torque) {
            SpaceOBC_Data.wheel_torques[i] = -SpaceOBC_Data.max_torque;
        }
    }
}

// Detumble control (B-dot algorithm)
void SpaceOBC_DetumbleControl(void) {
    float k_bdot = SpaceOBC_Data.kp_detumble;
    
    for (int i = 0; i < 3; i++) {
        // B-dot control: MTQ = -k * (w x B)
        // Simplified: MTQ proportional to negative angular rate
        SpaceOBC_Data.magnetorquer[i] = -k_bdot * SpaceOBC_Data.gyro_rates[i] * 
                                        fabs(SpaceOBC_Data.magnetometer[i]);
        
        // Limit magnetorquer output
        if (SpaceOBC_Data.magnetorquer[i] > 1.0) SpaceOBC_Data.magnetorquer[i] = 1.0;
        if (SpaceOBC_Data.magnetorquer[i] < -1.0) SpaceOBC_Data.magnetorquer[i] = -1.0;
    }
}

// Pointing control (PD controller)
void SpaceOBC_PointingControl(void) {
    float kp = SpaceOBC_Data.kp_point;
    
    for (int i = 0; i < 3; i++) {
        // Simple rate control (assume target rates = 0)
        SpaceOBC_Data.wheel_torques[i] = -kp * SpaceOBC_Data.gyro_rates[i];
    }
}

// Science control (precision pointing)
void SpaceOBC_ScienceControl(void) {
    // Use same as pointing but with tighter gains
    float kp = SpaceOBC_Data.kp_point * 0.5; // More precise
    
    for (int i = 0; i < 3; i++) {
        SpaceOBC_Data.wheel_torques[i] = -kp * SpaceOBC_Data.gyro_rates[i];
    }
}

// Space-rated fault detection
void SpaceOBC_FaultDetection(void) {
    SpaceOBC_Data.fault_flags = 0;
    
    // Check angular rates
    for (int i = 0; i < 3; i++) {
        if (fabs(SpaceOBC_Data.gyro_rates[i]) > SpaceOBC_Data.fault_threshold) {
            SpaceOBC_Data.fault_flags |= (1 << i);
        }
    }
    
    // Check health parameters
    if (SpaceOBC_Data.cpu_temperature > 85.0 || SpaceOBC_Data.cpu_temperature < -40.0) {
        SpaceOBC_Data.fault_flags |= 0x08; // Temperature fault
    }
    
    if (SpaceOBC_Data.voltage_3v3 < 3.0 || SpaceOBC_Data.voltage_5v < 4.5) {
        SpaceOBC_Data.fault_flags |= 0x10; // Power fault
    }
    
    // Autonomous safe mode entry
    if (SpaceOBC_Data.fault_flags != 0) {
        SpaceOBC_Data.fault_count++;
        
        if (SpaceOBC_Data.fault_count > 3) {
            SpaceOBC_Data.adcs_mode = 0; // Enter safe mode
            CFE_EVS_SendEvent(3, CFE_EVS_CRITICAL, 
                             "Space OBC: Fault detected, entering safe mode (flags=0x%02X)", 
                             SpaceOBC_Data.fault_flags);
        }
    } else {
        SpaceOBC_Data.fault_count = 0; // Reset counter if no faults
    }
}

// Update health monitoring
void SpaceOBC_UpdateHealth(void) {
    // Simulate realistic health parameters
    static uint32 health_cycle = 0;
    health_cycle++;
    
    // CPU temperature varies with power mode
    if (SpaceOBC_Data.power_mode == 0) { // Eclipse
        SpaceOBC_Data.cpu_temperature = 15.0 + (health_cycle % 10) * 0.5;
    } else { // Daylight
        SpaceOBC_Data.cpu_temperature = 45.0 + (health_cycle % 20) * 0.2;
    }
    
    // Voltage monitoring
    SpaceOBC_Data.voltage_3v3 = 3.3 + ((health_cycle % 100) - 50) * 0.001;
    SpaceOBC_Data.voltage_5v = 5.0 + ((health_cycle % 80) - 40) * 0.002;
    
    // Memory usage varies with operations
    SpaceOBC_Data.memory_usage = 65536 + (SpaceOBC_Data.control_cycles % 1000) * 10;
    
    // Update uptime
    if (health_cycle % 10 == 0) { // Every 10 cycles = ~1 second
        SpaceOBC_Data.uptime_seconds++;
        SpaceOBC_Data.mission_elapsed_time++;
    }
}

// Process commands
void SpaceOBC_ProcessCommand(CFE_SB_Msg_t *MsgPtr) {
    uint16 CommandCode = CFE_SB_GetCmdCode(MsgPtr);
    
    switch (CommandCode) {
        case OBC_NOOP_CC:
            CFE_EVS_SendEvent(4, CFE_EVS_INFORMATION, "Space OBC: No-op command received");
            break;
            
        case OBC_RESET_COUNTERS_CC:
            SpaceOBC_Data.control_cycles = 0;
            SpaceOBC_Data.fault_count = 0;
            CFE_EVS_SendEvent(5, CFE_EVS_INFORMATION, "Space OBC: Counters reset");
            break;
            
        case OBC_SET_ADCS_MODE_CC: {
            OBC_SetModeCmd_t *SetModeCmd = (OBC_SetModeCmd_t *)MsgPtr;
            if (SetModeCmd->Mode <= 3) {
                SpaceOBC_Data.adcs_mode = SetModeCmd->Mode;
                CFE_EVS_SendEvent(6, CFE_EVS_INFORMATION, 
                                 "Space OBC: ADCS mode set to %d", SetModeCmd->Mode);
            } else {
                CFE_EVS_SendEvent(7, CFE_EVS_ERROR, 
                                 "Space OBC: Invalid ADCS mode %d", SetModeCmd->Mode);
            }
            break;
        }
        
        case OBC_SET_GAINS_CC: {
            OBC_SetGainsCmd_t *SetGainsCmd = (OBC_SetGainsCmd_t *)MsgPtr;
            SpaceOBC_Data.kp_detumble = SetGainsCmd->Kp_Detumble;
            SpaceOBC_Data.kp_point = SetGainsCmd->Kp_Point;
            SpaceOBC_Data.fault_threshold = SetGainsCmd->FaultThreshold;
            SpaceOBC_Data.max_torque = SetGainsCmd->MaxTorque;
            CFE_EVS_SendEvent(8, CFE_EVS_INFORMATION, "Space OBC: Control gains updated");
            break;
        }
        
        case OBC_SAFE_MODE_CC:
            SpaceOBC_Data.adcs_mode = 0;
            SpaceOBC_Data.fault_count = 0;
            CFE_EVS_SendEvent(9, CFE_EVS_INFORMATION, "Space OBC: Safe mode commanded");
            break;
            
        default:
            CFE_EVS_SendEvent(10, CFE_EVS_ERROR, 
                             "Space OBC: Invalid command code %d", CommandCode);
            break;
    }
}

// Send actuator commands
void SpaceOBC_SendActuatorCommands(void) {
    ActuatorCmd_t ActuatorMsg;
    
    CFE_SB_InitMsg(&ActuatorMsg, ACTUATOR_CMD_MID, sizeof(ActuatorCmd_t), TRUE);
    
    ActuatorMsg.wheel_x = SpaceOBC_Data.wheel_torques[0];
    ActuatorMsg.wheel_y = SpaceOBC_Data.wheel_torques[1];
    ActuatorMsg.wheel_z = SpaceOBC_Data.wheel_torques[2];
    
    ActuatorMsg.mtq_x = SpaceOBC_Data.magnetorquer[0];
    ActuatorMsg.mtq_y = SpaceOBC_Data.magnetorquer[1];
    ActuatorMsg.mtq_z = SpaceOBC_Data.magnetorquer[2];
    
    ActuatorMsg.timestamp = CFE_TIME_GetTime().Seconds;
    
    CFE_SB_SendMsg((CFE_SB_Msg_t *)&ActuatorMsg);
}

// Send housekeeping telemetry
void SpaceOBC_SendHousekeeping(void) {
    OBC_HK_t HkMsg;
    
    CFE_SB_InitMsg(&HkMsg, OBC_HK_MID, sizeof(OBC_HK_t), TRUE);
    
    // Copy all data to housekeeping message
    memcpy(HkMsg.gyro_rates, SpaceOBC_Data.gyro_rates, sizeof(HkMsg.gyro_rates));
    memcpy(HkMsg.magnetometer, SpaceOBC_Data.magnetometer, sizeof(HkMsg.magnetometer));
    HkMsg.sun_angle = SpaceOBC_Data.sun_angle;
    
    memcpy(HkMsg.wheel_torques, SpaceOBC_Data.wheel_torques, sizeof(HkMsg.wheel_torques));
    memcpy(HkMsg.magnetorquer, SpaceOBC_Data.magnetorquer, sizeof(HkMsg.magnetorquer));
    
    HkMsg.adcs_mode = SpaceOBC_Data.adcs_mode;
    HkMsg.fault_flags = SpaceOBC_Data.fault_flags;
    HkMsg.power_mode = SpaceOBC_Data.power_mode;
    HkMsg.control_cycles = SpaceOBC_Data.control_cycles;
    HkMsg.uptime_seconds = SpaceOBC_Data.uptime_seconds;
    
    HkMsg.cpu_temperature = SpaceOBC_Data.cpu_temperature;
    HkMsg.voltage_3v3 = SpaceOBC_Data.voltage_3v3;
    HkMsg.voltage_5v = SpaceOBC_Data.voltage_5v;
    HkMsg.memory_usage = SpaceOBC_Data.memory_usage;
    HkMsg.fault_count = SpaceOBC_Data.fault_count;
    HkMsg.reset_count = SpaceOBC_Data.reset_count;
    
    memcpy(HkMsg.target_pointing, SpaceOBC_Data.target_pointing, sizeof(HkMsg.target_pointing));
    HkMsg.mission_elapsed_time = SpaceOBC_Data.mission_elapsed_time;
    
    CFE_SB_SendMsg((CFE_SB_Msg_t *)&HkMsg);
}
