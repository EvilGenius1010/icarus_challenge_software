#include "adcs_controller.h"
#include <cmath>
#include <cstring>
#include <algorithm>

ADCSController::ADCSController(float kp_detumble, float kp_point, float kd_point)
    : kp_detumble_(kp_detumble), kp_point_(kp_point), kd_point_(kd_point),
      max_torque_(0.1f), first_mag_reading_(true) {
    
    // FIXED: Now using array of pointers correctly
    for (int i = 0; i < 3; ++i) {
        pid_controllers_[i] = new PIDController(kp_point, 0.01f, kd_point);
        pid_controllers_[i]->setLimits(-max_torque_, max_torque_);
    }
    
    // FIXED: Using proper array with std::memset
    std::memset(previous_mag_, 0, sizeof(previous_mag_));
}

ADCSController::~ADCSController() {
    for (int i = 0; i < 3; ++i) {
        delete pid_controllers_[i];
    }
}

void ADCSController::computeControl(const float* gyro_rates, const float* magnetometer,
                                  float sun_angle, float* wheel_torques, 
                                  float* magnetorquer, int control_mode) {
    
    // FIXED: Using pointers for memset
    std::memset(wheel_torques, 0, 3 * sizeof(float));
    std::memset(magnetorquer, 0, 3 * sizeof(float));
    
    switch (control_mode) {
        case 0: // Safe Mode
            safeMode(wheel_torques, magnetorquer);
            break;
        case 1: // Detumble Mode
            detumbleControl(gyro_rates, magnetometer, magnetorquer);
            break;
        case 2: // Pointing Mode
            pointingControl(gyro_rates, wheel_torques);
            break;
        case 3: // Science Mode
            scienceControl(gyro_rates, wheel_torques);
            break;
        default:
            safeMode(wheel_torques, magnetorquer);
            break;
    }
}

void ADCSController::detumbleControl(const float* gyro_rates, const float* magnetometer,
                                   float* magnetorquer) {
    if (first_mag_reading_) {
        // FIXED: Using std::memcpy with proper size
        std::memcpy(previous_mag_, magnetometer, 3 * sizeof(float));
        first_mag_reading_ = false;
        return;
    }
    
    float dt = 0.1f;  // 10 Hz control rate
    
    for (int i = 0; i < 3; ++i) {
        // Compute magnetic field derivative
        float b_dot = (magnetometer[i] - previous_mag_[i]) / dt;
        
        // B-dot control law
        magnetorquer[i] = -kp_detumble_ * b_dot;
        
        // FIXED: Using array indexing properly
        magnetorquer[i] = std::max(-1.0f, std::min(1.0f, magnetorquer[i]));
        
        previous_mag_[i] = magnetometer[i];
    }
}

void ADCSController::pointingControl(const float* gyro_rates, float* wheel_torques) {
    float dt = 0.1f;  // 10 Hz control rate
    
    for (int i = 0; i < 3; ++i) {
        // FIXED: Using array indexing properly
        wheel_torques[i] = pid_controllers_[i]->compute(0.0f, gyro_rates[i], dt);
    }
}

void ADCSController::scienceControl(const float* gyro_rates, float* wheel_torques) {
    // Use more precise control for science operations
    float dt = 0.1f;
    
    for (int i = 0; i < 3; ++i) {
        // Tighter control with same target (zero rates)
        float torque = pid_controllers_[i]->compute(0.0f, gyro_rates[i], dt);
        // Scale down for more precise control
        wheel_torques[i] = torque * 0.5f;
        
        // FIXED: Using array indexing properly
        wheel_torques[i] = std::max(-0.05f, std::min(0.05f, wheel_torques[i]));
    }
}

void ADCSController::safeMode(float* wheel_torques, float* magnetorquer) {
    // FIXED: Using pointers for memset
    std::memset(wheel_torques, 0, 3 * sizeof(float));
    std::memset(magnetorquer, 0, 3 * sizeof(float));
}

void ADCSController::setGains(float kp_detumble, float kp_point, float kd_point) {
    kp_detumble_ = kp_detumble;
    kp_point_ = kp_point;
    kd_point_ = kd_point;
    
    // Update PID controllers
    for (int i = 0; i < 3; ++i) {
        delete pid_controllers_[i];
        pid_controllers_[i] = new PIDController(kp_point, 0.01f, kd_point);
        pid_controllers_[i]->setLimits(-max_torque_, max_torque_);
    }
}

void ADCSController::setMaxTorque(float max_torque) {
    max_torque_ = max_torque;
    
    for (int i = 0; i < 3; ++i) {
        pid_controllers_[i]->setLimits(-max_torque_, max_torque_);
    }
}
