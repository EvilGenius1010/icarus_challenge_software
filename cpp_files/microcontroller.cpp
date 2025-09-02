#include "microcontroller.h"
#include "adcs_controller.h"
#include <iostream>
#include <cmath>
#include <cstring>
#include <algorithm>

Microcontroller::Microcontroller()
    : control_mode_(0), fault_flags_(0), control_cycles_(0), uptime_seconds_(0),
      fault_threshold_(0.5f), max_torque_(0.1f), fault_count_(0), consecutive_faults_(0) {
    
    adcs_controller_ = new ADCSController(0.1f, 0.05f, 0.01f);
    
    std::memset(gyro_rates_, 0, sizeof(gyro_rates_));
    std::memset(magnetometer_, 0, sizeof(magnetometer_));
    std::memset(wheel_torques_, 0, sizeof(wheel_torques_));
    std::memset(magnetorquer_, 0, sizeof(magnetorquer_));
    
    sun_angle_ = 0.0f;
    
    std::cout << "Microcontroller initialized - Safe Mode" << std::endl;
}

Microcontroller::~Microcontroller() {
    delete adcs_controller_;
}

void Microcontroller::processSensorData(const SensorData& sensor_data) {
    if (!sensor_data.valid) {
        consecutive_faults_++;
        if (consecutive_faults_ > 5) {
            control_mode_ = 0;
        }
        return;
    }
    
    for (int i = 0; i < 3; ++i) {
        gyro_rates_[i] = sensor_data.gyro[i];
        magnetometer_[i] = sensor_data.magnetometer[i];
    }
    sun_angle_ = sensor_data.sun_angle;
    
    consecutive_faults_ = 0;
    
    performFaultDetection();
    
    // FIXED: Remove asterisks - pass arrays directly, not dereferenced
    adcs_controller_->computeControl(gyro_rates_, magnetometer_, sun_angle_, 
                                   wheel_torques_, magnetorquer_, control_mode_);
    
    for (int i = 0; i < 3; ++i) {
        wheel_torques_[i] = std::max(-max_torque_, std::min(max_torque_, wheel_torques_[i]));
        magnetorquer_[i] = std::max(-1.0f, std::min(1.0f, magnetorquer_[i]));
    }
    
    control_cycles_++;
    updateHealthMonitoring();
}

void Microcontroller::getActuatorCommands(ActuatorCommands& commands) {
    for (int i = 0; i < 3; ++i) {
        commands.wheel_torques[i] = wheel_torques_[i];
        commands.magnetorquer[i] = magnetorquer_[i];
    }
    commands.timestamp = control_cycles_;
}

void Microcontroller::setControlMode(int mode) {
    if (mode >= 0 && mode <= 3) {
        control_mode_ = mode;
        std::cout << "Control mode set to: " << mode << std::endl;
    }
}

void Microcontroller::setFaultThreshold(float threshold) {
    if (threshold > 0.0f) {
        fault_threshold_ = threshold;
    }
}

void Microcontroller::performFaultDetection() {
    fault_flags_ = 0;
    
    for (int i = 0; i < 3; ++i) {
        if (std::abs(gyro_rates_[i]) > fault_threshold_) {
            fault_flags_ |= (1 << i);
        }
    }
    
    bool sensor_fault = false;
    for (int i = 0; i < 3; ++i) {
        if (std::isnan(gyro_rates_[i]) || std::isinf(gyro_rates_[i]) ||
            std::isnan(magnetometer_[i]) || std::isinf(magnetometer_[i])) {
            sensor_fault = true;
            break;
        }
    }
    
    if (sensor_fault) {
        fault_flags_ |= 0x08;
    }
    
    if (fault_flags_ != 0) {
        fault_count_++;
        
        if (fault_count_ > 3 && control_mode_ != 0) {
            control_mode_ = 0;
            std::cout << "Multiple faults detected. Entering safe mode." << std::endl;
        }
    } else {
        fault_count_ = 0;
    }
}

void Microcontroller::updateHealthMonitoring() {
    if (control_cycles_ % 10 == 0) {
        uptime_seconds_++;
    }
}
