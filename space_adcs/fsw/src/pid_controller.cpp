#include "pid_controller.h"
#include <algorithm>

PIDController::PIDController(float kp, float ki, float kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0f), previous_error_(0.0f),
      min_output_(-1.0f), max_output_(1.0f), min_integral_(-10.0f), max_integral_(10.0f) {}

float PIDController::compute(float setpoint, float measured_value, float dt) {
    if (dt <= 0.0f) return 0.0f;  // Prevent division by zero
    
    float error = setpoint - measured_value;
    
    // Proportional term
    float P = kp_ * error;
    
    // Integral term with anti-windup
    integral_ += error * dt;
    integral_ = std::max(min_integral_, std::min(max_integral_, integral_));
    float I = ki_ * integral_;
    
    // Derivative term
    float D = kd_ * (error - previous_error_) / dt;
    previous_error_ = error;
    
    // PID output with limits
    float output = P + I + D;
    return std::max(min_output_, std::min(max_output_, output));
}

void PIDController::reset() {
    integral_ = 0.0f;
    previous_error_ = 0.0f;
}

void PIDController::setLimits(float min_output, float max_output) {
    min_output_ = min_output;
    max_output_ = max_output;
}

void PIDController::setIntegralLimits(float min_integral, float max_integral) {
    min_integral_ = min_integral;
    max_integral_ = max_integral;
}
