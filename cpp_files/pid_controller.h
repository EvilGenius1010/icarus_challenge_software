#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    float compute(float setpoint, float measured_value, float dt);
    void reset();
    void setLimits(float min_output, float max_output);
    void setIntegralLimits(float min_integral, float max_integral);

private:
    float kp_;
    float ki_;
    float kd_;
    float integral_;
    float previous_error_;
    float min_output_;
    float max_output_;
    float min_integral_;
    float max_integral_;
};

#endif // PID_CONTROLLER_H
