#include <random>

class Microcontroller {
public:
    double readGyro() {
        return base_rate_ + noise();  // Simulate gyro drift
    }

    void applyTorque(double torque) {
        last_torque_ = torque;
    }

    double getLastTorque() const {
        return last_torque_;
    }

private:
    double base_rate_ = 0.0;
    double last_torque_ = 0.0;

    double noise() {
        static std::default_random_engine gen;
        static std::normal_distribution<double> dist(0.0, 0.01);
        return dist(gen);
    }
};
