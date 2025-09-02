#include "pid_controller.cpp"
#include "microcontroller.cpp"
#include <iostream>

int main() {
    PID pid(0.5, 0.1, 0.05);
    Microcontroller mcu;

    double target_rate = 0.0;  // rad/s
    double dt = 0.1;           // time step (s)

    for (int i = 0; i < 100; ++i) {
        double measured_rate = mcu.readGyro();
        double torque = pid.compute(target_rate, measured_rate, dt);
        mcu.applyTorque(torque);

        std::cout << "Step " << i
                  << " | Measured: " << measured_rate
                  << " | Torque: " << torque << std::endl;
    }

    return 0;
}
