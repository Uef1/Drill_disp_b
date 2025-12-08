#pragma once
#include <Arduino.h>

// ПИ регулятор с conditional integration
class PIreg {
   public:
    float Kp = 0;
    float Ki = 0;
    float integral = 0;

    uint8_t compute(int input, int setpoint, float dt) {
        int err = setpoint - input;
        float out = err * Kp + integral;

        if ((out < 0 && err < 0) || (out >= 255 && err > 0)) {
            return constrain(out, 0, 255);
        }

        integral += err * Ki * dt;
        return out;
    }
};