#pragma once

#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    // Compute the output signal required from the current/actual value.
    float compute(float input) {

        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6f;
        prev_time = curr_time;

        error = setpoint - (input - zero_ref);

        // Prevent divide-by-zero on first iteration
        if (dt <= 0.0f) {
            return 0.0f;
        }

        // Integral term
        integral += error * dt;

        // Derivative term
        derivative = (error - prev_error) / dt;

        // PID output
        output = kp * error
            + ki * integral
            + kd * derivative;

        // Store error for next iteration
        prev_error = error;

        return output;
    }

    // Function used to return the last calculated error. 
    // The error is the difference between the desired position and current position. 
    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    float getError() {
      return error;
    }

    float updateHeading(){
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        uint32_t now = micros();

        float dt = (now - previousTime) / 
        previousTime = now;

        heading += g.gyro.z * dt;

    }

    // This must be called before trying to achieve a setpoint.
    // The first argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
        prev_time = micros();
        zero_ref = zero;
        setpoint = target;

        integral = 0;
        prev_error = 0;
    }

    

public:
    uint32_t prev_time, curr_time = micros();
    float dt;

private:
    float kp, ki, kd;
    float error, derivative, integral, output;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;
    Adafruit_MPU6050 mpu;
    float previousTime = 0;

};

}  // namespace mtrn3100
