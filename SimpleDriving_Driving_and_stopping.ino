
#include <Arduino.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <VL6180X.h>

#include <Motor.hpp>
#include <Encoder.hpp>
#include <PIDController.hpp>

const float WHEEL_DIAMETER = 32.0f; // 32mm
const uint16_t COUNTS_PER_REVOLUTION = 694;

//Hardware objects
mtrn3100::Motor leftMotor(11, 12);
mtrn3100::Motor rightMotor(9, 10);

// LIDAR Setup
VL6180X frontLidar;
const int enablePins[] = {A0, A1, A2}; // A0 = left, A1 = right, A2 = front
void setupLidar() {
    // Disable all sensors initially
    for (int i = 0; i < 3; i++) {
        pinMode(enablePins[i], OUTPUT);
        digitalWrite(enablePins[i], LOW);
    }
    delay(100);

    // Enable ONLY the front sensor on A2
    digitalWrite(enablePins[2], HIGH);
    delay(100);

    frontLidar.init();
    frontLidar.configureDefault();
    frontLidar.setTimeout(250);
}



// --- TASK CONTROLLER PARAMETERS ---
const float TARGET_DISTANCE_MM = 100.0f; // Exact target distance
const int MIN_PWM = 30;                  // Minimum PWM to overcome motor deadband

// Control Gains (tweak KP/KD if needed)
const float KP = 1.5f;                   
const float KD = 0.6f;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  setupLidar();

  Serial.println("Front LIDAR Initialized. Controller ready...");
  delay(1000);
}

void loop() {
  static float previousError = 0.0f;
    static uint32_t lastTime = millis();

    uint32_t currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0f;

    if (dt < 0.02f) { // 50 Hz control update rate
        return;
    }
    lastTime = currentTime;

    // Read current distance from front sensor
    uint8_t currentDistance = frontLidar.readRangeSingleMillimeters();

    if (frontLidar.timeoutOccurred()) {
        Serial.println("LIDAR Timeout!");
        leftMotor.setPWM(0);
        rightMotor.setPWM(0);
        return;
    }

    // Target error calculation
    float error = (float)currentDistance - TARGET_DISTANCE_MM;

    // Calculate PD Control Signal
    float derivative = (dt > 0) ? (error - previousError) / dt : 0.0f;
    previousError = error;

    float controlOutput = (KP * error) + (KD * derivative);

    int motorPWM = 0;

    // Deadband offset handling so small errors can still move the robot slightly
    if (abs(error) > 3.0f) { // Small 1mm threshold to prevent motor humming when exact
        if (controlOutput > 0) {
            motorPWM = (int)controlOutput;
        } else {
            motorPWM = (int)controlOutput;
        }
        motorPWM = constrain(motorPWM, -255, 255);
    } else {
        motorPWM = 0;
    }

    /*
     * Motor directions follow your mtrn3100 setup:
     * - Forward = Negative Left PWM, Positive Right PWM
     * - Backward = Positive Left PWM, Negative Right PWM
     */
    leftMotor.setPWM(-motorPWM);
    rightMotor.setPWM(motorPWM);

    // Diagnostics
    Serial.print("Dist: ");
    Serial.print(currentDistance);
    Serial.print(" mm | Err: ");
    Serial.print(error);
    Serial.print(" | PWM: ");
    Serial.println(motorPWM);

}
