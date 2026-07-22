
#include <Arduino.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <VL6180X.h>

const float WHEEL_DIAMETER = 32.0f; // 32mm
const uint16_t COUNTS_PER_REVOLUTION = 694;

namespace mtrn3100 {


/*
 * Motor class
 *
 * Provides a simple interface for controlling one DC motor.
 * The motor driver requires:
 *  - One PWM pin to control speed
 *  - One direction pin to control rotation direction
 */

class Motor {
public:

     /*
     * Constructor
     *
     * pwmPin       Pin connected to the motor driver's PWM input
     * directionPin Pin connected to the motor driver's direction input
     */

    Motor(uint8_t pwmPin, uint8_t directionPin) :  pwm_pin(pwmPin), dir_pin(directionPin) {
        pinMode(pwm_pin,OUTPUT); // driver
        pinMode(dir_pin,OUTPUT); // direction
    }

     /*
     * Sets the motor speed and direction.
     *
     * Positive PWM values produce one direction.
     * Negative PWM values produce the opposite direction.
     * The absolute value determines motor speed.
     *
     * The PWM magnitude is limited to the Arduino range of 0–255.
     */

    void setPWM(int16_t pwm) {

      // Select motor direction based on the sign of the command
      if (pwm > 0) {
        digitalWrite(dir_pin,HIGH);
      } else {
        digitalWrite(dir_pin,LOW);
      }
      
      // Convert negative PWM values into a positive magnitude
      int16_t pwmMagnitude = abs(pwm);

      // Ensure the PWM command remains within the valid Arduino range
      pwmMagnitude = constrain(pwmMagnitude, 0, 255);

      // Output the PWM signal to control motor speed
      analogWrite(pwm_pin, pwmMagnitude);
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}  // namespace mtrn3100

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
const float KP = 1.8f;                   
const float KD = 0.15f;

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
    if (abs(error) > 1.0f) { // Small 1mm threshold to prevent motor humming when exact
        if (controlOutput > 0) {
            motorPWM = MIN_PWM + (int)controlOutput;
        } else {
            motorPWM = -MIN_PWM + (int)controlOutput;
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
