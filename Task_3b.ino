#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <math.h>

namespace mtrn3100 {

class Motor {
public:
    Motor(uint8_t pwmPin, uint8_t directionPin)
        : pwm_pin(pwmPin), dir_pin(directionPin) {
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
    }

    void setPWM(int16_t pwm) {
        pwm = constrain(pwm, -255, 255);

        if (pwm == 0) {
            analogWrite(pwm_pin, 0);
            return;
        }

        digitalWrite(dir_pin, pwm > 0 ? HIGH : LOW);
        analogWrite(pwm_pin, abs(pwm));
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}

mtrn3100::Motor leftMotor(11, 12);
mtrn3100::Motor rightMotor(9, 10);

Adafruit_VL53L0X frontLidar;

const float TARGET_DISTANCE_MM = 100.0f;
const float DISTANCE_TOLERANCE_MM = 5.0f;

const float KP = 0.80f;
const float KI = 0.00f;
const float KD = 0.05f;

const int MIN_WALL_PWM = 35;
const int MAX_WALL_PWM = 80;

const uint32_t CONTROL_PERIOD_US = 10000;
const uint32_t TASK_DURATION_MS = 35000;

float readFrontDistanceMM();
void maintainFrontDistance(float targetDistanceMM, uint32_t durationMS);
void stopMotors();

void setup() {
    Serial.begin(9600);
    Wire.begin();

    if (!frontLidar.begin()) {
        Serial.println("VL53L0X not detected");
        while (true) {
            stopMotors();
            delay(100);
        }
    }

    Serial.println("Starting Task 3.2 in 2 seconds");
    delay(2000);

    maintainFrontDistance(TARGET_DISTANCE_MM, TASK_DURATION_MS);

    stopMotors();
    Serial.println("Task 3.2 finished");
}

void loop() {
}

float readFrontDistanceMM() {
    VL53L0X_RangingMeasurementData_t measurement;
    frontLidar.rangingTest(&measurement, false);

    if (measurement.RangeStatus == 4) {
        return NAN;
    }

    return static_cast<float>(measurement.RangeMilliMeter);
}

void maintainFrontDistance(float targetDistanceMM, uint32_t durationMS) {
    float integralError = 0.0f;
    float previousError = 0.0f;

    uint32_t previousControlTime = micros();
    uint32_t previousPrintTime = millis();
    uint32_t startTime = millis();

    while (millis() - startTime < durationMS) {
        uint32_t currentTime = micros();

        if (currentTime - previousControlTime < CONTROL_PERIOD_US) {
            continue;
        }

        float dt =
            static_cast<float>(currentTime - previousControlTime) /
            1000000.0f;

        previousControlTime = currentTime;

        float measuredDistanceMM = readFrontDistanceMM();

        if (!isfinite(measuredDistanceMM) ||
            measuredDistanceMM < 20.0f ||
            measuredDistanceMM > 1000.0f) {
            stopMotors();
            integralError = 0.0f;
            continue;
        }

        float error = measuredDistanceMM - targetDistanceMM;

        if (fabs(error) <= DISTANCE_TOLERANCE_MM) {
            stopMotors();
            integralError = 0.0f;
            previousError = error;
        } else {
            integralError += error * dt;
            integralError = constrain(integralError, -200.0f, 200.0f);

            float derivativeError = (error - previousError) / dt;
            previousError = error;

            float output =
                KP * error +
                KI * integralError +
                KD * derivativeError;

            output = constrain(
                output,
                -static_cast<float>(MAX_WALL_PWM),
                static_cast<float>(MAX_WALL_PWM)
            );

            int drivePWM = static_cast<int>(roundf(output));

            if (drivePWM > 0 && drivePWM < MIN_WALL_PWM) {
                drivePWM = MIN_WALL_PWM;
            } else if (drivePWM < 0 && drivePWM > -MIN_WALL_PWM) {
                drivePWM = -MIN_WALL_PWM;
            }

            leftMotor.setPWM(-drivePWM);
            rightMotor.setPWM(drivePWM);
        }

        if (millis() - previousPrintTime >= 100) {
            previousPrintTime = millis();

            Serial.print("Distance: ");
            Serial.print(measuredDistanceMM);
            Serial.print(" mm, error: ");
            Serial.println(error);
        }
    }

    stopMotors();
}

void stopMotors() {
    leftMotor.setPWM(0);
    rightMotor.setPWM(0);
}