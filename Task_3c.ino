#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
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

Adafruit_MPU6050 mpu;

float gyroBiasZ = 0.0f;
float yawDegrees = 0.0f;
uint32_t previousIMUTime = 0;

const float TARGET_TURN_DEGREES = -90.0f;
const float ANGLE_TOLERANCE_DEGREES = 3.0f;

const float KP = 2.8f;
const float KD = 0.18f;

const int MIN_TURN_PWM = 45;
const int MAX_TURN_PWM = 110;

const uint32_t CONTROL_PERIOD_US = 10000;
const uint32_t TASK_DURATION_MS = 30000;

void calibrateGyro();
void updateYaw();
void maintainTargetHeading(float targetHeading, uint32_t durationMS);
void stopMotors();

void setup() {
    Serial.begin(9600);
    Wire.begin();

    if (!mpu.begin()) {
        Serial.println("MPU6050 not detected");
        while (true) {
            stopMotors();
            delay(100);
        }
    }

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(500);
    calibrateGyro();

    Serial.println("Starting Task 3.3 in 2 seconds");
    delay(2000);

    float targetHeading = yawDegrees + TARGET_TURN_DEGREES;
    maintainTargetHeading(targetHeading, TASK_DURATION_MS);

    stopMotors();
    Serial.println("Task 3.3 finished");
}

void loop() {
}

void calibrateGyro() {
    const int SAMPLE_COUNT = 500;
    float total = 0.0f;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sensors_event_t acceleration;
        sensors_event_t gyro;
        sensors_event_t temperature;

        mpu.getEvent(&acceleration, &gyro, &temperature);
        total += gyro.gyro.z;
        delay(5);
    }

    gyroBiasZ = total / SAMPLE_COUNT;
    yawDegrees = 0.0f;
    previousIMUTime = micros();
}

void updateYaw() {
    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu.getEvent(&acceleration, &gyro, &temperature);

    uint32_t currentTime = micros();

    if (previousIMUTime == 0) {
        previousIMUTime = currentTime;
        return;
    }

    float dt =
        static_cast<float>(currentTime - previousIMUTime) /
        1000000.0f;

    previousIMUTime = currentTime;

    float yawRateDeg =
        (gyro.gyro.z - gyroBiasZ) * 180.0f / PI;

    if (fabs(yawRateDeg) < 0.2f) {
        yawRateDeg = 0.0f;
    }

    yawDegrees += yawRateDeg * dt;
}

void maintainTargetHeading(float targetHeading, uint32_t durationMS) {
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

        sensors_event_t acceleration;
        sensors_event_t gyro;
        sensors_event_t temperature;

        mpu.getEvent(&acceleration, &gyro, &temperature);

        float yawRateDeg =
            (gyro.gyro.z - gyroBiasZ) * 180.0f / PI;

        if (fabs(yawRateDeg) < 0.2f) {
            yawRateDeg = 0.0f;
        }

        yawDegrees += yawRateDeg * dt;

        float error = targetHeading - yawDegrees;

        if (fabs(error) <= ANGLE_TOLERANCE_DEGREES) {
            stopMotors();
        } else {
            float output = KP * error - KD * yawRateDeg;

            output = constrain(
                output,
                -static_cast<float>(MAX_TURN_PWM),
                static_cast<float>(MAX_TURN_PWM)
            );

            int turnPWM = static_cast<int>(roundf(output));

            if (turnPWM > 0 && turnPWM < MIN_TURN_PWM) {
                turnPWM = MIN_TURN_PWM;
            } else if (turnPWM < 0 && turnPWM > -MIN_TURN_PWM) {
                turnPWM = -MIN_TURN_PWM;
            }

            leftMotor.setPWM(turnPWM);
            rightMotor.setPWM(turnPWM);
        }

        if (millis() - previousPrintTime >= 100) {
            previousPrintTime = millis();

            Serial.print("Target: ");
            Serial.print(targetHeading);
            Serial.print(", yaw: ");
            Serial.print(yawDegrees);
            Serial.print(", error: ");
            Serial.println(error);
        }
    }

    stopMotors();
}

void stopMotors() {
    leftMotor.setPWM(0);
    rightMotor.setPWM(0);
}