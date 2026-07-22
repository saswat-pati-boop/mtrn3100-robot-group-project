#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

const float WHEEL_DIAMETER = 32.0f;
const uint16_t COUNTS_PER_REVOLUTION = 694;
const float MAZE_CELL_MM = 180.0f;

const char COMMANDS[] = "LFRFFLFR";

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

class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2)
        : encoder1_pin(enc1), encoder2_pin(enc2) {
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        if (instance1 == nullptr) {
            instance1 = this;
            attachInterrupt(
                digitalPinToInterrupt(encoder1_pin),
                readEncoderISR1,
                RISING
            );
        } else if (instance2 == nullptr) {
            instance2 = this;
            attachInterrupt(
                digitalPinToInterrupt(encoder1_pin),
                readEncoderISR2,
                RISING
            );
        }
    }

    void readEncoder() {
        if (digitalRead(encoder2_pin) == HIGH) {
            count++;
        } else {
            count--;
        }
    }

    long getCount() {
        noInterrupts();
        long copy = count;
        interrupts();
        return copy;
    }

    float getRotation() {
        return
            static_cast<float>(getCount()) /
            static_cast<float>(COUNTS_PER_REVOLUTION) *
            2.0f * PI;
    }

    void reset() {
        noInterrupts();
        count = 0;
        interrupts();
    }

private:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile long count = 0;

    static Encoder* instance1;
    static Encoder* instance2;

    static void readEncoderISR1() {
        if (instance1 != nullptr) {
            instance1->readEncoder();
        }
    }

    static void readEncoderISR2() {
        if (instance2 != nullptr) {
            instance2->readEncoder();
        }
    }
};

Encoder* Encoder::instance1 = nullptr;
Encoder* Encoder::instance2 = nullptr;

}

mtrn3100::Motor leftMotor(11, 12);
mtrn3100::Motor rightMotor(9, 10);
mtrn3100::Encoder leftEncoder(2, 7);
mtrn3100::Encoder rightEncoder(3, 8);

Adafruit_MPU6050 mpu;

float gyroBiasZ = 0.0f;
float yawDegrees = 0.0f;
uint32_t previousIMUTime = 0;

void calibrateGyro();
void updateYaw();
void driveDistance(float targetDistanceMM, int driveSpeed);
void turnRelative(float relativeDegrees);
void executeCommand(char command);
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

    Serial.println("Starting Task 3.4 in 2 seconds");
    delay(2000);

    for (size_t i = 0; COMMANDS[i] != '\0'; i++) {
        executeCommand(COMMANDS[i]);
        stopMotors();
        delay(150);
    }

    stopMotors();
    Serial.println("Task 3.4 finished");
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

void driveDistance(float targetDistanceMM, int driveSpeed) {
    leftEncoder.reset();
    rightEncoder.reset();

    const float KP = 40.0f;
    const float KD = 5.0f;
    const float MAX_CORRECTION = 25.0f;

    const uint32_t CONTROL_PERIOD_US = 10000;
    const uint32_t MAX_DRIVE_TIME_MS = 10000;

    float heading = 0.0f;
    float averageDistance = 0.0f;

    uint32_t previousControlTime = micros();
    uint32_t startTime = millis();

    while (averageDistance < targetDistanceMM) {
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

        float yawRate = gyro.gyro.z - gyroBiasZ;
        heading += yawRate * dt;

        float headingError = -heading;

        float correction =
            KP * headingError -
            KD * yawRate;

        correction = constrain(
            correction,
            -MAX_CORRECTION,
            MAX_CORRECTION
        );

        int leftPWM = -(driveSpeed - correction);
        int rightPWM = driveSpeed + correction;

        leftMotor.setPWM(leftPWM);
        rightMotor.setPWM(rightPWM);

        float leftDistance =
            fabs(leftEncoder.getRotation()) *
            (WHEEL_DIAMETER / 2.0f);

        float rightDistance =
            fabs(rightEncoder.getRotation()) *
            (WHEEL_DIAMETER / 2.0f);

        averageDistance =
            (leftDistance + rightDistance) / 2.0f;

        if (millis() - startTime >= MAX_DRIVE_TIME_MS) {
            break;
        }
    }

    stopMotors();
}

void turnRelative(float relativeDegrees) {
    updateYaw();

    float targetYaw = yawDegrees + relativeDegrees;

    const float KP = 2.8f;
    const float KD = 0.18f;

    const float ANGLE_TOLERANCE = 3.0f;
    const int MIN_TURN_PWM = 45;
    const int MAX_TURN_PWM = 110;

    const uint32_t CONTROL_PERIOD_US = 10000;
    const uint32_t MAX_TURN_TIME_MS = 6000;

    uint32_t previousControlTime = micros();
    uint32_t startTime = millis();
    uint32_t insideToleranceSince = 0;

    while (millis() - startTime < MAX_TURN_TIME_MS) {
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

        float error = targetYaw - yawDegrees;

        if (fabs(error) <= ANGLE_TOLERANCE) {
            stopMotors();

            if (insideToleranceSince == 0) {
                insideToleranceSince = millis();
            }

            if (millis() - insideToleranceSince >= 200) {
                break;
            }

            continue;
        }

        insideToleranceSince = 0;

        float output =
            KP * error -
            KD * yawRateDeg;

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

    stopMotors();
}

void executeCommand(char command) {
    command = static_cast<char>(tolower(command));

    switch (command) {
        case 'f':
            driveDistance(MAZE_CELL_MM, 50);
            break;

        case 'l':
            turnRelative(90.0f);
            break;

        case 'r':
            turnRelative(-90.0f);
            break;
    }
}

void stopMotors() {
    leftMotor.setPWM(0);
    rightMotor.setPWM(0);
}