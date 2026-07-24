#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ============================================================
// CONSTANTS & HARDWARE CONFIGURATION
// ============================================================

const float WHEEL_DIAMETER = 32.0f;          // 32mm wheel diameter
const uint16_t COUNTS_PER_REVOLUTION = 694;  // Encoder CPR
const float CELL_SIZE_MM = 180.0f;            // 180mm per maze cell

namespace mtrn3100 {

/*
 * Motor Class
 */
class Motor {
public:
    Motor(uint8_t pwmPin, uint8_t directionPin) : pwm_pin(pwmPin), dir_pin(directionPin) {
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
    }

    void setPWM(int16_t pwm) {
        if (pwm > 0) {
            digitalWrite(dir_pin, HIGH);
        } else {
            digitalWrite(dir_pin, LOW);
        }

        int16_t pwmMagnitude = abs(pwm);
        pwmMagnitude = constrain(pwmMagnitude, 0, 255);
        analogWrite(pwm_pin, pwmMagnitude);
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

/*
 * Quadrature Encoder Class
 */
class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2) : encoder1_pin(enc1), encoder2_pin(enc2) {
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

        if (instance1 == nullptr) {
            instance1 = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR1, RISING);
        } else if (instance2 == nullptr) {
            instance2 = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR2, RISING);
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
        long countCopy = count;
        interrupts();
        return countCopy;
    }

    float getRotation() {
        long countCopy = getCount();
        return ((float)countCopy / (float)COUNTS_PER_REVOLUTION) * 2.0f * M_PI;
    }

    void reset() {
        noInterrupts();
        count = 0;
        interrupts();
    }

    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;

private:
    volatile long count = 0;
    static Encoder* instance1;
    static Encoder* instance2;

    static void readEncoderISR1() { if (instance1 != nullptr) instance1->readEncoder(); }
    static void readEncoderISR2() { if (instance2 != nullptr) instance2->readEncoder(); }
};

Encoder* Encoder::instance1 = nullptr;
Encoder* Encoder::instance2 = nullptr;

}  // namespace mtrn3100

// Hardware Objects
mtrn3100::Motor leftMotor(11, 12);
mtrn3100::Encoder leftEncoder(2, 7);
mtrn3100::Motor rightMotor(9, 10);
mtrn3100::Encoder rightEncoder(3, 8);

Adafruit_MPU6050 mpu;

// Global State Variables
float gyroBiasZ = 0.0f;
float currentHeadingRad = 0.0f;         // Continuous global heading tracking
float globalTargetHeadingDeg = 0.0f;    // Target heading in degrees

// Function Declarations
void calibrateGyro();
float wrapAngleRadians(float angle);
void stopMotors();
void driveDistance(float targetDistanceMM, int driveSpeed);
void turnToAngle(float targetDegrees);

// ============================================================
// SETUP
// ============================================================

void setup() {
    Serial.begin(9600);
    Wire.begin();

    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (true) {
            stopMotors();
            delay(10);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(500);

    Serial.println("Calibrating Gyro... keep robot stationary");
    calibrateGyro();

    currentHeadingRad = 0.0f;
    globalTargetHeadingDeg = 0.0f;

    Serial.println("Starting maze execution in 2 seconds...");
    delay(2000);

    // Command sequence string
    const char* commandSequence = "lfrfflfr";

    // Loop through each movement action
    for (int i = 0; commandSequence[i] != '\0'; i++) {
        char cmd = commandSequence[i];
        Serial.print("--- Executing Step ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(cmd);

        if (cmd == 'f' || cmd == 'F') {
            // Drive forward 1 cell (180 mm) at base PWM 50
            driveDistance(CELL_SIZE_MM, 50);
        } else if (cmd == 'l' || cmd == 'L') {
            // Turn +90 degrees (Counter-Clockwise)
            globalTargetHeadingDeg += 90.0f;
            turnToAngle(globalTargetHeadingDeg);
        } else if (cmd == 'r' || cmd == 'R') {
            // Turn -90 degrees (Clockwise)
            globalTargetHeadingDeg -= 90.0f;
            turnToAngle(globalTargetHeadingDeg);
        }

        // Brief settling delay between movements
        stopMotors();
        delay(250);
    }

    Serial.println("Maze Sequence Completed!");
}

void loop() {
    // Keep motors off after task completion
    stopMotors();
}

// ============================================================
// HELPER FUNCTIONS
// ============================================================

void stopMotors() {
    leftMotor.setPWM(0);
    rightMotor.setPWM(0);
}

void calibrateGyro() {
    const int SAMPLE_COUNT = 500;
    float totalGyroZ = 0.0f;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sensors_event_t acceleration, gyro, temperature;
        mpu.getEvent(&acceleration, &gyro, &temperature);
        totalGyroZ += gyro.gyro.z;
        delay(5);
    }

    gyroBiasZ = totalGyroZ / SAMPLE_COUNT;
    Serial.print("Gyro bias Z: ");
    Serial.print(gyroBiasZ, 6);
    Serial.println(" rad/s");
}

float wrapAngleRadians(float angle) {
    while (angle > M_PI)  angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

// ============================================================
// PID MOTION CONTROLLERS
// ============================================================

/*
 * Turns the robot to an absolute target heading in degrees using PID control.
 */
void turnToAngle(float targetDegrees) {
    // PID Gains
    const float KP_TURN = 2.5f;
    const float KI_TURN = 0.08f;
    const float KD_TURN = 0.10f;
    
    const float MAX_I_TERM = 30.0f;             // Anti-windup limit
    const int MAX_TURN_PWM = 90;
    const int MIN_TURN_PWM = 40;               // Minimum motor friction threshold
    const float ANGLE_TOLERANCE_DEG = 2.0f;
    const uint32_t CONTROL_PERIOD_US = 10000;  // 100 Hz update loop

    float targetHeadingRad = targetDegrees * M_PI / 180.0f;
    
    uint32_t previousControlTime = micros();
    uint32_t settlementStartTime = 0;
    float integralError = 0.0f;

    while (true) {
        uint32_t currentTime = micros();
        if (currentTime - previousControlTime < CONTROL_PERIOD_US) {
            continue;
        }

        float dt = (currentTime - previousControlTime) / 1000000.0f;
        previousControlTime = currentTime;

        sensors_event_t acceleration, gyro, temperature;
        mpu.getEvent(&acceleration, &gyro, &temperature);

        float yawRate = gyro.gyro.z - gyroBiasZ;
        currentHeadingRad += yawRate * dt;
        currentHeadingRad = wrapAngleRadians(currentHeadingRad);

        float headingErrorRad = wrapAngleRadians(targetHeadingRad - currentHeadingRad);
        float headingErrorDeg = headingErrorRad * 180.0f / M_PI;
        float yawRateDeg = yawRate * 180.0f / M_PI;
        
        // --- INTEGRAL ACCUMULATION & ANTI-WINDUP ---
        if (fabs(headingErrorDeg) < 30.0f) {
            integralError += headingErrorDeg * dt;
            integralError = constrain(integralError, -MAX_I_TERM, MAX_I_TERM);
        } else {
            integralError = 0.0f;
        }

        // PID Control Calculation
        float pTerm = KP_TURN * headingErrorDeg;
        float iTerm = KI_TURN * integralError;
        float dTerm = -KD_TURN * yawRateDeg;

        float controlOutput = pTerm + iTerm + dTerm;

        // Check if within angle tolerance
        if (fabs(headingErrorDeg) <= ANGLE_TOLERANCE_DEG) {
            stopMotors();
            if (settlementStartTime == 0) {
                settlementStartTime = millis();
            } else if (millis() - settlementStartTime >= 300) { 
                // Stable inside tolerance window for 300ms
                break;
            }
        } else {
            settlementStartTime = 0;

            controlOutput = constrain(controlOutput, -MAX_TURN_PWM, MAX_TURN_PWM);

            if (controlOutput > 0.0f && controlOutput < MIN_TURN_PWM) {
                controlOutput = MIN_TURN_PWM;
            } else if (controlOutput < 0.0f && controlOutput > -MIN_TURN_PWM) {
                controlOutput = -MIN_TURN_PWM;
            }

            int turnPWM = (int)controlOutput;
            leftMotor.setPWM(turnPWM);
            rightMotor.setPWM(turnPWM);
        }

        Serial.print("Heading: ");
        Serial.print(currentHeadingRad * 180.0f / M_PI);
        Serial.print(" deg | Error: ");
        Serial.print(headingErrorDeg);
        Serial.print(" deg | ITerm: ");
        Serial.println(iTerm);
    }

    stopMotors();
}

/*
 * Drives the robot forward by a set distance in mm using PID heading correction.
 */
void driveDistance(float targetDistanceMM, int driveSpeed) {
    leftEncoder.reset();
    rightEncoder.reset();

    // PID Gains for straight line tracking
    const float KP_DRIVE = 40.0f;
    const float KI_DRIVE = 1.5f;
    const float KD_DRIVE = 5.0f;
    
    const float MAX_I_TERM = 10.0f;
    const float MAX_CORRECTION = 25.0f;
    const uint32_t CONTROL_PERIOD_US = 10000;  // 100 Hz
    const uint32_t MAX_DRIVE_TIME_MS = 10000;

    float targetHeadingRad = globalTargetHeadingDeg * M_PI / 180.0f;

    uint32_t previousControlTime = micros();
    uint32_t startTime = millis();
    float averageDistanceTravelled = 0.0f;
    float integralHeadingError = 0.0f;

    while (averageDistanceTravelled < targetDistanceMM) {
        uint32_t currentTime = micros();
        if (currentTime - previousControlTime < CONTROL_PERIOD_US) {
            continue;
        }

        float dt = (currentTime - previousControlTime) / 1000000.0f;
        previousControlTime = currentTime;

        sensors_event_t acceleration, gyro, temperature;
        mpu.getEvent(&acceleration, &gyro, &temperature);

        float yawRate = gyro.gyro.z - gyroBiasZ;
        currentHeadingRad += yawRate * dt;
        currentHeadingRad = wrapAngleRadians(currentHeadingRad);

        // Heading Error
        float headingError = wrapAngleRadians(targetHeadingRad - currentHeadingRad);

        // --- PID HEADING CORRECTION ---
        integralHeadingError += headingError * dt;
        integralHeadingError = constrain(integralHeadingError, -MAX_I_TERM, MAX_I_TERM);

        float pTerm = KP_DRIVE * headingError;
        float iTerm = KI_DRIVE * integralHeadingError;
        float dTerm = -KD_DRIVE * yawRate;

        float correction = pTerm + iTerm + dTerm;
        correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

        int leftPWM = -(driveSpeed - correction);
        int rightPWM = driveSpeed + correction;

        leftPWM = constrain(leftPWM, -255, 255);
        rightPWM = constrain(rightPWM, -255, 255);

        leftMotor.setPWM(leftPWM);
        rightMotor.setPWM(rightPWM);

        float distanceLeft = fabs(leftEncoder.getRotation()) * (WHEEL_DIAMETER / 2.0f);
        float distanceRight = fabs(rightEncoder.getRotation()) * (WHEEL_DIAMETER / 2.0f);
        averageDistanceTravelled = (distanceLeft + distanceRight) / 2.0f;

        if (millis() - startTime >= MAX_DRIVE_TIME_MS) {
            Serial.println("Drive segment timed out!");
            break;
        }
    }

    stopMotors();
}
