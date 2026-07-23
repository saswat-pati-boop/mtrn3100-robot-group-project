#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


// ============================================================
// MOTOR CLASS
// ============================================================

namespace mtrn3100 {

class Motor {
public:
    Motor(uint8_t pwmPin, uint8_t directionPin)
        : pwm_pin(pwmPin), dir_pin(directionPin) {

        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
    }

    void setPWM(int16_t pwm) {

        // Set direction from the sign of the PWM command
        if (pwm > 0) {
            digitalWrite(dir_pin, HIGH);
        } else {
            digitalWrite(dir_pin, LOW);
        }

        // Convert to a positive PWM magnitude
        int16_t pwmMagnitude = abs(pwm);

        // Arduino PWM must remain between 0 and 255
        pwmMagnitude = constrain(pwmMagnitude, 0, 255);

        analogWrite(pwm_pin, pwmMagnitude);
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}  // namespace mtrn3100


// ============================================================
// HARDWARE OBJECTS
// ============================================================

// Same motor pins as your working straight-line code
mtrn3100::Motor leftMotor(11, 12);
mtrn3100::Motor rightMotor(9, 10);

// MPU6050 IMU
Adafruit_MPU6050 mpu;


// ============================================================
// CONTROLLER CONSTANTS
// ============================================================

// Gyroscope offset found during startup calibration
float gyroBiasZ = 0.0f;

// Current estimated heading in radians
float currentHeading = 0.0f;

/*
 * The robot must turn 90 degrees clockwise.
 *
 * With the usual gyro convention:
 * positive angle = anticlockwise
 * negative angle = clockwise
 */
const float TARGET_HEADING_DEG = -90.0f;
const float TARGET_HEADING_RAD =
    TARGET_HEADING_DEG * M_PI / 180.0f;

/*
 * PD controller gains.
 *
 * These are starting values and may need tuning.
 */
const float KP = 2.2f;
const float KD = 0.30f;

// Maximum turning PWM
const int MAX_TURN_PWM = 90;

// You said PWM 50 reliably moves the robot
const int MIN_TURN_PWM = 50;

/*
 * Stop within 2.5 degrees.
 *
 * The assessment allows ±5 degrees, so this gives some margin.
 */
const float ANGLE_TOLERANCE_DEG = 2.5f;

// Controller runs every 10 ms = 100 Hz
const uint32_t CONTROL_PERIOD_US = 10000;

/*
 * Change this to -1 if the robot turns in the wrong motor direction.
 */
const int TURN_DIRECTION_SIGN = 1;


// ============================================================
// FUNCTION DECLARATIONS
// ============================================================

void calibrateGyro();

float wrapAngleRadians(float angle);

void setTurnPWM(int pwm);

void stopMotors();

void updateTurnController();


// ============================================================
// SETUP
// ============================================================

void setup() {

    Serial.begin(9600);
    Wire.begin();

    // Initialise the MPU6050
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");

        while (true) {
            stopMotors();
            delay(10);
        }
    }

    Serial.println("MPU6050 found");

    // Same IMU settings as your straight-line program
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Allow the IMU readings to settle
    delay(500);

    /*
     * Keep the robot completely stationary during calibration.
     */
    Serial.println("Keep robot stationary while gyro calibrates");

    calibrateGyro();

    // Treat the starting orientation as zero degrees
    currentHeading = 0.0f;

    Serial.println("Starting 90 degree clockwise turn in 2 seconds");

    delay(2000);
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

    /*
     * This runs continuously.
     *
     * Initially, it turns the robot to -90 degrees.
     *
     * After reaching the target, it remains active. If the demonstrator
     * lifts and rotates the robot, the gyro tracks that movement.
     * When the robot is placed back on the floor, it rotates back to
     * the same -90 degree setpoint.
     */
    updateTurnController();
}


// ============================================================
// GYRO CALIBRATION
// ============================================================

void calibrateGyro() {

    const int SAMPLE_COUNT = 500;

    float totalGyroZ = 0.0f;

    for (int i = 0; i < SAMPLE_COUNT; i++) {

        sensors_event_t acceleration;
        sensors_event_t gyro;
        sensors_event_t temperature;

        mpu.getEvent(
            &acceleration,
            &gyro,
            &temperature
        );

        totalGyroZ += gyro.gyro.z;

        delay(5);
    }

    // Average stationary gyro reading
    gyroBiasZ = totalGyroZ / SAMPLE_COUNT;

    Serial.print("Gyro bias Z: ");
    Serial.print(gyroBiasZ, 6);
    Serial.println(" rad/s");
}


// ============================================================
// ANGLE WRAPPING
// ============================================================

float wrapAngleRadians(float angle) {

    /*
     * Keep the angle between -pi and +pi.
     *
     * This makes the controller choose the shortest direction back
     * to the target.
     */
    while (angle > M_PI) {
        angle -= 2.0f * M_PI;
    }

    while (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }

    return angle;
}


// ============================================================
// MOTOR CONTROL
// ============================================================

void setTurnPWM(int pwm) {

    pwm = constrain(pwm, -255, 255);

    pwm *= TURN_DIRECTION_SIGN;

    /*
     * Your motors are physically mounted in opposite orientations.
     *
     * For straight motion you used:
     *
     * leftMotor.setPWM(-speed);
     * rightMotor.setPWM(speed);
     *
     * Therefore, using the same electrical sign on both motors should
     * rotate the robot in place.
     */
    leftMotor.setPWM(pwm);
    rightMotor.setPWM(pwm);
}


void stopMotors() {

    leftMotor.setPWM(0);
    rightMotor.setPWM(0);
}


// ============================================================
// HEADING CONTROLLER
// ============================================================

void updateTurnController() {

    static uint32_t previousControlTime = micros();
    static uint32_t previousPrintTime = millis();

    uint32_t currentTime = micros();

    // Only update every 10 ms
    if (currentTime - previousControlTime < CONTROL_PERIOD_US) {
        return;
    }

    // Time since previous update in seconds
    float dt =
        (currentTime - previousControlTime) / 1000000.0f;

    previousControlTime = currentTime;


    // Read the MPU6050
    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    mpu.getEvent(
        &acceleration,
        &gyro,
        &temperature
    );


    /*
     * Correct the measured yaw rate using the startup gyro bias.
     *
     * Units are radians per second.
     */
    float yawRate = gyro.gyro.z - gyroBiasZ;


    /*
     * Estimate heading by integrating angular velocity:
     *
     * heading = heading + yaw rate × time
     */
    currentHeading += yawRate * dt;

    currentHeading = wrapAngleRadians(currentHeading);


    /*
     * Calculate shortest angular error between target and current
     * heading.
     */
    float headingError =
        wrapAngleRadians(TARGET_HEADING_RAD - currentHeading);


    // Convert angles to degrees for controller tuning and printing
    float headingErrorDeg =
        headingError * 180.0f / M_PI;

    float currentHeadingDeg =
        currentHeading * 180.0f / M_PI;

    float yawRateDeg =
        yawRate * 180.0f / M_PI;


    /*
     * Stop when within the required tolerance.
     *
     * The controller remains active, so if the robot is moved away,
     * it will immediately begin returning to the target.
     */
    if (fabs(headingErrorDeg) <= ANGLE_TOLERANCE_DEG) {

        stopMotors();
    }
    else {

        /*
         * PD controller:
         *
         * control = KP × angular error - KD × yaw rate
         *
         * The proportional term turns toward the target.
         * The derivative term slows the turn and reduces overshoot.
         */
        float controlOutput =
            KP * headingErrorDeg
            - KD * yawRateDeg;


        // Limit maximum turning speed
        controlOutput = constrain(
            controlOutput,
            -MAX_TURN_PWM,
            MAX_TURN_PWM
        );


        /*
         * Overcome motor deadband.
         *
         * If the controller wants motion, ensure the PWM is at least
         * the minimum value that reliably moves the robot.
         */
        if (controlOutput > 0.0f &&
            controlOutput < MIN_TURN_PWM) {

            controlOutput = MIN_TURN_PWM;
        }
        else if (controlOutput < 0.0f &&
                 controlOutput > -MIN_TURN_PWM) {

            controlOutput = -MIN_TURN_PWM;
        }


        setTurnPWM((int)controlOutput);
    }


    // Print useful values every 100 ms
    if (millis() - previousPrintTime >= 100) {

        previousPrintTime = millis();

        Serial.print("Heading: ");
        Serial.print(currentHeadingDeg, 2);

        Serial.print(" deg | Error: ");
        Serial.print(headingErrorDeg, 2);

        Serial.print(" deg | Yaw rate: ");
        Serial.print(yawRateDeg, 2);

        Serial.println(" deg/s");
    }
}