#pragma once

#include <Arduino.h>

#include <Motor.hpp>
#include <Encoder.hpp>
#include <PIDController.hpp>

#include <math.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const float WHEEL_DIAMETER = 32.0f; // 32mm
const uint16_t COUNTS_PER_REVOLUTION = 694;


/*
 * Robot hardware objects
 *
 * Left motor:
 *  PWM pin       = 11
 *  Direction pin = 12
 *
 * Left encoder:
 *  Interrupt pin = 2
 *  Direction pin = 7
 *
 * Right motor:
 *  PWM pin       = 9
 *  Direction pin = 10
 *
 * Right encoder:
 *  Interrupt pin = 3
 *  Direction pin = 8
 */

mtrn3100::Motor leftMotor(11,12); //1
mtrn3100::Encoder leftEncoder(2,7);
//mtrn3100::PIDController PIDController(100,0,0);
mtrn3100::Motor rightMotor(9,10); //1
mtrn3100::Encoder rightEncoder(3,8);

Adafruit_MPU6050 mpu;

/*
 * Measured stationary offset in the MPU6050 Z-axis gyro.
 *
 * This bias is subtracted from future gyro measurements to reduce
 * heading drift caused by the sensor reporting a small angular
 * velocity while stationary.
 */

float gyroBiasZ = 0.0f;

void calibrateGyro();

void driveDistance(float targetDistanceMM, int driveSpeed);

void turnToAngle(float targetDegrees, int turnSpeed);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Start I2C communication for the MPU6050
  Wire.begin();

   // Check if the MPU6050 sensor is detected
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (true) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  /*
     * Apply a 21 Hz digital low-pass filter.
     *
     * This reduces high-frequency sensor noise in the accelerometer
     * and gyroscope measurements.
     */

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

   // Allow the sensor readings to settle
  delay(500);

  // Measure the stationary gyroscope offset
  calibrateGyro();

  Serial.println("Starting in 2 seconds");
  delay(2000);

  /*
    * Command the robot to drive approximately 1020 mm
    * using a base motor command of 50.
    */

  driveDistance(1020,50);

  Serial.println("Finished!");
}

void loop() {
    /*
     * Empty because the current test only needs to run once.
     *
     * The movement command is called from setup().
     */
}


/*
 * Calibrates the Z-axis gyroscope.
 *
 * The robot must remain stationary during this function.
 *
 * The average of 500 gyro readings is treated as the sensor bias.
 */

void calibrateGyro() {
  const int SAMPLE_COUNT = 500;
  float totalGyroZ = 0.0f;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    // Read accelerometer, gyroscope and temperature data
    mpu.getEvent(&acceleration, &gyro, &temperature);

    // Add the current Z-axis angular velocity to the running total
    totalGyroZ += gyro.gyro.z;

    // Small delay between samples
    delay(5);
  }

  // Calculate the average stationary Z-axis gyro reading
  gyroBiasZ = totalGyroZ / SAMPLE_COUNT;

  Serial.print("Gyro bias Z: ");
  Serial.println(gyroBiasZ, 6);
  Serial.println(" rad/s");
}


/*
 * Drives the robot forward for a requested distance.
 *
 * targetDistanceMM:
 *  Desired travel distance in millimetres
 *
 * driveSpeed:
 *  Base PWM magnitude applied to each motor
 *
 * The encoders determine how far the robot has travelled.
 * The gyroscope is used to maintain a straight heading.
 */

void driveDistance(float targetDistanceMM, int driveSpeed) {
  leftEncoder.reset();
  rightEncoder.reset();

  /*
     * Heading-controller gains
     *
     * KP:
     *  Corrects based on accumulated heading error
     *
     * KD:
     *  Corrects based on current yaw rate
     *
     * This behaves like a PD heading controller.
     */

  const float KP = 40.0f;
  const float KD = 5.0f;

  // Limit the controller's influence on the motor PWM values
  const float MAX_CORRECTION = 25.0f;

  /*
     * Run the controller once every 10,000 microseconds.
     *
     * 10,000 µs = 0.01 s
     * Control frequency = 100 Hz
     */
  const uint32_t CONTROL_PERIOD_US = 10000; //100Hz

  // Stop the robot if the requested distance is not reached within 25 s
  const uint32_t MAX_DRIVE_TIME_MS = 25000;

  /*
     * Estimated heading in radians.
     *
     * Heading is obtained by integrating the measured yaw rate:
     *
     * heading = heading + yawRate × dt
     */
  float heading = 0.0f;

  uint32_t previousControlTime = micros();
  uint32_t startTime = millis();
  uint32_t previousPrintTime = millis();

   // Current estimated average distance travelled by both wheels
  float averageDistanceTravelled = 0.0f;


  /*
     * Continue driving until the average encoder distance reaches
     * the requested target distance.
     */
  while (averageDistanceTravelled < targetDistanceMM) {
    uint32_t currentTime = micros();

    /*
         * Wait until the next 100 Hz control update.
         *
         * The loop immediately restarts if fewer than 10 ms have passed.
         */
    if (currentTime - previousControlTime < CONTROL_PERIOD_US) {
      continue;
    }

    // Calculate time since the previous controller update in seconds
    float dt = (currentTime - previousControlTime) / 1000000.0f;

    // Save the current time for the next control-loop iteration
    previousControlTime = currentTime;

    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;


    // Read the latest MPU6050 measurements
    mpu.getEvent(&acceleration, &gyro, &temperature);


    /*
         * Correct the measured Z-axis angular velocity using the
         * stationary bias found during calibration.
         *
         * yawRate is measured in radians per second.
         */
    float yawRate = gyro.gyro.z - gyroBiasZ;


    /*
         * Integrate yaw rate to estimate the robot's heading.
         *
         * This assumes yaw rate remains approximately constant over dt.
         */
    heading += yawRate * dt;

    // The robot should maintain its original heading of zero radians
    const float targetHeading = 0.0f;

    // Difference between desired and estimated heading
    float headingError = targetHeading - heading;

    /*
         * PD heading controller
         *
         * Proportional term:
         *      KP × headingError
         *
         * Derivative term:
         *      -KD × yawRate
         *
         * Since the target heading is constant, the derivative of the
         * heading error is approximately the negative yaw rate.
         */
    float correction = KP * headingError - KD * yawRate;

    // Prevent excessively large steering corrections
    correction = constrain(
      correction,
      -MAX_CORRECTION,
      MAX_CORRECTION
    );

    /*
         * Apply opposite motor signs because the motors are physically
         * mounted in opposite orientations.
         *
         * The correction changes the left and right motor speeds to
         * steer the robot back toward the target heading.
         */
    int leftPWM = -(driveSpeed - correction);
    int rightPWM = driveSpeed + correction;

    // Keep both motor commands within the valid PWM range
    leftPWM = constrain(leftPWM, -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);

    // Send the calculated commands to the motors
    leftMotor.setPWM(leftPWM);
    rightMotor.setPWM(rightPWM);


    /*
         * Convert encoder rotation into linear distance.
         *
         * Arc length:
         *      distance = rotation angle × wheel radius
         *
         * The absolute value is used because the left and right encoder
         * counts may have opposite signs due to motor orientation.
         */
    float distanceTravelledLeft = fabs(leftEncoder.getRotation()) * (WHEEL_DIAMETER / 2.0f);

    float distanceTravelledRight = fabs(rightEncoder.getRotation()) * (WHEEL_DIAMETER / 2.0f);

    /*
         * Use the average wheel distance as the robot's estimated
         * forward travel distance.
         */
    averageDistanceTravelled = (distanceTravelledLeft + distanceTravelledRight) / 2.0f;

    if (millis() - previousPrintTime >= 100) {
      previousPrintTime = millis();

      Serial.print("Distance:");
      Serial.print(averageDistanceTravelled);

      Serial.print(", yaw rate:");
      Serial.print(yawRate, 4);

      Serial.print(", correction:");
      Serial.println(correction);
    }

     /*
         * Safety timeout.
         *
         * Stops the movement if the encoders fail or the robot becomes
         * physically stuck and cannot reach the target distance.
         */
    if (millis() - startTime >= MAX_DRIVE_TIME_MS) {
        Serial.println("Drive timed out");
        break;
    }
  }
  
  // Stop both motors after reaching the target or timing out
  leftMotor.setPWM(0);
  rightMotor.setPWM(0);

  Serial.print("Final distance: ");
  Serial.print(averageDistanceTravelled);
  Serial.println(" mm");
}
