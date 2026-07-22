
#pragma once

#include <Arduino.h>

#include "math.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
const float WHEEL_DIAMETER = 32; // 32mm
namespace mtrn3100 {


// The motor class is a simple interface designed to assist in motor control
// You may choose to impliment additional functionality in the future such as dual motor or speed control 
class Motor {
public:
    Motor( uint8_t pwm_pin, uint8_t in2) :  pwm_pin(pwm_pin), dir_pin(in2) {
        pinMode(pwm_pin,OUTPUT); // driver
        pinMode(dir_pin,OUTPUT); // direction
    }


    // This function outputs the desired motor direction and the PWM signal. 
    // NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.

    void setPWM(int16_t pwm) {

      // TODO: Output digital direction pin based on if input signal is positive or negative.
      
      if (pwm > 0) {
        digitalWrite(dir_pin,HIGH);
      } else {
        digitalWrite(dir_pin,LOW);
      }
      
      // TODO: Output PWM signal between 0 - 255.
      pwm = abs(pwm);

      if (pwm > 255) {
        analogWrite(pwm_pin,255);
      } else {
        analogWrite(pwm_pin,pwm);
      }
      //delay(3000);
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}  // namespace mtrn3100

#pragma once

#include <Arduino.h>

namespace mtrn3100 {


// The encoder class is a simple interface which counts and stores an encoders count.
// Encoder pin 1 is attached to the interupt on the arduino and used to trigger the count.
// Encoder pin 2 is attached to any digital pin and used to derive rotation direction.
// The count is stored as a volatile variable due to the high frequency updates. 
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
        noInterrupts();
        if (digitalRead(encoder1_pin) == HIGH) {
          count++;
        } else {
          count--;
        }
        interrupts();
    }

    float getRotation() {
        return ((float)count / counts_per_revolution) * 2 * M_PI;
    }

    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    volatile int8_t direction;
    float position = 0;
    uint16_t counts_per_revolution = 694; // was 1400
    volatile long count = 0;
    uint32_t prev_time;
    bool read = false;

private:
    static void readEncoderISR1() { if (instance1 != nullptr) instance1->readEncoder(); }
    static void readEncoderISR2() { if (instance2 != nullptr) instance2->readEncoder(); }

    static Encoder* instance1;
    static Encoder* instance2;
};

Encoder* Encoder::instance1 = nullptr;
Encoder* Encoder::instance2 = nullptr;


}  // namespace mtrn3100


mtrn3100::Motor leftMotor(11,12); //1
mtrn3100::Encoder leftEncoder(2,7);
//mtrn3100::PIDController PIDController(100,0,0);
mtrn3100::Motor rightMotor(9,10); //1
mtrn3100::Encoder rightEncoder(3,8);

Adafruit_MPU6050 mpu;

// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(9600);
//   //PIDController.zeroAndSetTarget(0,M_PI);


//    // Check if the MPU6050 sensor is detected
//   if (!mpu.begin()) {
//     Serial.println("Failed to find MPU6050 chip");
//     while (1) {
//       delay(10);
//     }
//   }
//   Serial.println("MPU6050 Found!");

//   // set accelerometer range to +-8G
//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

//   // set gyro range to +- 500 deg/s
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);

//   // set filter bandwidth to 21 Hz
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

//   // Add a delay for stability
//   delay(100);
// }

// void loop() {
//   // put your main code here, to run repeatedly:

//   for(int i =0; i <10;i++) {
// // leftMotor.setPWM(30);
//       //   rightMotor.setPWM(-50);
//       // delay(3000);

//     leftMotor.setPWM(-30);
//         rightMotor.setPWM(30);
//       delay(3000);
//   }
    

//     for(int i = 0; i < 4; i++) {
//       leftMotor.setPWM(50);    
//       delay(2450);
//       leftMotor.setPWM(0);
//     }
    
//     delay(1450);

//     for(int i = 0; i < 4; i++) {
//       rightMotor.setPWM(50);    
//       delay(2450);
//       rightMotor.setPWM(0);
//     }


//     rightMotor.setPWM(150);    
//     delay(1450);
//     rightMotor.setPWM(0);
//     delay(1450);

//     Serial.print("Current Count: ");
//     Serial.println(leftEncoder.count);
//     Serial.println(leftEncoder.getRotation());
//     //delay(30000);leftMotor.setPWM(0);

//     // Get new sensor events with the readings
//     sensors_event_t a, g, temp;
//     mpu.getEvent(&a, &g, &temp);

//     // Print out the rotation readings in rad/s
//     Serial.print("Rotation:       X:");
//     Serial.print(g.gyro.x);
//     Serial.print(", Y:");
//     Serial.print(g.gyro.y);
//     Serial.print(", Z:");
//     Serial.print(g.gyro.z);
//     Serial.println(" (rad/s)");
    

// }
void turnToAngle(float targetDegrees, int turnSpeed) {
  
}
void driveDistance(int distance, int driveSpeed) {
  leftEncoder.count = 0;
  rightEncoder.count = 0;

  const float errorDistance = 15;
  const int driveDirection = driveSpeed > 0 ? 1 : -1;

  leftMotor.setPWM(-1 * driveDirection * fabs(driveSpeed));
  rightMotor.setPWM( driveDirection * fabs(driveSpeed));

  float distanceTravelledLeft;
  float distanceTravelledRight;
  float averageDistanceTravelled = 0;
  

  while(averageDistanceTravelled < distance) {
    distanceTravelledLeft = (leftEncoder.getRotation() * (WHEEL_DIAMETER / 2));
    distanceTravelledRight = (rightEncoder.getRotation() * (WHEEL_DIAMETER / 2));
    averageDistanceTravelled = (distanceTravelledLeft + distanceTravelledRight) / 2;
    Serial.print("left:");
    Serial.println(leftEncoder.getRotation());
    Serial.print("right:");
    Serial.println(rightEncoder.getRotation());
  } 

  leftMotor.setPWM(0);
  rightMotor.setPWM(0);
  
  delay(5000);

}


// // Temporary calibration sketch
// void setup() {
//   Serial.begin(9600);
// }
// void loop() {
//   Serial.println(leftEncoder.count);
//   delay(500);
// }
void setup() {
  Serial.begin(9600);
}

void loop() {

// while (1) {
//   Serial.print("left: ");
//   Serial.print(leftEncoder.getRotation());
//   Serial.print("  right: ");
//   Serial.println(rightEncoder.getRotation());
//   delay(100);
// }  
  // Drive 200mm (20cm)
  driveDistance(200, 40);


    //driveDistance(200,-40);

  // 90 Degree Rotation - Wk4 Submission // Hardcoded

  // Go clockwise
  for (int i = 0; i < 4; i++) {
    leftMotor.setPWM(60);
    rightMotor.setPWM(60);
    delay(1000);
    leftMotor.setPWM(0);
    rightMotor.setPWM(0);
    delay(2000);
  }

  // Go anti-clockwise
  for (int i = 0; i < 4; i++) {
    leftMotor.setPWM(-60);
    rightMotor.setPWM(-60);
    delay(1000);
    leftMotor.setPWM(0);
    rightMotor.setPWM(0);
        delay(2000);

  }


}

