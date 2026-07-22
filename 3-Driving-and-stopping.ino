#include <Wire.h>
#include <VL6180X.h>

VL6180X sensor;

const int enablePins[] = {A0, A1, A2}; // A0 = left sensor, A1 = right sensor, A2 = front sensor

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Disable all sensors
  for (int i = 0; i < 3; i++) {
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], LOW);
  }

  delay(100);

  // Test each sensor
  for (int i = 0; i < 3; i++) {
    digitalWrite(enablePins[i], HIGH);
    delay(100);

    sensor.init();
    sensor.configureDefault();
    sensor.setTimeout(250);

    Serial.print("Sensor on A");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(sensor.readRangeSingleMillimeters());

    

    digitalWrite(enablePins[i], LOW);
    delay(100);
  }
}

void loop() {}