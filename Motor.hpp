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

