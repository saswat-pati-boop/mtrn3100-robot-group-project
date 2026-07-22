class Encoder {
public:

    Encoder(uint8_t enc1, uint8_t enc2) : encoder1_pin(enc1), encoder2_pin(enc2) {
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);

         /*
         * Each Encoder object is assigned one of the two available
         * interrupt service routines.
         */

        if (instance1 == nullptr) {
            instance1 = this;

            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR1, RISING);
        } 
        else if (instance2 == nullptr) {
            instance2 = this;
            attachInterrupt(digitalPinToInterrupt(encoder1_pin), readEncoderISR2, RISING);
        }
    }

     /*
     * Called whenever encoder channel A produces a rising edge.
     *
     * The state of encoder channel B determines whether the count
     * should increase or decrease.
     */

    void readEncoder() {
        if (digitalRead(encoder2_pin) == HIGH) {
          count++;
        } else {
          count--;
        }
    }

    /*
     * Safely returns the current encoder count.
     *
     * Interrupts are temporarily disabled so the count cannot change
     * while it is being copied.
     */

    long getCount() {
      noInterrupts();
      long countCopy = count;
      interrupts();

      return countCopy;
    }

    /*
     * Converts encoder counts into wheel rotation in radians.
     *
     * rotation =
     *      counts / counts-per-revolution × 2π
     */

    float getRotation() {
        long countCopy = getCount();

        return ((float)countCopy / (float)COUNTS_PER_REVOLUTION) * 2.0f * M_PI;
    }

    /*
     * Resets the encoder count to zero.
     */

    void reset() {
      noInterrupts();
      count = 0;
      interrupts();
    }

    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;

private:
// Updated by the interrupt service routine
    volatile long count = 0;

    /*
     * Static pointers allow the static interrupt functions to call
     * the readEncoder() function belonging to the correct object.
     */
    static Encoder* instance1;
    static Encoder* instance2;

    // Interrupt service routine for the encoders
    static void readEncoderISR1() { if (instance1 != nullptr) instance1->readEncoder(); }
    static void readEncoderISR2() { if (instance2 != nullptr) instance2->readEncoder(); }

};
