/* DEFINES */
#define DOUT    12
#define BCK     13
#define LRCK    14

volatile bool left = true; 
volatile int counter = 32;
volatile uint32_t dataLeft = 0, dataRight = 0;

float phase = 0.0;
float twopi = 3.14159 * 2;
elapsedMicros usec = 0;

void setup() {
    analogWriteResolution(12);

    /* SETUP SERIAL */
    while (!Serial);

    /* SETUP INPUT PINS */
    pinMode(DOUT, INPUT);
    pinMode(BCK, INPUT);
    pinMode(LRCK, INPUT);

    /* SETUP CLOCK INTERRUPTS */
    attachInterrupt(BCK, bitClock, FALLING);
    attachInterrupt(LRCK, leftRightClock, CHANGE);
}

void leftRightClock(void) {
    /* RESET COUNTER */ 
    counter = 32;

    /* VERIFY CHANNEL */
    if (digitalRead(BCK)){
        left = true;
        Serial.print(dataLeft);
        Serial.print("\t");
        Serial.println(dataRight);
        dataLeft = 0;
    } else {
        left = false;
        dataRight = 0;
    }
}

void bitClock(void) {
    if (left)
        dataLeft  |= (digitalRead(DOUT) << counter--);
    else
        dataRight |= (digitalRead(DOUT) << counter--);
}

void loop() {
    /* FRUIT LOOPS */
    float val = sin(phase) * 2000.0 + 2050.0;
    analogWrite(A14, (int)val);
    phase = phase + 0.02;
    if (phase >= twopi) phase = 0;
    while (usec < 500) ; // wait
    usec = usec - 500;
}

