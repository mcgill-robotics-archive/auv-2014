#define SIZE_OF_ARRAY   100
#define PERIOD          1000000

const int pin = 0;
volatile unsigned long times[SIZE_OF_ARRAY];
volatile int counter = 0;

IntervalTimer serialTimer;

void setup() {
    /* SET UP SERIAL */
    while(!Serial);

    /* SET INPUT PINS */
    pinMode(pin, INPUT);

    /* ATTACH INTERRUPTS */
    attachInterrupt(pin, button, CHANGE);

    /* SET UP SERIAL TO TRANSFER EVERY SECOND */
    serialTimer.begin(send, PERIOD);
}

void loop() {

}

/* INTERRUPT */
void button() {
    times[counter++] = micros();
}

/* SEND AND RESET DATA */
void send() {
    for (int i = 0; i < SIZE_OF_ARRAY; i++) {
        if (times[i] != 0) {
            Serial.println(times[i]);
            times[i] = 0;
        } else break;
    }
    counter = 0;
}
