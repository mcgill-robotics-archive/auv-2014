/* DEFINES */
#define NUMBER_OF_MICS  3                   // SELF-EXPLANATORY
#define FREQUENCY       500                 // SAMPLING FREQUENCY (Hz)
#define PERIOD          1000 / FREQUENCY    // PERIOD AT WHICH TO POLL FOR SERIAL DATA (ms)

/* PINS */
const int micPins[NUMBER_OF_MICS] = { A3, A1, A2 };
volatile int counter = 0;

/* TIMER */
IntervalTimer serialTimer;

void setup() {
    analogReadResolution(8);
    analogReadAveraging(1);

    /* SETUP SERIAL */
    while (!Serial);

    /* SETUP INPUT PINS */
    for (int i = 0; i < NUMBER_OF_MICS; i++)
        pinMode(micPins[i], INPUT);

    /* SETUP SERIAL TO TRANSFER EVERY SECOND */
    serialTimer.begin(measure, PERIOD);
}

void loop() {
    /* FRUIT LOOPS */
}

void measure() {
    Serial.print(counter);
    Serial.println(analogRead(micPins[counter++]));
    if (counter == NUMBER_OF_MICS) counter = 0;
}
