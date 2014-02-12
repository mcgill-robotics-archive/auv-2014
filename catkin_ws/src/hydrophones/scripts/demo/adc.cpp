/* DEFINES */
#define NUMBER_OF_MICS  3           // SELF-EXPLANATORY
#define PERIOD          75          // PERIOD AT WHICH TO POLL FOR SERIAL DATA

/* PINS */
const int micPins[NUMBER_OF_MICS] = { A0, A1, A2 };
volatile int counter = 0;

/* TIMER */
IntervalTimer serialTimer;

void setup() {
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
    Serial.println((String)counter + ":" + (String)analogRead(micPins[counter++]));
    if (counter == NUMBER_OF_MICS) counter = 0;
}
