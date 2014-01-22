/* DEFINES */
#define SIZE_OF_ARRAY   400         // STORES TIMES OF INTERRUPTS
#define NUMBER_OF_MICS  4           // SELF-EXPLANATORY
#define PERIOD          1000000     // PERIOD AT WHICH TO POLL FOR SERIAL DATA
#define RESOLUTION      10          // RESOLUTION OF PWM
#define FREQUENCY       46875       // FREQUENCY OF PWM

/* PINS */
const int micPins[NUMBER_OF_MICS] = { 0, 1, 2, 3 };

/* TIMES OF EACH HYDROPHONE */
volatile unsigned long micO[SIZE_OF_ARRAY];
volatile unsigned long micX[SIZE_OF_ARRAY];
volatile unsigned long micY[SIZE_OF_ARRAY];
volatile unsigned long micZ[SIZE_OF_ARRAY];

/* COUNTERS */
volatile int counterO = 0;
volatile int counterX = 0;
volatile int counterY = 0;
volatile int counterZ = 0;

/* TIMER */
IntervalTimer serialTimer;

void setup() {
    /* SETUP SERIAL */
    while (!Serial);

    /* SETUP INPUT PINS */
    for (int i = 0; i < NUMBER_OF_MICS; i++)
        pinMode(micPins[i], INPUT);

    /* SETUP INTERRUPTS */
    attachInterrupt(micPins[0], micOISR, CHANGE);
    attachInterrupt(micPins[1], micXISR, CHANGE);
    attachInterrupt(micPins[2], micYISR, CHANGE);
    attachInterrupt(micPins[3], micZISR, CHANGE);

    /* WAIT UNTIL FIRST PING */
    while (!counterO);
    delay(500);

    /* SETUP SERIAL TO TRANSFER EVERY SECOND */
    serialTimer.begin(socialize, PERIOD);
}

void loop() {
    /* FRUIT LOOPS */
}

/* TODO: WATCH OUT FOR STACK OVERFLOW */
void micOISR() {
    /* RECORD TIME */
    micO[counterO++] = micros();
}

void micXISR() {
    /* RECORD TIME */
    micX[counterX++] = micros();
}

void micYISR() {
    /* RECORD TIME */
    micY[counterY++] = micros();
}

void micZISR() {
    /* RECORD TIME */
    micZ[counterZ++] = micros();
}

void socialize() {
    /* SEND AND RESET TIMES */
    for (int i = 0; i < counterO; i++) {
        Serial.println("O" + (String)micO[i]);
        micO[i] = 0;
    } 
    for (int i = 0; i < counterX; i++) {
        Serial.println("X" + (String)micX[i]);
        micX[i] = 0;
    }
    for (int i = 0; i < counterY; i++) {
        Serial.println("Y" + (String)micY[i]);
        micY[i] = 0;
    }
    for (int i = 0; i < counterZ; i++) {
        Serial.println("Z" + (String)micZ[i]);
        micZ[i] = 0;
    }

    /* INDICATE END OF STREAM */
    if (counterO || counterX || counterY || counterZ)
        Serial.println(".");

    /* RESET COUNTERS */
    counterO = 0;
    counterX = 0;
    counterY = 0;
    counterZ = 0;
}