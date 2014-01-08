/**
 *  
 *  #######   INTERRUPT-BASED TIME DIFFERENCE HYDROPHONE ARRAY   #######
 *
 *  @author     Anass Al-Wohoush
 *  @version    0.1
 *  
 *  This Teensy 3.0 sketch collects the times at which every hydrophone
 *  in the hydrophone array passes a threshold thanks to an interrupt-
 *  driven system. It then sends the collected data via I2C to a master
 *  device when requested with parity bits set to ensure accurate
 *  communication.
 *
 *  NOTE: This has not been tested yet, but it compiles.
 *  
 */

/* INCLUDES */
#include <Wire.h>

/* DEFINES */
#define NUMBER_OF_MICS  4           // NUMBER OF HYDROPHONES
#define ADDRESS         44          // ADDRESS OF I2C SLAVE (PLACEHOLDER)
#define TOO_LONG        1000        // THRESHOLD OF DIFFERENCE IN TIME
#define THRESHOLD       330         // INITIAL THRESHOLD OF INTERRUPTS

/* PINS OF EACH MIC */
const int pins[NUMBER_OF_MICS] = { 0, 1, 2, 3 };

/* GLOBAL VARIABLES */
int threshold = THRESHOLD;          // THRESHOLD OF INTERRUPTS
int size = sizeof(long);            // SIZE OF LONG TYPE FOR CODE PORTABILITY
bool ready;                         // STATE WHETHER READY TO SEND OR NOT

/* MIC STRUCTURE */
struct mic {
    volatile unsigned long time;    // TIME RECORDED BY MIC
    volatile bool done;             // STATES WHETHER DONE OR NOT
    void reset();                   // RESET MIC DATA
} mics[NUMBER_OF_MICS];

/* RESET MIC */
void mic::reset() {
    this->time = 0;
    this->done = false;
}

/* PROTOTYPES */
inline void reset(void);
void mic0ISR(void);
void mic1ISR(void);
void mic2ISR(void);
void mic3ISR(void);
void sendData(void);
int countDone(void);
unsigned long timeOfLastPulse(void);
const unsigned char * generateParityBit(unsigned long);

/* SETUP */
void setup() {
    /* SET UP I2C AS A SLAVE */
    Wire.begin(ADDRESS);
    Wire.onRequest(sendData);

    for (int i = 0; i < NUMBER_OF_MICS; i++)
        pinMode(pins[i], INPUT);

    /* SET UP INTERRUPTS */
    attachInterrupt(pins[0], mic0ISR, FALLING);
    attachInterrupt(pins[1], mic1ISR, FALLING);
    attachInterrupt(pins[2], mic2ISR, FALLING);
    attachInterrupt(pins[3], mic3ISR, FALLING);

    /* INITIALIZE */
    reset();
}

/* FRUIT LOOPS */
void loop() {
    int done = countDone();

    /* SET READY IF DONE */
    if (done == NUMBER_OF_MICS)
        ready = true;

    /* RESET IF IT HAS BEEN TOO LONG SINCE LAST MIC */
    else if (done > 0 && !ready && (micros() - timeOfLastPulse()) > TOO_LONG) {
        reset();
    }
}

/* RESET ALL */
inline void reset() {
    for (int i = 0; i < NUMBER_OF_MICS; i++)
        mics[i].reset();

    ready = false;
}

/* MIC INTERRUPT SERVICE ROUTINES */
void mic0ISR() {
    mics[0].time = micros();
    mics[0].done = true;
}

void mic1ISR() {
    mics[1].time = micros();
    mics[1].done = true;
}

void mic2ISR() {
    mics[2].time = micros();
    mics[2].done = true;
}

void mic3ISR() {
    mics[3].time = micros();
    mics[3].done = true;
}

/* SEND DATA VIA I2C */
void sendData() {
    /* DISABLE INTERRUPTS */
    noInterrupts();

    /* WRITE DATA WITH PARITY BIT */
    for (int i = 0; i < NUMBER_OF_MICS; i++)
        Wire.write(generateParityBit(mics[i].time), size);

    /* RESET DATA */
    reset();

    /* REENABLE INTERRUPTS */
    interrupts();
}

/* COUNT HOW MANY MICS ARE DONE */
int countDone() {
    int done = 0;

    for (int i = 0; i < NUMBER_OF_MICS; i++)
        if (mics[i].done)
            done++;

    return done;
}

/* RETURN TIME OF LAST PULSE */
unsigned long timeOfLastPulse() {
    unsigned long timer = mics[0].time;

    for (int i = 1; i < NUMBER_OF_MICS; i++)
        if (timer < mics[i].time)
            timer = mics[i].time;

    return timer;
}

/* GENERATE ARRAY OF BYTES WITH PARITY BIT CHECKED */
const unsigned char * generateParityBit(unsigned long num) {
    /* INITIALIZE COUNTER */
    int count = 0;

    /* COUNT VON COUNT */
    for (int i = 0; i < size; i++)
        if (((num >> i) & 0x01) == 1)
            count++;
    
    /* SET PARITY BIT IF NEEDED */
    if ((count % 2) == 1)
        num |= 1 << (size - 1);

    /* CONVERT TO ARRAY OF BYTES - ASSUMING 32 BITS */
    const unsigned char bytes[] = {
        (num << 030) & 0xFF,
        (num << 020) & 0xFF,
        (num << 010) & 0xFF,
        (num << 000) & 0xFF
    };

    return bytes;
}
