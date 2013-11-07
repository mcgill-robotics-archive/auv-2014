/**
 *  
 *  ########   THREE-DIMENSIONAL SOUND SOURCE LOCATION SYSTEM   ########
 *
 *  @author     Anass Al-Wohoush, Gabriel Cormier-Affleck & Igor Sadikov
 *  @version    0.1 BETA
 *  
 *  This Arduino sketch estimates the orientation of a sound source in
 *  3-dimensional space using the time distance between an array of 4 
 *  microphones. It then rotates a laser-mounted servo toward the sound.
 *
 *  NOTE:
 *  This has not been tested yet.
 *  
 *  Contact original authors for more information on the math, physics
 *  and difficulties of the matter.
 *
 */

/* INCLUDES */
#include <Servo.h>
#include <math.h>

/* NUMBER OF MICROPHONES */
#define NMICS       4
/* MAXIMUM DELAY BEFORE RESET (TICKS) */
#define MAX_DELAY   10000
/* VOLUME THRESHOLD (OUT OF 330) - EXPERIMENTAL VALUE */
#define THRESHOLD   50
/* TIME PER LOOP CYCLE (MICROSECONDS) - EXPERIMENTAL VALUE */
#define TICK_TIME   40
/* DISTANCE BETWEEN MICROPHONES (MILLIMETERS) */
#define DISTANCE    1.0e3
/* SPEED OF SOUND IN MEDIUM (METERS PER SECOND) */
#define SPEED       343

/* MICROPHONE AND SERVO PINS */
const int pins[NMICS] = { A0, A1, A2, A3 };
const int pin_yaw = 9;
const int pin_pitch = 10;

/* SERVOS */
Servo servo_yaw;
Servo servo_pitch;

/* OVERCLOCKING: PS_08 CORRESPONDS TO 2 MHZ */
const unsigned char PS_08 = (1 << ADPS1) | (1 << ADPS0);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

/* FOR EASIER PRINTING TO SERIAL (BASED ON A POST ON ARDUINO FORUM) */
const char endl = '\n';
template<class T> inline Print &operator <<(Print &obj, T arg) {
    obj.print(arg); return obj;
}

struct mic {
    int value;              // CURRENT VALUE
    int peak;               // PEAK VALUE WITHIN MAX_DELAY
    long time;              // TICKS SINCE PEAK
    bool done;              // TRUE IF MAX_DELAY HAS PASSED
    void reset();
} mics[NMICS];

void mic::reset() {
    this->value = 0;
    this->peak = 0;
    this->time = 0;
    this->done = false;
}

void setup() {
    Serial.begin(115200);

    /* OVERCLOCKING */
    ADCSRA &= ~PS_128;      // REMOVE PRESET BITS
    ADCSRA |= PS_08;        // SET TO SELECTED CLOCK RATE

    /* SERVOS */
    servo_yaw.attach(pin_yaw);
    servo_pitch.attach(pin_pitch);

    for (int i = 0; i < NMICS; i++) {
        pinMode(pins[i], INPUT);
        mics[i].reset();    // INITIALIZE
    }
}

void loop() {
    /* READ VALUES */
    for (int v, i = 0; i < NMICS; i++) {
        v = analogRead(pins[i]) - 330;
        // WARNING: abs is presumably a macro
        mics[i].value = abs(v);
    }

    /* INCREMENT TIMER AND CHECK FOR PEAKS */
    for (int i = 0; i < NMICS; i++) {
        if (mics[i].value > mics[i].peak) {
            // found peak: reset time
            mics[i].peak = mics[i].value;
            mics[i].time = 0;
        } else if (++mics[i].time > MAX_DELAY) {
            // max_delay has passed
            mics[i].done = true;
        }
    }

    /* CHECK IF ALL ARE DONE */
    for (int i = 0; i < NMICS; i++) {
        if (!mics[i].done) {
            // not done: keep listening
            return;
        }
    }

    /* CHECK IF ALL ARE ABOVE THRESHOLD */
    bool all = true;
    for (int i = 0; i < NMICS; i++) {
        if (mics[i].peak < THRESHOLD) {
            all = false; break;
        }
    }

    /* PERFORM CALCULATIONS */
    if (all) {
        double d = DISTANCE;

        /* TIME DIFFERENCES IN MICROSECONDS */
        long dty = (mics[1].time - mics[0].time) * TICK_TIME;
        long dtx = (mics[2].time - mics[0].time) * TICK_TIME;
        long dtz = (mics[3].time - mics[0].time) * TICK_TIME;

        /* DISTANCE DIFFERENCES (ds = v * dt) IN MILLIMETERS */
        double sx = (double) (dtx * SPEED) / (double) 1000;
        double sy = (double) (dty * SPEED) / (double) 1000;
        double sz = (double) (dtz * SPEED) / (double) 1000;

        /* CALCULATE YAW */
        // NOTE: the following was generated and optimized using maple 
        double t1 = d * d; double t3 = t1 * sy;
        double t5 = sy * sy; double t7 = sx * sx; double t8 = t7 * t5; double t9 = sy * t5; double t11 = t1 * t1;
        double t16 = t7 * t7; double t21 = t5 * t5; double t27 = sx * t7; double t35 = sx * t16; double t39 = t7 * t16;
        double t44 = 2 * t7 * t1 * t11 - 3 * t16 * t11 - 3 * t8 * t11 + t21 * t7 * t1 + 4 * t16 * t5 * t1 - 2 * t27 * t9 * t1 + 2 * t27 * sy * t11 - t16 * t21 - 2 * t35 * t1 * sy + t39 * t1 + 2 * t35 * t9 - t39 * t5;
        double t45 = sqrt(t44); double t46 = t3 * sx - t1 * t5 +  t8 - t9 * sx + t11 - t1 * t7 + t45;
        double t48 = t1 - t5 - t7;
        double yaw = atan((t1 * sx + sy * t46 / t48 - t3 + t7 * sy - sx * t5) / sx / t46 * t48);

        /* CALCULATE PITCH */
        // NOTE: the following was generated and optimized using maple 
        double t1 = d * d; double t3 = t1 * sz;
        double t5 = sz * sz; double t7 = sx * sx; double t8 = t7 * t5; double t9 = sz * t5; double t11 = t1 * t1;
        double t16 = t7 * t7; double t21 = t5 * t5; double t27 = sx * t7; double t35 = sx * t16; double t39 = t7 * t16;
        double t44 = 2 * t7 * t1 * t11 - 3 * t16 * t11 - 3 * t8 * t11 + t21 * t7 * t1 + 4 * t16 * t5 * t1 - 2 * t27 * t9 * t1 + 2 * t27 * sz * t11 - t16 * t21 - 2 * t35 * t1 * sz + t39 * t1 + 2 * t35 * t9 - t39 * t5;
        double t45 = sqrt(t44); double t46 = t3 * sx - t1 * t5 +  t8 - t9 * sx + t11 - t1 * t7 + t45;
        double t48 = t1 - t5 - t7;
        double pitch = atan((t1 * sx + sz * t46 / t48 - t3 + t7 * sz - sx * t5) / sx / t46 * t48);

        /* ROTATE SERVOS AND DISPLAY ANGLES SUCH THAT -90 < ANGLE < 90 */
        if (!isnan(yaw) && !isnan(pitch)) {
            yaw *= 180 / 3.1415926535;
            servo_yaw.write(yaw + 90);
            pitch *= 180 / 3.1415926535;
            servo_pitch.write(pitch + 90);
            Serial << yaw + ", " + pitch << endl;
            delay(500);
        } else
            // ignoring degenerate triangles from faulty data
            Serial << "NaN" << endl;
    }

    /* RESET */
    for (int i = 0; i < NMICS; i++) {
        mics[i].reset();
    }
}
