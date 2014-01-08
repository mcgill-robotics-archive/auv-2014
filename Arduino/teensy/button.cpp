const int pin = 0;

void setup() {
    /* SET UP SERIAL */
    while(!Serial);

    /* SET INPUT PINS */
    pinMode(pin, INPUT);

    /* ATTACH INTERRUPTS */
    attachInterrupt(pin, button, CHANGE);
}

void loop() {

}

void button() {
    Serial.println(micros());
}
