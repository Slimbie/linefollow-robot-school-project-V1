#include "config.h"

int sensorData[8];
float afstand = 100.0; // Hier maken we hem aan
const int sensorPinnen[] = {32, 33, 34, 35, 36, 39, 25, 26};

void setupSensors() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void scanSensoren() {
    // IR
    for(int i = 0; i < 8; i++) {
        sensorData[i] = analogRead(sensorPinnen[i]);
    }
    // Ultrasoon (optioneel: gebruik NewPing library voor stabiliteit)
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH, 20000);
    if (duration > 0) afstand = duration * 0.034 / 2;
}

bool zieLijn() {
    for(int i = 0; i < 8; i++) {
        if(sensorData[i] > THRESHOLD) return true;
    }
    return false;
}