#include <Arduino.h>
#include "config.h"
#include <ESP32Encoder.h>

// Verwijs naar de encoders die in logic.cpp staan
extern ESP32Encoder encL, encR;
extern float Kp, Ki, Kd;
extern int basisSnelheid;

float pid_error = 0, vorige_pid_error = 0, integraal_pid = 0;
bool tuningActief = false;
int testMode = 1; 
bool simulatieMode = false;
int simFout = 0; 
bool testing = false; 

void SensorTask(void * pvParameters) {
    for(;;) { scanSensoren(); vTaskDelay(2 / portTICK_PERIOD_MS); }
}

void setup() {
    Serial.begin(115200);
    setupHardware();
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    
    // De encoders worden hier geconfigureerd
    ESP32Encoder::useInternalWeakPullResistors = UP;
    encL.attachFullQuad(12, 13);
    encR.attachFullQuad(15, 2);
    
    xTaskCreatePinnedToCore(SensorTask, "Sensors", 4000, NULL, 1, NULL, 0);

    Serial.println("\n*** PID TUNING TOOL GEREED ***");
    Serial.println("Commando's:");
    Serial.println("  pX.XX  - Zet Kp waarde");
    Serial.println("  iX.XXX - Zet Ki waarde");
    Serial.println("  dX.XX  - Zet Kd waarde");
    Serial.println("  sXXX   - Zet snelheid (0-255)");
    Serial.println("  u      - Test 180 graden draai (check motor richting!)");
    Serial.println("  x      - Stop motoren");
    Serial.println("  sim[fout] - Simulatie mode met lijnfout (-50 tot +50)");
    Serial.println("Start knop = motoren aan/uit");
    Serial.println("================================================");
}

void verwerkSerial() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() == 0) return;

        Serial.printf(">>> ontvangen command: '%s'\n", input.c_str());

        // Simulatie modus (om droog te testen zonder sensoren)
        if (input.startsWith("sim") || input.startsWith("SIM")) {
            simulatieMode = true;
            simFout = input.substring(3).toInt();
            Serial.printf(">>> SIMULATIE: Lijnfout ingesteld op %d\n", simFout);
            return;
        }

        char cmd = tolower(input.charAt(0));
        float val = input.substring(1).toFloat();

        // PID en Snelheid aanpassen
        if (cmd == 'p') Kp = val;
        else if (cmd == 'i') Ki = val;
        else if (cmd == 'd') Kd = val;
        else if (cmd == 's') basisSnelheid = (int)val;
        
        // 180-graden draai test (check motor richtingen!)
        else if (cmd == 'u') {
            Serial.println(">>> ACTIE: Start 180 graden draai test...");
            Serial.println("Kijk naar de motoren - beide moeten naar DEZELFDE richting draaien!");
            testing = true;
            encL.clearCount();
            encR.clearCount();
            motorSturing(130, -130);
            delay(850); // Pas dit getal aan tot hij exact 180 graden is
            motorSturing(0,0);
            Serial.printf(">>> RESULTAAT LEFT: %ld ticks | RIGHT: %ld ticks\n", (long)ENC_LEFT_COUNT(), (long)ENC_RIGHT_COUNT());
            if (ENC_LEFT_COUNT() * ENC_RIGHT_COUNT() > 0) {
                Serial.println("✓ Motor richtingen zijn CORRECT!");
            } else {
                Serial.println("✗ WAARSCHUWING: Motor richtingen zijn tegengesteld!");
            }
            testing = false;
        }

        // Stop commando
        else if (cmd == 'x') {
            tuningActief = false;
            motorSturing(0,0);
            Serial.println("MOTOREN GESTOPT via X commando");
        }

        // Onbekend commando catch
        else if (cmd != 'p' && cmd != 'i' && cmd != 'd' && cmd != 's' && cmd != 'u') {
            Serial.println(">>> ONBEKEND COMMANDO (gebruik p/i/d/s/u/x/sim)");
        }

        // Huidae PID waarden tonen
        Serial.printf("UPDATE -> P:%.2f | I:%.3f | D:%.2f | Snelheid:%d\n", Kp, Ki, Kd, basisSnelheid);
    }
}


void loop() {
    verwerkSerial();

    static unsigned long lastBtn = 0;
    if (millis() - lastBtn > 500 && digitalRead(START_BUTTON_PIN) == LOW) {
        tuningActief = !tuningActief;
        lastBtn = millis();
        if(!tuningActief) {
            motorSturing(0,0);
            Serial.println("MOTOREN STOP");
        } else {
            Serial.println("MOTOREN START - PID tuning actief!");
        }
    }

    if (tuningActief && !testing) {
        static unsigned long lastLogic = 0;
        if (millis() - lastLogic >= 10) {
            lastLogic = millis();
            float corr;
            if (simulatieMode) {
                // Simulatie: zorg voor test van PID zonder echte sensoren
                corr = (Kp * simFout) + (Kd * (simFout - vorige_pid_error));
                vorige_pid_error = simFout;
            } else {
                // Echte lijndetectie via sensoren
                corr = berekenPID(pid_error, vorige_pid_error, integraal_pid, basisSnelheid);
            }
            motorSturing(basisSnelheid + (int)corr, basisSnelheid - (int)corr);
            
            // Print elke 500ms de huedia toestand
            if (millis() % 500 < 10) {
                Serial.printf("S:%d Err:%.1f Corr:%.1f L:%ld R:%ld\n", 
                    basisSnelheid, 
                    (simulatieMode ? (float)simFout : pid_error), 
                    corr,
                    ENC_LEFT_COUNT(),
                    ENC_RIGHT_COUNT());
            }
        }
    }
}
