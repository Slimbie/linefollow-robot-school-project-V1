#include <Arduino.h>
#include "config.h"
#include <ESP32Encoder.h>

State currentState = STANDBY;
unsigned long stateTimer = 0;
bool isGestart = false;

State previousState = STANDBY;
long startBoogL = 0, startBoogR = 0;

// Verwijs naar de encoders die in logic.cpp echt bestaan
extern ESP32Encoder encL, encR;
float error = 0, vorige_error = 0, integraal = 0;

void SensorTask(void * pvParameters) {
    for(;;) { scanSensoren(); vTaskDelay(2 / portTICK_PERIOD_MS); }
}

void printRouteData() {
    Serial.println("\n========= ROUTE DATA DUMP =========");
    Serial.printf("Totaal aantal segmenten: %d\n", nodeCount);
    for (int i = 0; i < nodeCount; i++) {
        const char* typeStr = "ONBEKEND";
        if(route[i].type == STRAIGHT) typeStr = "RECHT";
        else if(route[i].type == TURN90) typeStr = "BOCHT 90";
        else if(route[i].type == OBSTACLE) typeStr = "OBSTAKEL";
        else if(route[i].type == FINISH) typeStr = "FINISH";
        
        Serial.printf("[%d] Pos: %ld | Type: %s | Intens: %d\n", i, route[i].encoderPos, typeStr, route[i].intensiteit);
    }
    Serial.println("===================================\n");
}

void setup() {
    Serial.begin(115200);
    setupHardware();
    
    pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);

    ESP32Encoder::useInternalWeakPullResistors = UP;
    encL.attachFullQuad(12, 13);
    encR.attachFullQuad(15, 2);

    xTaskCreatePinnedToCore(SensorTask, "Sensors", 4000, NULL, 1, NULL, 0);
    Serial.println("Systeem Online. Wacht op startknop...");
}

void loop() {
    static unsigned long lastBtn = 0;
    
    if (millis() - lastBtn > 500 && digitalRead(START_BUTTON_PIN) == LOW) {
        isGestart = !isGestart;
        lastBtn = millis();
        
        if (isGestart) {
            Serial.println("Startknop gedetecteerd! Start over 3s...");
            stateTimer = millis();
            currentState = START_DELAY;
        } else {
            Serial.println("Stopknop gedetecteerd! Motoren uit.");
            motorSturing(0, 0);
            printRouteData(); 
            currentState = STANDBY;
        }
    }

     if (!isGestart) return;

    static unsigned long lastLogic = 0;
    if (millis() - lastLogic >= 10) {
        lastLogic = millis();
        float correctie = 0; // Eén keer definiëren voor alle cases

        // 1. GLOBALE CHECK VOOR OBSTAKEL (Tijdens rijden)
        if ((currentState == MAPPING || currentState == SPEEDRUN) && afstand < 10.0) {
            previousState = currentState; 
            currentState = OBSTACLE_MANOEUVRE;
            startBoogL = encL.getCount();
            startBoogR = encR.getCount();
            Serial.println("Obstakel gedetecteerd! Uitwijken...");
        }

        switch (currentState) {
            case START_DELAY:
                if (millis() - stateTimer > 3000) {
                    currentState = SEARCH_LINE;
                }
                break;

            case SEARCH_LINE:
                motorSturing(100, 100);
                if (zieLijn()) {
                    currentState = (digitalRead(MODE_SWITCH_PIN) == LOW) ? SPEEDRUN : MAPPING;
                    encL.clearCount(); encR.clearCount();
                    nodeCount = 0;
                    huidigeNode = 0; // Reset ook de node-index voor de speedrun
                }
                break;

            case MAPPING:
                // PID Berekening (De robot blijft op de lijn)
                correctie = berekenPID(error, vorige_error, integraal, SNELHEID_MAPPING);
                motorSturing(SNELHEID_MAPPING + (int)correctie, SNELHEID_MAPPING - (int)correctie);
                analyseerParcours(); // Check continu op nieuwe nodes
                break;
            

            case SPEEDRUN:
                { // Accolades nodig omdat we 'doelSnelheid' hier lokaal aanmaken
                    int doelSnelheid = bepaalDoelSnelheid(); 
                    correctie = berekenPID(error, vorige_error, integraal, doelSnelheid);
                    motorSturing(doelSnelheid + (int)correctie, doelSnelheid - (int)correctie);
                }
                break;

            case OBSTACLE_MANOEUVRE:
                if (previousState == MAPPING) {
                    motorSturing(180, 80); // De boog die je leert
                    if (abs(encL.getCount() - startBoogL) > 500 && zieLijn()) {
                        registreerObstakel(startBoogL, encL.getCount() - startBoogL, encR.getCount() - startBoogR);
                        currentState = MAPPING;
                    }
                } else {
                    // In Speedrun: we gebruiken de data van de huidige node
                    long doelL = route[huidigeNode].boogAfstandL;
                    motorSturing(220, 100); // Sneller naspelen
                    if (abs(encL.getCount() - startBoogL) >= doelL) {
                        currentState = SPEEDRUN;
                        huidigeNode++; // Ga naar de volgende herinnering
                    }
                }
                break;

            default: break;
        }
    }
}