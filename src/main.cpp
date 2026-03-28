#include <Arduino.h>
#include "config.h"
#include <ESP32Encoder.h>

State currentState = STANDBY;
unsigned long stateTimer = 0;
bool isGestart = false;
bool debugPrint = false;

State previousState = STANDBY;
long startBoogL = 0, startBoogR = 0;

bool finishDetected = false;

// Verwijs naar de encoders die in logic.cpp echt bestaan
extern ESP32Encoder encL, encR;
float error = 0, vorige_error = 0, integraal = 0;

// Speedrun-meetdata (voor tweedelijns optimalisatie)
long speedrunNodeDelta[200] = {0};
int speedrunTrial = 0;
bool speedrunLogging = false;

bool isPCConnected() {
    // 1) Optionele hardware pin
    bool pinConnected = (digitalRead(PC_DEBUG_MODE_PIN) == LOW);

    // 2) Detect seriële activiteit binnen 1000ms
    static unsigned long lastSerialActivity = 0;
    static bool debugMode = false;  // Persistent debug mode
    if (Serial.available() > 0) {
        char c = Serial.read();
        lastSerialActivity = millis();

        // commando's voor debug toggle
        if (c == 'd' || c == 'D') {
            debugMode = true;
            Serial.println("DEBUG ON via commando");
        } else if (c == 'b' || c == 'B') {
            debugMode = false;
            Serial.println("DEBUG OFF via commando");
        }
    }

    bool serialConnected = (millis() - lastSerialActivity < 1000) || debugMode;

    return pinConnected || serialConnected;
}

void resetPidState() {
    error = 0;
    vorige_error = 0;
    integraal = 0;
}

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST 0

int tempo = 120;
int melody[] = {
  NOTE_A4,-4, NOTE_A4,-4, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_F4,8, REST,8,
  NOTE_A4,-4, NOTE_A4,-4, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_A4,16, NOTE_F4,8, REST,8,
  NOTE_A4,4, NOTE_A4,4, NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16,
  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,
  NOTE_E5,4, NOTE_E5,4, NOTE_E5,4, NOTE_F5,-8, NOTE_C5,16,
  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,
  NOTE_A5,4, NOTE_A4,-8, NOTE_A4,16, NOTE_A5,4, NOTE_GS5,-8, NOTE_G5,16,
  NOTE_DS5,16, NOTE_D5,16, NOTE_DS5,8, REST,8, NOTE_A4,8, NOTE_DS5,4, NOTE_D5,-8, NOTE_CS5,16,
  NOTE_C5,16, NOTE_B4,16, NOTE_C5,16, REST,8, NOTE_F4,8, NOTE_GS4,4, NOTE_F4,-8, NOTE_A4,-16,
  NOTE_C5,4, NOTE_A4,-8, NOTE_C5,16, NOTE_E5,2,
  NOTE_A5,4, NOTE_A4,-8, NOTE_A4,16, NOTE_A5,4, NOTE_GS5,-8, NOTE_G5,16,
  NOTE_DS5,16, NOTE_D5,16, NOTE_DS5,8, REST,8, NOTE_A4,8, NOTE_DS5,4, NOTE_D5,-8, NOTE_CS5,16,
  NOTE_C5,16, NOTE_B4,16, NOTE_C5,16, REST,8, NOTE_F4,8, NOTE_GS4,4, NOTE_F4,-8, NOTE_A4,-16,
  NOTE_A4,4, NOTE_F4,-8, NOTE_C5,16, NOTE_A4,2,
};
int notes = sizeof(melody) / sizeof(melody[0]) / 2;
int wholenote = (60000 * 4) / tempo;

static int buzzerIndex = 0;
static unsigned long buzzerStartTime = 0;
static unsigned long buzzerNoteEnd = 0;
static bool buzzerEnabled = false;
static bool buzzerNotePlaying = false;

void SensorTask(void * pvParameters) {
    for(;;) { scanSensoren(); vTaskDelay(2 / portTICK_PERIOD_MS); }
}

void printRouteData() {
    Serial.println("\n========= ROUTE DATA DUMP =========");
    Serial.printf("Totaal aantal segmenten: %d\n", nodeCount);
    Serial.printf("(instel: 58 ticks/cm, 1 rotatie ~850 ticks)\n");
    for (int i = 0; i < nodeCount; i++) {
        const char* typeStr = "ONBEKEND";
        if(route[i].type == STRAIGHT) typeStr = "RECHT";
        else if(route[i].type == TURN90) typeStr = "BOCHT 90";
        else if(route[i].type == OBSTACLE) typeStr = "OBSTAKEL";
        else if(route[i].type == FINISH) typeStr = "FINISH";

        long delta = (i == 0) ? route[i].encoderPos : route[i].encoderPos - route[i-1].encoderPos;
        float delta_cm = delta / 58.0;

        Serial.printf("[%d] Pos:%ld | Delta:%ld (~%.1fcm) | Type:%s | Afstand:%ld | Intens:%d\n",
                      i, route[i].encoderPos, delta, delta_cm, typeStr, route[i].intensiteit, route[i].intensiteit);
    }
    Serial.println("===================================\n");
}

void setup() {
    Serial.begin(115200);
    setupHardware();

    // Buzzer setup (niet motor PWM kanalen 0/1 gebruiken, kanaal 2 vrij)
    pinMode(BUZZER_PIN, OUTPUT);
    ledcSetup(2, 2000, 8);     // kanaal 2, 2 kHz, 8 bit (met ledcWriteTone kan anders)
    ledcAttachPin(BUZZER_PIN, 2);

    pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);
    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(PC_DEBUG_MODE_PIN, INPUT_PULLUP);  // debug mode schakelaar (LOW=PC connected)

    ESP32Encoder::useInternalWeakPullResistors = UP;
    encL.attachFullQuad(12, 13);
    encR.attachFullQuad(15, 2);

    xTaskCreatePinnedToCore(SensorTask, "Sensors", 4000, NULL, 1, NULL, 0);
    Serial.println("Systeem Online. Wacht op startknop...");
}

void loop() {
    static unsigned long lastBtn = 0;
    static unsigned long lastLineSeen = 0;
    static bool deadEndManeuver = false;
    static long deadEndStartPos = 0;

    // Richtlijn: start pas met lijnscannen na minimaal 5cm (≈ 290 ticks) in SEARCH_LINE
    static long searchStartPos = 0;
    static bool searchLineActive = false;
    
    bool pcConnected = isPCConnected();

    if (millis() - lastBtn > 500 && digitalRead(START_BUTTON_PIN) == LOW) {
        lastBtn = millis();
        isGestart = !isGestart;

        if (isGestart) {
            if (pcConnected) Serial.println("Startknop gedetecteerd! Start over 8s met Imperial March...");
            resetPidState();
            finishDetected = false;
            // start in-running debug only met PC contact
            if (pcConnected) debugPrint = true;
            currentState = START_DELAY;
        } else {
            if (pcConnected) Serial.println("Stopknop gedetecteerd! Motoren uit.");
            resetPidState();
            motorSturing(0, 0);
            ledcWriteTone(2, 0);
            buzzerEnabled = false;
            if (pcConnected) {
                debugPrint = true;
                printRouteData();
            } else {
                debugPrint = false;
            }
            currentState = STANDBY;
        }
    }

    // Bij battery-run NIET spammen over serial (draadloos / offline)
    // debugPrint wordt nu persistent beheerd via commando's

    if (!isGestart) return;

    static unsigned long lastLogic = 0;
    static long lastLinePos = 0;
    static int lastLineDir = 1; // 1 = lijn rechts, -1 = lijn links
    static const long DEAD_END_TICKS = 58 * 10; // ~10 cm (58 ticks/cm)
    static const unsigned long DEAD_END_TIMEOUT_MS = 400;

    if (millis() - lastLogic >= 10) {
        lastLogic = millis();
        float correctie = 0; // Eén keer definiëren voor alle cases

        long huidigePos = (ENC_LEFT_COUNT() + ENC_RIGHT_COUNT()) / 2;
        bool lijn = zieLijn();

        if (lijn) {
            lastLineSeen = millis();
            lastLinePos = huidigePos;
            lastLineDir = (error >= 0) ? 1 : -1; // gebruik laatste PID-fout om richting te bepalen
            deadEndManeuver = false;
        }

        // Geen harde doodlopende-pad detectie meer, PID verwerkt lijnverlies opgewarmd.
        // (deadEndManeuver blijft false zodat we niet in omkeer-cirkel belanden)

        // 1. GLOBALE CHECK VOOR OBSTAKEL (Tijdens rijden)
        // Alleen reageren als afstand tussen 5cm en 20cm ligt (0 = geen meting)
        if ((currentState == MAPPING || currentState == SPEEDRUN) && afstand > 5.0 && afstand < 20.0) {
            previousState = currentState; 
            currentState = OBSTACLE_MANOEUVRE;
            startBoogL = ENC_LEFT_COUNT();
            startBoogR = ENC_RIGHT_COUNT();
            if (debugPrint) Serial.println("Obstakel gedetecteerd! Uitwijken...");
        }

        switch (currentState) {
            case START_DELAY:
                // Start delay verwijderd (geen muziek meer). Direct naar SEARCH_LINE.
                currentState = SEARCH_LINE;
                break;

            case SEARCH_LINE: {
                long huidigePos = (ENC_LEFT_COUNT() + ENC_RIGHT_COUNT()) / 2;

                // Bij instappen in SEARCH_LINE initialiseren
                if (!searchLineActive) {
                    searchStartPos = huidigePos;
                    searchLineActive = true;
                    if (debugPrint) Serial.println("SEARCH_LINE: beginfase, minimum afstandscans activeren");
                }

                // Rij vooruit totdat we minimaal 5cm hebben afgelegd
                motorSturing(100, 100);
                if (!searchLineActive || abs(huidigePos - searchStartPos) < 290) {
                    // nog niet ver genoeg voor lijnvalidatie
                    break;
                }

                // pas nu echte lijndetectie
                if (zieLijn()) {
                    lastLineSeen = millis();
                    deadEndManeuver = false;
                    bool modeSpeedrun = (digitalRead(MODE_SWITCH_PIN) == LOW);
                    currentState = modeSpeedrun ? SPEEDRUN : MAPPING;
                    encL.clearCount(); encR.clearCount();
                    nodeCount = 0;
                    huidigeNode = 0; // Reset ook de node-index voor de speedrun

                    if (debugPrint) Serial.printf("SEARCH_LINE: lijn gevonden, mode=%s\n", modeSpeedrun ? "SPEEDRUN" : "MAPPING");

                    if (currentState == SPEEDRUN) {
                        speedrunTrial++;
                        speedrunLogging = true;
                        for (int i = 0; i < 200; i++) speedrunNodeDelta[i] = 0;
                        if (debugPrint) Serial.printf("Speedrun %d gestart (logging aan)\n", speedrunTrial);
                    }
                }
                break;
            }

            case MAPPING: {
                if (finishDetected) {
                    motorSturing(0, 0);
                    if (debugPrint) Serial.println("Finish in mapping bereikt, wachten op lijn of stopknop.");
                    currentState = FINISH_WAIT;
                    break;
                }

                // deadEndManeuver uitgeschakeld: reageren direct via PID in plaats van omkeren

                // PID Berekening (De robot blijft op de lijn)
                correctie = berekenPID(error, vorige_error, integraal, SNELHEID_MAPPING);
                motorSturing(SNELHEID_MAPPING + (int)correctie, SNELHEID_MAPPING - (int)correctie);
                analyseerParcours(); // Check continu op nieuwe nodes
                break;
            }
            

            case SPEEDRUN:
                if (finishDetected) {
                    motorSturing(0, 0);
                    if (debugPrint) Serial.println("Finish in speedrun bereikt, wachten op lijn of stopknop.");
                    currentState = FINISH_WAIT;
                    break;
                }

                if (speedrunLogging && nodeCount == 0) {
                    if (debugPrint) Serial.println("Speedrun: geen gemapte nodes, terug naar MAPPING om eerst in kaart te brengen");
                    currentState = MAPPING;
                    break;
                }

                // deadEndManeuver uitgeschakeld in speedrun ook, PID neemt de lijn terug over

                { // Accolades nodig omdat we 'doelSnelheid' hier lokaal aanmaken
                    int prevNode = huidigeNode;
                    int doelSnelheid = bepaalDoelSnelheid(); 
                    long huidigePos = (ENC_LEFT_COUNT() + ENC_RIGHT_COUNT()) / 2;

                    // Log verschillen tussen verwacht en echt (voor optimalisatie in volgende runs)
                    if (speedrunLogging && prevNode < nodeCount && huidigeNode != prevNode) {
                        speedrunNodeDelta[prevNode] = huidigePos - route[prevNode].encoderPos;
                        if (debugPrint) Serial.printf("Speedrun: node %d delta=%ld\n", prevNode, speedrunNodeDelta[prevNode]);
                    }

                    correctie = berekenPID(error, vorige_error, integraal, doelSnelheid);
                    motorSturing(doelSnelheid + (int)correctie, doelSnelheid - (int)correctie);

                    // Als we voorbij het laatste node zijn, stop dan en toon de meetdata
                    // nodeCount>0 voorkomt onmiddellijke stop wanneer er nog geen map is gemaakt
                    if (speedrunLogging && nodeCount > 0 && huidigeNode >= nodeCount) {
                        if (debugPrint) Serial.println("Speedrun klaar! Meetdata:");
                        if (debugPrint) for (int i = 0; i < nodeCount; i++) {
                            long delta = (i==0) ? route[i].encoderPos : route[i].encoderPos - route[i-1].encoderPos;
                            float delta_cm = delta / 58.0;
                            Serial.printf("  node %d: plan=%ld delta=%ld (actueel+%ld) afstand~%.1fcm\n",
                                          i, route[i].encoderPos, speedrunNodeDelta[i], delta, delta_cm);
                        }
                        speedrunLogging = false;
                        motorSturing(0, 0);
                        currentState = STANDBY;
                    }
                }
                break;
                case OBSTACLE_MANOEUVRE: {
    static int subState = 0; // 0=stop, 1=uitwijken, 2=lijn zoeken
    static unsigned long subTimer = 0;

    if (previousState == MAPPING) {
        // --- MAPPING MODE: Leren hoe we eromheen rijden ---
        if (subState == 0) { 
            motorSturing(0, 0); // Eerst even stoppen!
            if (millis() - subTimer > 300) { subState = 1; subTimer = millis(); }
        } 
        else if (subState == 1) {
            motorSturing(200, 60); // Een scherpere boog om het object heen
            // We rijden minimaal 600ms voordat we überhaupt naar de lijn gaan zoeken
            if (millis() - subTimer > 600) { subState = 2; }
        } 
        else if (subState == 2) {
            motorSturing(100, 180); // Terug naar de lijn draaien (bocht naar links)
            if (zieLijn()) {
                // Sla op hoe ver we gereden hebben voor de speedrun
                registreerObstakel(startBoogL, ENC_LEFT_COUNT() - startBoogL, ENC_RIGHT_COUNT() - startBoogR);
                currentState = MAPPING;
                subState = 0; // Reset voor de volgende keer
                if (debugPrint) Serial.println("Obstakel gepasseerd, lijn gevonden!");
            }
        }
    } else {
        // --- SPEEDRUN MODE: De opgeslagen boog herhalen ---
        long doelL = route[huidigeNode].boogAfstandL;
        motorSturing(220, 100); 
        if (abs(ENC_LEFT_COUNT() - startBoogL) >= doelL) {
            currentState = SPEEDRUN;
            huidigeNode++;
        }
    }
    break;
}

            case FINISH_WAIT:
                motorSturing(0, 0);
                if (zieLijn()) {
                    bool modeSpeedrun = (digitalRead(MODE_SWITCH_PIN) == LOW);
                    currentState = modeSpeedrun ? SPEEDRUN : MAPPING;
                    if (debugPrint) Serial.println("Lijn gezien na finish, doorgaan.");
                }
                break;

            default: break;
        }

        // Visuele feedback tijdens testen (PC-mode, 100ms)
        static unsigned long lastFeedback = 0;
        if (millis() - lastFeedback > 100 && debugPrint) {
            lastFeedback = millis();

            // Modus + state
            const char *stateName = currentState == STANDBY ? "STANDBY" :
                                    currentState == START_DELAY ? "START_DELAY" :
                                    currentState == SEARCH_LINE ? "SEARCH_LINE" :
                                    currentState == MAPPING ? "MAPPING" :
                                    currentState == SPEEDRUN ? "SPEEDRUN" :
                                    currentState == OBSTACLE_MANOEUVRE ? "OBSTACLE" :
                                    currentState == FINISH_WAIT ? "FINISH_WAIT" : "UNKNOWN";
            const char *modeName = digitalRead(MODE_SWITCH_PIN) == LOW ? "SPEEDRUN" : "MAPPING";

            // Sensor visualisatie: X=zwart/line, .=wit
            char s[10];
            for (int i = 0; i < 8; i++) {
                s[i] = sensorData[i] ? 'X' : '.';
            }
            s[8] = 0;

            Serial.printf("[%s | %s] finish=%d | line=%s | IR=%s", stateName, modeName,
                          finishDetected ? 1 : 0,
                          zieLijn() ? "JA" : "NEE",
                          s);
            Serial.println();
        }
    }
}