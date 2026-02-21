#include "config.h"

ESP32Encoder encL;
ESP32Encoder encR;
Node route[200];
int nodeCount = 0;
int huidigeNode = 0; // Hier maken we hem aan

// ONDERZOEKS-PID WAARDEN
const float KP_LOW = 2.5, KI_LOW = 0.0, KD_LOW = 1.2; 
const float KP_HIGH = 4.0, KI_HIGH = 0.05, KD_HIGH = 3.5; 
const int SPEED_LOW = 120;
const int SPEED_HIGH = 250;

float Kp = 2.5, Ki = 0.0, Kd = 1.2;
int basisSnelheid = 120;

void berekenDynamischePid(int huidigeSnelheid) {
    float ratio = (float)(huidigeSnelheid - SPEED_LOW) / (SPEED_HIGH - SPEED_LOW);
    ratio = constrain(ratio, 0.0, 1.0);
    Kp = KP_LOW + (ratio * (KP_HIGH - KP_LOW));
    Ki = KI_LOW + (ratio * (KI_HIGH - KI_LOW));
    Kd = KD_LOW + (ratio * (KD_HIGH - KD_LOW));
}

float berekenPID(float &error, float &vorige_error, float &integraal, int snelheid) {
    berekenDynamischePid(snelheid);
    int s[8];
    int totaal = 0;
    for(int i=0; i<8; i++) {
        s[i] = (sensorData[i] > THRESHOLD) ? 1 : 0;
        totaal += s[i];
    }
    if (totaal == 0) {
        error = (vorige_error > 0) ? 40 : -40;
        return (Kp * error); 
    }
    float teller = (s[0]*-40) + (s[1]*-30) + (s[2]*-20) + (s[3]*-10) + 
                   (s[4]*10) + (s[5]*20) + (s[6]*30) + (s[7]*40);
    error = teller / totaal;
    integraal = constrain(integraal + error, -200, 200);
    float afgeleide = error - vorige_error;
    float correctie = (Kp * error) + (Ki * integraal) + (Kd * afgeleide);
    vorige_error = error;
    return correctie;
}

int bepaalDoelSnelheid() {
    // Kijk of we dichtbij het punt zijn waar we een bocht hebben opgeslagen
    long huidigePos = encL.getCount();
    
    // Als we voorbij het punt van de huidige node zijn, ga naar de volgende
    if (huidigeNode < nodeCount && huidigePos > route[huidigeNode].encoderPos) {
        huidigeNode++;
    }

    // Kijk wat het type is van de KOMENDE node (bijv. 200 ticks vooruit)
    for (int i = huidigeNode; i < nodeCount; i++) {
        if (route[i].encoderPos - huidigePos < 500) { // Anticipeer 500 ticks vooruit
            if (route[i].type == TURN90) return 100; // Rem af voor scherpe bocht
            if (route[i].type == CURVE) return 180;  // Iets sneller in flauwe bocht
        }
    }
    return 250; // Rechtdoor? Vol gas!
}

void registreerNode(NodeType t, int intens) {
    if (nodeCount < 200) {
        route[nodeCount].encoderPos = encL.getCount();
        route[nodeCount].type = t;
        route[nodeCount].intensiteit = intens;
        nodeCount++;
    }
}

void registreerObstakel(long startPos, long afstandL, long afstandR) {
    if (nodeCount < 200) {
        route[nodeCount].encoderPos = startPos;
        route[nodeCount].type = OBSTACLE;
        route[nodeCount].boogAfstandL = afstandL;
        route[nodeCount].boogAfstandR = afstandR;
        nodeCount++;
    }
}