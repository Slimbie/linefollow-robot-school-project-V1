#include "config.h"

ESP32Encoder encL;
ESP32Encoder encR;
Node route[200]; // aanname: max 200 segmenten in het parcours, pas aan indien nodig
int nodeCount = 0;
int huidigeNode = 0; // Hier maken we hem aan

// ONDERZOEKS-PID WAARDEN
//const float KP_LOW = 2.5, KI_LOW = 0.0, KD_LOW = 1.2; 
//const float KP_HIGH = 4.0, KI_HIGH = 0.05, KD_HIGH = 3.5; 
const int SPEED_LOW = 90;
const int SNELHEID_MAPPING = 100;
const int SPEED_HIGH = 180;//250 is max, maar is te veel volt voor deze motors dus limieteer tot 180 voor 6v 


float Kp = 8, Ki = 0.01, Kd = 1.2;
int basisSnelheid = 100;
// Globale variabelen voor communicatie met mapping.cpp
int sBinair[8];
int aantalGroepen = 0;
//dit heb ik even in commentaar gezet omdat ik niet zeker ben of dit wel een ding is ? denk van niet
/*
void berekenDynamischePid(int huidigeSnelheid) {
    float ratio = (float)(huidigeSnelheid - SPEED_LOW) / (SPEED_HIGH - SPEED_LOW);
    ratio = constrain(ratio, 0.0, 1.0);
    Kp = KP_LOW + (ratio * (KP_HIGH - KP_LOW));
    Ki = KI_LOW + (ratio * (KI_HIGH - KI_LOW));
    Kd = KD_LOW + (ratio * (KD_HIGH - KD_LOW));
}
*/


// --- NIEUWE FUNCTIE: FILTERT DE LIJN BIJ KRUISPUNTEN ---
int verwerkSensorenEnFilterLijn() {
    int totaalZwart = 0;
    aantalGroepen = 0;
    bool inGroep = false;

    // 1. Digitaliseren en groepen tellen (Gat-detectie)
    for(int i=0; i<8; i++) {
        sBinair[i] = (sensorData[i] > THRESHOLD) ? 1 : 0;
        totaalZwart += sBinair[i];

        if (sBinair[i] == 1 && !inGroep) {
            aantalGroepen++;
            inGroep = true;
        } else if (sBinair[i] == 0) {
            inGroep = false;
        }
    }

    // 2. LIJN-FILTER: Als er gaten zijn (kruispunt), negeer de uiterste sensoren voor de PID
    // We doen dit alleen als we niet op het finishvlak (alles zwart) zitten
    if (aantalGroepen > 1 && totaalZwart < 7) {
        sBinair[0] = 0; sBinair[1] = 0;
        sBinair[6] = 0; sBinair[7] = 0;
    }
    
    return totaalZwart;
}

/* void berekenDynamischePid(int huidigeSnelheid) {
    Kp = KP_HIGH;// kp wordt gelijk gesteld naar kp_high
    Ki = KI_HIGH;
    Kd = KD_HIGH;// moet mis nul zijn omdat we anders te veel corrigeren bij hoge snelheden, maar dit zijn de waarden die ik het beste vond tijdens het testen
}
*/

//oude pid berekenaar houd geen rekening met corner fouten!
/*
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
}*/
float berekenPID(float &error, float &vorige_error, float &integraal, int snelheid) {
    
    // Gebruik de nieuwe filter functie
    int totaalZwart = verwerkSensorenEnFilterLijn();

    if (totaalZwart == 0) {
        error = (vorige_error > 0) ? 40 : -40;
        return (Kp * error); 
    }

    // Berekening op basis van de (gefilterde) sBinair
    float teller = (sBinair[0]*-40) + (sBinair[1]*-30) + (sBinair[2]*-20) + (sBinair[3]*-10) + 
                   (sBinair[4]*10) + (sBinair[5]*20) + (sBinair[6]*30) + (sBinair[7]*40);
    
    // Tel hoeveel 1-tjes er overblijven na filtering voor een correct gemiddelde
    int actiefZwart = 0;
    for(int i=0; i<8; i++) actiefZwart += sBinair[i];
    
    if (actiefZwart > 0) {
        error = teller / actiefZwart;
    }

    integraal = constrain(integraal + error, -200, 200);
    float afgeleide = error - vorige_error;
    float correctie = (Kp * error) + (Ki * integraal) + (Kd * afgeleide);
    vorige_error = error;
    
    return correctie;
}

int bepaalDoelSnelheid() {
    // Gebruik gemiddelde encoderpositie (beide wielen) voor stabieler afstanden
    long huidigePos = (ENC_LEFT_COUNT() + ENC_RIGHT_COUNT()) / 2;
    
    // Als we voorbij het punt van de huidige node zijn, ga naar de volgende
    if (huidigeNode < nodeCount && huidigePos > route[huidigeNode].encoderPos) {
        huidigeNode++;
    }

    // Kijk wat het type is van de KOMENDE node (bijv. 200 ticks vooruit)
    for (int i = huidigeNode; i < nodeCount; i++) {
        if (route[i].encoderPos - huidigePos < 1200) { // Anticipeer 1200 ticks vooruit (~1 rotatie)
            if (route[i].type == TURN90) return SPEED_HIGH-90; // Rem af voor scherpe bocht
            if (route[i].type == CROSSING) return SPEED_HIGH-60; // Iets minder remmen voor kruispunt
            if (route[i].type == ZIGZAG) return SPEED_HIGH-50; // Rem iets voor zigzag, maar niet te veel want we moeten snel corrigeren
            if (route[i].type == OBSTACLE) return SPEED_HIGH-80; // Rem flink af voor obstakel
            if (route[i].type == FINISH) return SPEED_LOW; // Bereid je voor op de finish, rem flink af
        }
    }
    return SPEED_HIGH; // Rechtdoor? Vol gas!
}

// slaagt positie op van hindernis
void registreerNode(NodeType t, int intens) {
    if (nodeCount < 200) {
        long pos = (encL.getCount() + encR.getCount()) / 2;
        route[nodeCount].encoderPos = pos;
        route[nodeCount].type = t;

        // Als er geen expliciete intensiteit opgegeven is, sla de afstand vanaf de vorige node op.
        if (intens == 0 && nodeCount > 0) {
            route[nodeCount].intensiteit = pos - route[nodeCount - 1].encoderPos;
        } else {
            route[nodeCount].intensiteit = intens;
        }

        nodeCount++;
    }
}

//slaagt positie en boogafstand op van obstakel, zodat we deze later kunnen naspelen in de speedrun
void registreerObstakel(long startPos, long afstandL, long afstandR) {
    if (nodeCount < 200) {
        route[nodeCount].encoderPos = startPos;
        route[nodeCount].type = OBSTACLE;
        route[nodeCount].boogAfstandL = afstandL;
        route[nodeCount].boogAfstandR = afstandR;
        nodeCount++;
    }
}