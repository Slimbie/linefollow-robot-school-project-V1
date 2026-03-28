#include "config.h"

ESP32Encoder encL;
ESP32Encoder encR;
Node route[200]; // aanname: max 200 segmenten in het parcours, pas aan indien nodig
int nodeCount = 0;
int huidigeNode = 0; // Hier maken we hem aan

// ONDERZOEKS-PID WAARDEN
//const float KP_LOW = 2.5, KI_LOW = 0.0, KD_LOW = 1.2; 
//const float KP_HIGH = 4.0, KI_HIGH = 0.05, KD_HIGH = 3.5; 
const int SPEED_LOW = 100;
const int SNELHEID_MAPPING = 60;
const int SPEED_HIGH = 250;// verhoogd naar max voor snellere speedrun 


// PID-tuning voor stabiel gedrag: minder overshooting, rustiger terugdraaien
float Kp = 2.3, Ki = 0.00, Kd = 16.0;  // Was: Kp=0.8, Ki=0.008, Kd=14.0 -> Nu: rustiger + meer demping
int basisSnelheid = 100; 
// Globale variabelen voor communicatie met mapping.cpp
int sBinair[8];
int aantalGroepen = 0;



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


float berekenPID(float &error, float &vorige_error, float &integraal, int snelheid) {
    
    // Gebruik de nieuwe filter functie
    int totaalZwart = verwerkSensorenEnFilterLijn();

    if (totaalZwart == 0) {
        // Bij lijnverlies: milde correctie om bochten te volgen zonder te hard te reageren
        error = vorige_error * 0.5;  // Halveer vorige error voor rustige drift
        
        // Integraal langzaam afbouwen om wind-up te voorkomen
        integraal = constrain(integraal * 0.8, -100, 100);
        
        float afgeleide = error - vorige_error;
        float correctie = (Kp * error) + (Ki * integraal) + (Kd * afgeleide);
        vorige_error = error;
        return correctie;
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
            if (route[i].type == CROSSING) return SPEED_HIGH; // Geen remming voor T-splitsingen rechtdoor
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
