#include "config.h"

extern int sensorData[8];
extern float error;
extern ESP32Encoder encL;

long laatsteRegistratiePos = 0;
long finishStartPos = 0; // Om te meten hoe lang we op zwart rijden

void analyseerParcours() {
    int s[8];
    int totaalZwart = 0;
    for(int i=0; i<8; i++) {
        s[i] = (sensorData[i] > THRESHOLD) ? 1 : 0;
        totaalZwart += s[i];
    }

    long huidigePos = encL.getCount();

    // --- VERBETERDE FINISH DETECTIE (Regel d: 40x40cm zwart) ---
    if (totaalZwart >= 7) {
        if (finishStartPos == 0) finishStartPos = huidigePos; // Start meting
        
        // Als we meer dan 600 ticks (ca. 10cm) op zwart rijden, is het de finish
        if (abs(huidigePos - finishStartPos) > 600) {
            registreerNode(FINISH);
            Serial.println("DETECTIE: Echte Finish bevestigd!");
        }
        return; // Tijdens zwart rijden doen we geen andere detecties
    } else {
        finishStartPos = 0; // Reset als we weer wit zien
    }

    // Debounce: voorkom dubbele registratie van bochten/kruispunten
    if (abs(huidigePos - laatsteRegistratiePos) < 400) return;

    // --- VERBETERDE KRUISPUNT DETECTIE (Regel i.i.4 & 5) ---
    // We kijken of de buitenste sensoren zwart zijn EN we niet aan het wiebelen zijn
    if ((s[0] == 1 || s[7] == 1) && totaalZwart >= 4) {
        registreerNode(CROSSING);
        laatsteRegistratiePos = huidigePos;
        Serial.println("DETECTIE: Kruispunt gevonden");
    }

    // --- SCHERPE 90 GRADEN BOCHT (Regel i.i.1) ---
    // Alleen als we niet een kruispunt zien (daarom 'else if')
    else if ((s[0] && s[1] && s[2]) || (s[5] && s[6] && s[7])) {
        registreerNode(TURN90);
        laatsteRegistratiePos = huidigePos;
        Serial.println("DETECTIE: 90 graden bocht");
    }

    // --- ZIGZAG DETECTIE ---
    static int wiebelTeller = 0;
    if (abs(error) > 38) { 
        wiebelTeller++;
        if (wiebelTeller > 20) { 
            registreerNode(ZIGZAG);
            wiebelTeller = 0;
            laatsteRegistratiePos = huidigePos;
        }
    } else {
        if (wiebelTeller > 0) wiebelTeller--;
    }
}