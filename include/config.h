#ifndef CONFIG_H
#define CONFIG_H  
#include <Arduino.h>
#include "driver/gpio.h" // Nodig voor GPIO_NUM types
#include <ESP32Encoder.h>

// --- PINNEN: INTERFACE ---
#define MODE_SWITCH_PIN 23
#define START_BUTTON_PIN 19
#define THRESHOLD 3000 // dit past de lees waarde aan van de ir sensors

// --- PINNEN: MOTOREN (OT2065 / L298N) ---
#define MOT_L_PWM 27
#define MOT_L_AIN1 GPIO_NUM_14
#define MOT_L_AIN2 GPIO_NUM_4
#define MOT_R_PWM 22
#define MOT_R_BIN1 GPIO_NUM_21
#define MOT_R_BIN2 GPIO_NUM_16

// --- PINNEN: ULTRASOON ---
#define TRIG_PIN 5
#define ECHO_PIN 18

// De verschillende statussen van de robot
enum State { STANDBY, START_DELAY, SEARCH_LINE, MAPPING, SPEEDRUN, OBSTACLE_MANOEUVRE };

// Zorg dat deze exact overeenkomen met je regelboek!
enum NodeType { 
    STRAIGHT, 
    TURN90, 
    CROSSING,   // Kruispunten & T-splitsing (i.i.4 & 5)
    ZIGZAG,     // Zaagtand & Golfjes (i.i.8 & 9)
    OBSTACLE,   // Witte cilinder (iii)
    FINISH      // Zwart stopvak (d)
};



struct Node {
    long encoderPos;
    NodeType type;
    int intensiteit;      // Voor bochtscherpte of zigzag
    long boogAfstandL;    // Opgeslagen boog voor obstakel
    long boogAfstandR;    // Opgeslagen boog voor obstakel
};

// --- EXTERNS (Beloftes aan andere bestanden) ---
extern ESP32Encoder encL, encR;
extern Node route[200];
extern int nodeCount;
extern int huidigeNode;
extern int sensorData[8];
extern float afstand;

extern float Kp, Ki, Kd;
extern const int SNELHEID_MAPPING;
extern const int SNELHEID_MAX;

// --- FUNCTIES ---
void setupHardware();
void scanSensoren();
bool zieLijn();
void motorSturing(int doelL, int doelR);
float berekenPID(float &error, float &vorige_error, float &integraal, int snelheid);
void registreerNode(NodeType t, int intens = 0);
void registreerObstakel(long startPos, long afstandL, long afstandR);
int bepaalDoelSnelheid();
void analyseerParcours(); 


#endif
