#include "config.h"

extern int SPEED_HIGH;

const float ACCELERATIE = 4.0; // Hoe snel de motors versnellen/vertragen (hogere waarde = snellere reactie, maar kan slip veroorzaken)
//const int SNELHEID_MAX = 180;
//max technisch250, maar is te veel volt voor deze motors dus limieteer tot 180 voor 6v op max charge batterij
//const int SNELHEID_BOCHT = 90;

static float huidigeL = 0;
static float huidigeR = 0;

void setupHardware() {
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<MOT_L_AIN1) | (1ULL<<MOT_L_AIN2) | (1ULL<<MOT_R_BIN1) | (1ULL<<MOT_R_BIN2);
    gpio_config(&io_conf);

    ledcSetup(0, 20000, 8); 
    ledcSetup(1, 20000, 8);
    ledcAttachPin(MOT_L_PWM, 0);
    ledcAttachPin(MOT_R_PWM, 1);
}

void motorSturing(int doelL, int doelR) {
    // Instant stop als doel 0 is
    if (doelL == 0 && doelR == 0) {
        huidigeL = 0;
        huidigeR = 0;
    } else {
        if (huidigeL < doelL) huidigeL += ACCELERATIE;
        else if (huidigeL > doelL) huidigeL -= ACCELERATIE;

        if (huidigeR < doelR) huidigeR += ACCELERATIE;
        else if (huidigeR > doelR) huidigeR -= ACCELERATIE;
    }

    // Beperk waarden zodat we binnen PWM 0-255 blijven
    huidigeL = constrain(huidigeL, -255, 255);
    huidigeR = constrain(huidigeR, -255, 255);

    // Linker motor is fysiek omgekeerd gemonteerd: invert sign bij outputs
    gpio_set_level(MOT_L_AIN1, huidigeL <= 0);
    gpio_set_level(MOT_L_AIN2, huidigeL > 0);
    gpio_set_level(MOT_R_BIN1, huidigeR >= 0);
    gpio_set_level(MOT_R_BIN2, huidigeR < 0);

    ledcWrite(0, abs((int)huidigeL));
    ledcWrite(1, abs((int)huidigeR));
}
