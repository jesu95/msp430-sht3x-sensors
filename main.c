#include "sht3x.h"

void main(void) {

	int16_t temp = 0, hum = 0;

    WDT_A_hold(WDT_A_BASE);
    initSht3x();
    Init_LCD();
    PMM_unlockLPM5();
    __bis_SR_register(GIE);

    while(1)
    {
        getMeasure();
        temp = getTemp();
        hum  = getHum();
        showTempHum(temp, hum);
    }
}
