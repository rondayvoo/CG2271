#include "definitions.h"
#include "audioFunctions.h"

int songConnEst[SONGCONNEST_NOTE_COUNT] = {C4, D4, E4, F4, G4, A4, B4, C5, B4, A4, G4, F4, E4, D4, C4};
int songMain[0] = {};
int songRunFin[0] = {};

void audioStop(void)
{
}

void audioConnEst(void)
{
    for (int i = 0; i < SONGCONNEST_NOTE_COUNT; i++) {
        TPM0->MOD = songConnEst[i] * 6;       // play at music note frequency
        TPM0_C2V = songConnEst[i] * 6 / 2;    // mantain 50% duty cycle
    }
}

void audioSong(int note)
{
	TPM0->MOD = songMain[note];
	TPM0_C2V = songMain[note] / 2;
	osDelay(100);
}

void audioRunFin(void)
{
	for (int i = 0; i < SONGRUNFIN_NOTE_COUNT; i++) {
        TPM0->MOD = songRunFin[i];       // play at music note frequency
        TPM0_C2V = songRunFin[i] / 2;    // mantain 50% duty cycle
    }
}
