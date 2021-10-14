#include "definitions.h"
#include "audioFunctions.h"

extern int songConnEst[SONGCONNEST_NOTE_COUNT];
extern int songMain[0];

void audioStop(void)
{
}

void audioConnEst(void)
{
    for (int i = 0; i < SONGCONNEST_NOTE_COUNT; i++) {
        TPM0->MOD = songConnEst[i];       // play at music note frequency
        TPM0_C0V = songConnEst[i] / 2;    // mantain 50% duty cycle
    }
}

void audioSong(int note)
{
	TPM0->MOD = songMain[note];
	TPM0_C0V = songMain[note] / 2;
	osDelay(100);
}

void audioRunFin(void)
{
}
