#include "definitions.h"
#include "audioFunctions.h"

int songConnEst[SONGCONNEST_NOTE_COUNT] = {
As4, As4, 0, 0, As4, As4, 0, 0, As4, As4, 0, 0, As4, As4, 0, 0,
As4, As4, As4, B4, B4, B4, F4 * 2, F4 * 2, As4, As4, 0, 0, As4, As4, 0, 0
};
int songMain[0] = {};
int songRunFin[0] = {};
	
static void delay(volatile uint32_t nof) {
	while (nof != 0) {
		__asm("NOP");
		nof--;
	}
}

void audioStop(void)
{
}

void audioConnEst(void)
{
    for (int i = 0; i < SONGCONNEST_NOTE_COUNT; i++) {
        TPM1->MOD = FREQUENCY_TO_MOD(songConnEst[i] * 4);       // play at music note frequency
        TPM1_C0V = FREQUENCY_TO_MOD(songConnEst[i] * 4) / 2;    // mantain 50% duty cycle
				osDelay(100);
    }
		TPM1->MOD = 0;
		TPM1_C0V = 0;
}

void audioSong(int note)
{
	TPM1->MOD = songMain[note];
	TPM1_C0V = songMain[note] / 2;
	osDelay(100);
}

void audioRunFin(void)
{
	for (int i = 0; i < SONGRUNFIN_NOTE_COUNT; i++) {
        TPM1->MOD = songRunFin[i];       // play at music note frequency
        TPM1_C0V = songRunFin[i] / 2;    // mantain 50% duty cycle
    }
}
