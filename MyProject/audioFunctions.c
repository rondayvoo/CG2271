#include "definitions.h"
#include "audioFunctions.h"

int songConnEst[SONGCONNEST_NOTE_COUNT] = {
As4, As4, 0, 0, As4, As4, 0, 0, As4, As4, 0, 0, As4, As4, 0, 0,
As4, As4, As4, B4, B4, B4, F4 * 2, F4 * 2, As4, As4, 0, 0, As4, As4, 0, 0
};
int songMain[SONGMAIN_NOTE_COUNT] = {
A4, G4, A4, E4 * 2, D4 * 2, C5, A4, A4, A4, G4, A4, E4 * 2, D4 * 2, C5, A4, A4,
C5, B4, G4, C5, C5, B4, G4, G4, C5, B4, G4, D4 * 2, D4 * 2, B4, G4, G4, 
A4, G4, A4, E4 * 2, D4 * 2, C5, A4, A4, A4, G4, A4, E4 * 2, D4 * 2, C5, A4, A4,
A4, A4, 0, A4, B4, 0, B4, 0, C5, C5, 0, C5, B4, 0, B4, 0
};
int songRunFin[0] = {};

void audioStop(void)
{
	TPM1->MOD = 0;
	TPM1_C0V = 0;
}

void audioConnEst(void)
{
    for (int i = 0; i < SONGMAIN_NOTE_COUNT; i++) {
        TPM1->MOD = FREQUENCY_TO_MOD(songConnEst[i] * 4);       // play at music note frequency
        TPM1_C0V = FREQUENCY_TO_MOD(songConnEst[i] * 4) / 2;    // mantain 50% duty cycle
				osDelay(100);
    }
		TPM1->MOD = 0;
		TPM1_C0V = 0;
}

void audioSong(int note)
{
	TPM1->MOD = FREQUENCY_TO_MOD(songMain[note] * 4);
	TPM1_C0V = FREQUENCY_TO_MOD(songMain[note] * 4) / 2;
	osDelay(90);
}

void audioRunFin(void)
{
	for (int i = 0; i < SONGMAIN_NOTE_COUNT; i++) {
        TPM1->MOD = FREQUENCY_TO_MOD(songRunFin[i] * 4);       // play at music note frequency
        TPM1_C0V = FREQUENCY_TO_MOD(songRunFin[i] * 4) / 2;    // mantain 50% duty cycle
				osDelay(100);
    }
		TPM1->MOD = 0;
		TPM1_C0V = 0;
}
