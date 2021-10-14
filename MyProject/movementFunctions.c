#include "definitions.h"
#include "movementFunctions.h"

void moveStop(void)
{
	TPM1_C0V = 0;
	TPM1_C1V = 0;
	TPM2_C0V = 0;
	TPM2_C1V = 0;
}

void moveForward(int power)
{
	if (power > 100)
		power = 100;
	if (power < 0)
		power = 0;
	
	float coeff = (float) power / 100.0f;
	
	TPM1_C0V = TPM1->MOD * coeff;
	TPM1_C1V = 0;
	TPM2_C0V = TPM2->MOD * coeff;
	TPM2_C1V = 0;
}

void moveBackward(int power)
{
	if (power > 100)
		power = 100;
	if (power < 0)
		power = 0;
	
	float coeff = (float) power / 100.0f;
	
	TPM1_C0V = 0;
	TPM1_C1V = TPM1->MOD * coeff;
	TPM2_C0V = 0;
	TPM2_C1V = TPM2->MOD * coeff;
}

void moveLeft(int power)
{
	if (power > 100)
		power = 100;
	if (power < 0)
		power = 0;
	
	float coeff = (float) power / 100.0f;
	
	TPM1_C0V = 0;
	TPM1_C1V = TPM1->MOD * coeff;
	TPM2_C0V = TPM2->MOD * coeff;
	TPM2_C1V = 0;
}

void moveRight(int power)
{
	if (power > 100)
		power = 100;
	if (power < 0)
		power = 0;
	
	float coeff = (float) power / 100.0f;
	
	TPM1_C0V = TPM1->MOD * coeff;
	TPM1_C1V = 0;
	TPM2_C0V = 0;
	TPM2_C1V = TPM2->MOD * coeff;
}
