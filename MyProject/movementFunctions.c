#include "definitions.h"
#include "movementFunctions.h"

void moveStop(void)
{
	TPM0_C0V = 0;
	TPM0_C1V = 0;
	TPM0_C2V = 0;
	TPM0_C3V = 0;
}

void moveForward(int power)
{
	if (power > 100)
		power = 100;
	if (power < 0)
		power = 0;
	
	float coeff = (float) power / 100.0f;
	
	TPM0_C0V = TPM0->MOD * coeff;
	TPM0_C1V = 0;
	TPM0_C2V = TPM0->MOD * coeff;
	TPM0_C3V = 0;
}

void moveBackward(int power)
{
	if (power > 100)
		power = 100;
	if (power < 0)
		power = 0;
	
	float coeff = (float) power / 100.0f;
	
	TPM0_C0V = 0;
	TPM0_C1V = TPM0->MOD * coeff;
	TPM0_C2V = 0;
	TPM0_C3V = TPM0->MOD * coeff;
}

void moveLeft(int power)
{
	if (power > 100)
		power = 100;
	if (power < 0)
		power = 0;
	
	float coeff = (float) power / 100.0f;
	
	TPM0_C0V = 0;
	TPM0_C1V = TPM0->MOD * coeff;
	TPM0_C2V = TPM0->MOD * coeff;
	TPM0_C3V = 0;
}

void moveRight(int power)
{
	if (power > 100)
		power = 100;
	if (power < 0)
		power = 0;
	
	float coeff = (float) power / 100.0f;
	
	TPM0_C0V = TPM0->MOD * coeff;
	TPM0_C1V = 0;
	TPM0_C2V = 0;
	TPM0_C3V = TPM0->MOD * coeff;
}
