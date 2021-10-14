#include "definitions.h"
#include "ledFunctions.h"

void greenLedOff(void)
{
	PTB->PCOR |= MASK(GREEN_LED_1);
	PTB->PCOR |= MASK(GREEN_LED_2);
	PTB->PCOR |= MASK(GREEN_LED_3);
	PTB->PCOR |= MASK(GREEN_LED_4);
	PTB->PCOR |= MASK(GREEN_LED_5);
	PTB->PCOR |= MASK(GREEN_LED_6);
	PTB->PCOR |= MASK(GREEN_LED_7);
	PTB->PCOR |= MASK(GREEN_LED_8);
}

void greenLedOn(void)
{
	PTB->PSOR |= MASK(GREEN_LED_1);
	PTB->PSOR |= MASK(GREEN_LED_2);
	PTB->PSOR |= MASK(GREEN_LED_3);
	PTB->PSOR |= MASK(GREEN_LED_4);
	PTB->PSOR |= MASK(GREEN_LED_5);
	PTB->PSOR |= MASK(GREEN_LED_6);
	PTB->PSOR |= MASK(GREEN_LED_7);
	PTB->PSOR |= MASK(GREEN_LED_8);
}

void greenLedTwoBlinks(void)
{
	greenLedOn();
	osDelay(1000);
	greenLedOff();
	osDelay(1000);
	greenLedOn();
	osDelay(1000);
	greenLedOff();
	osDelay(1000);
}

void greenLedRunning(void)
{
	PTB->PSOR |= MASK(GREEN_LED_1);
	osDelay(1000);
	PTB->PCOR |= MASK(GREEN_LED_1);
	
	PTB->PSOR |= MASK(GREEN_LED_2);
	osDelay(1000);
	PTB->PCOR |= MASK(GREEN_LED_2);
	
	PTB->PSOR |= MASK(GREEN_LED_3);
	osDelay(1000);
	PTB->PCOR |= MASK(GREEN_LED_3);
	
	PTB->PSOR |= MASK(GREEN_LED_4);
	osDelay(1000);
	PTB->PCOR |= MASK(GREEN_LED_4);
	
	PTB->PSOR |= MASK(GREEN_LED_5);
	osDelay(1000);
	PTB->PCOR |= MASK(GREEN_LED_5);
	
	PTB->PSOR |= MASK(GREEN_LED_6);
	osDelay(1000);
	PTB->PCOR |= MASK(GREEN_LED_6);
	
	PTB->PSOR |= MASK(GREEN_LED_7);
	osDelay(1000);
	PTB->PCOR |= MASK(GREEN_LED_7);
	
	PTB->PSOR |= MASK(GREEN_LED_8);
	osDelay(1000);
	PTB->PCOR |= MASK(GREEN_LED_8);
}

void redLedOff(void)
{
	PTB->PCOR |= MASK(RED_LED);
}

void redLedOn(void)
{
	PTB->PSOR |= MASK(RED_LED);
}

void redBlink(int ms)
{
	redLedOn();
	osDelay(ms);
	redLedOff();
}
