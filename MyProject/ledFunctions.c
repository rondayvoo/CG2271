#include "definitions.h"
#include "ledFunctions.h"

/*
#define RED_LED 7                   //PTD7
#define GREEN_LED_1 12              //PTC12
#define GREEN_LED_2 13              //PTC13
#define GREEN_LED_3 16              //PTC16
#define GREEN_LED_4 17              //PTC17
#define GREEN_LED_5 16              //PTA16
#define GREEN_LED_6 17              //PTA17
#define GREEN_LED_7 31              //PTE31
#define GREEN_LED_8 6               //PTD6
*/

void greenLedOff(void)
{
	PTC->PCOR |= MASK(GREEN_LED_1);
	PTC->PCOR |= MASK(GREEN_LED_2);
	PTC->PCOR |= MASK(GREEN_LED_3);
	PTC->PCOR |= MASK(GREEN_LED_4);
	PTA->PCOR |= MASK(GREEN_LED_5);
	PTA->PCOR |= MASK(GREEN_LED_6);
	PTE->PCOR |= MASK(GREEN_LED_7);
	PTD->PCOR |= MASK(GREEN_LED_8);
}

void greenLedOn(void)
{
	PTC->PSOR |= MASK(GREEN_LED_1);
	PTC->PSOR |= MASK(GREEN_LED_2);
	PTC->PSOR |= MASK(GREEN_LED_3);
	PTC->PSOR |= MASK(GREEN_LED_4);
	PTA->PSOR |= MASK(GREEN_LED_5);
	PTA->PSOR |= MASK(GREEN_LED_6);
	PTE->PSOR |= MASK(GREEN_LED_7);
	PTD->PSOR |= MASK(GREEN_LED_8);
}

void greenLedTwoBlinks(void)
{
	greenLedOff();
	greenLedOn();
	osDelay(200);
	greenLedOff();
	osDelay(200);
	greenLedOn();
	osDelay(200);
	greenLedOff();
}

void greenLedRunning(int currLit)
{
	greenLedOff();
	
	switch (currLit) 
	{
		case 0:
			PTC->PSOR |= MASK(GREEN_LED_1);
			osDelay(200);
			PTC->PCOR |= MASK(GREEN_LED_1);
			break;
		case 1:
			PTC->PSOR |= MASK(GREEN_LED_2);
			osDelay(200);
			PTC->PCOR |= MASK(GREEN_LED_2);
			break;
		case 2:
			PTC->PSOR |= MASK(GREEN_LED_3);
			osDelay(200);
			PTC->PCOR |= MASK(GREEN_LED_3);
			break;
		case 3:
			PTC->PSOR |= MASK(GREEN_LED_4);
			osDelay(200);
			PTC->PCOR |= MASK(GREEN_LED_4);
			break;
		case 4:
			PTA->PSOR |= MASK(GREEN_LED_5);
			osDelay(200);
			PTA->PCOR |= MASK(GREEN_LED_5);
			break;
		case 5:
			PTA->PSOR |= MASK(GREEN_LED_6);
			osDelay(200);
			PTA->PCOR |= MASK(GREEN_LED_6);
			break;
		case 6:
			PTE->PSOR |= MASK(GREEN_LED_7);
			osDelay(200);
			PTE->PCOR |= MASK(GREEN_LED_7);
			break;
		case 7:
			PTD->PSOR |= MASK(GREEN_LED_8);
			osDelay(200);
			PTD->PCOR |= MASK(GREEN_LED_8);
			break;
		default:
			break;
	}
}

void redLedOff(void)
{
	PTD->PCOR |= MASK(RED_LED);
}

void redLedOn(void)
{
	PTD->PSOR |= MASK(RED_LED);
}

void redBlink(int ms)
{
	redLedOn();
	osDelay(ms);
	redLedOff();
	osDelay(ms);
}
