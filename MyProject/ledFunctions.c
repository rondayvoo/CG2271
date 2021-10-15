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
	PTC->PSOR |= MASK(GREEN_LED_1);
	osDelay(1000);
	PTC->PCOR |= MASK(GREEN_LED_1);
	
	PTC->PSOR |= MASK(GREEN_LED_2);
	osDelay(1000);
	PTC->PCOR |= MASK(GREEN_LED_2);
	
	PTC->PSOR |= MASK(GREEN_LED_3);
	osDelay(1000);
	PTC->PCOR |= MASK(GREEN_LED_3);
	
	PTC->PSOR |= MASK(GREEN_LED_4);
	osDelay(1000);
	PTC->PCOR |= MASK(GREEN_LED_4);
	
	PTA->PSOR |= MASK(GREEN_LED_5);
	osDelay(1000);
	PTA->PCOR |= MASK(GREEN_LED_5);
	
	PTA->PSOR |= MASK(GREEN_LED_6);
	osDelay(1000);
	PTA->PCOR |= MASK(GREEN_LED_6);
	
	PTE->PSOR |= MASK(GREEN_LED_7);
	osDelay(1000);
	PTE->PCOR |= MASK(GREEN_LED_7);
	
	PTD->PSOR |= MASK(GREEN_LED_8);
	osDelay(1000);
	PTD->PCOR |= MASK(GREEN_LED_8);
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
}
