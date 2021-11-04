#include "ultrasonicFunctions.h"

void startUltrasonic (void)
{
	// Begin pulsing Ultrasonic TRIGGER
	PIT->CHANNEL[0].TCTRL |= PIT_TCTRL_TEN_MASK;
  NVIC_SetPriority(PIT_IRQn, 1);	
	NVIC_EnableIRQ(PIT_IRQn);
	
	// Ultrasonic Echo
	NVIC_SetPriority(TPM2_IRQn, 0);
	NVIC_EnableIRQ(TPM2_IRQn);
}
