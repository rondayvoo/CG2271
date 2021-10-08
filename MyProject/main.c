/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#define RED_LED 18
#define GREEN_LED 19
#define BLUE_LED 1
#define FRONT_LEFT_MOTOR 0
#define FRONT_RIGHT_MOTOR 0
#define BACK_LEFT_MOTOR 0
#define BACK_RIGHT_MOTOR 0
#define MASK(x) (1 << x)
 
/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/


void initLED()
{
	SIM->SCGC5 |= ((SIM_SCGC5_PORTB_MASK) | (SIM_SCGC5_PORTD_MASK));
	
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED] |= PORT_PCR_MUX(1);
	
	PORTD->PCR[BLUE_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[BLUE_LED] |= PORT_PCR_MUX(1);
}

void initMotors() 
{
	//Enable clock for Port B
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	//Configure to be mode 3: TPM1_CH0
	PORTB->PCR[FRONT_LEFT_MOTOR] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[FRONT_LEFT_MOTOR] |= PORT_PCR_MUX(3);
	
	//Configure to be mode 3: TPM1_CH1
	PORTB->PCR[FRONT_RIGHT_MOTOR] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[FRONT_RIGHT_MOTOR] |= PORT_PCR_MUX(3);
	
	//Enable clock for TPM1 module
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	//Select MCGFLLCLK
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	//Set to 50Hz
	TPM1->MOD = 7500;
	
	//Set LPTPM counter to increment on every counter clock
	//Set prescaler to 1/128
	//Select up-counting mode
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= ((TPM_SC_CMOD(1)) | (TPM_SC_PS(7)));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	//Select edge-aligned PWM with high-true pulses
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1)) | (TPM_CnSC_MSB(1));
}

void initBuzzer()
{
}

void app_main (void *argument) {
 
  // ...
  for (;;) {}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
	initLED();
  initMotors();
 
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
}
