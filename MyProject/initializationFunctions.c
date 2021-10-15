#include "definitions.h"
#include "queueFunctions.h"
#include "initializationFunctions.h"

extern Q_T Rx_Q;
extern Q_T Tx_Q;

void initLED(void)
{
	// Enable Clock to PORTB
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	// Configure MUX settings to make all 3 pins GPIO
	PORTB->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RED_LED] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED_1] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED_1] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED_2] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED_2] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED_3] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED_3] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED_4] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED_4] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED_5] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED_5] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED_6] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED_6] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED_7] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED_7] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[GREEN_LED_8] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[GREEN_LED_8] |= PORT_PCR_MUX(1);
	
	//PTB->PDDR |= (MASK(RED_LED) | MASK(GREEN_LED));
}

void initMotors(void) 
{
	//Enable clock for Port B
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	//Configure to be mode 3: TPM1_CH0
	PORTB->PCR[LEFT_MOTOR_FWD] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[LEFT_MOTOR_FWD] |= PORT_PCR_MUX(3);
	
	//Configure to be mode 3: TPM1_CH1
	PORTB->PCR[LEFT_MOTOR_RVS] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[LEFT_MOTOR_RVS] |= PORT_PCR_MUX(3);
	
	//Configure to be mode 3: TPM2_CH0
	PORTB->PCR[RIGHT_MOTOR_FWD] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RIGHT_MOTOR_FWD] |= PORT_PCR_MUX(3);
	
	//Configure to be mode 3: TPM2_CH1
	PORTB->PCR[RIGHT_MOTOR_RVS] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[RIGHT_MOTOR_RVS] |= PORT_PCR_MUX(3);
	
	//Enable clock for TPM1 module
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	
	//Enable clock for TPM2 module
	SIM->SCGC6 |= SIM_SCGC6_TPM2_MASK;
	
	//Select MCGFLLCLK for system
	SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK;
	SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1);
	
	//Set both timers to 50Hz
	TPM1->MOD = 7500;
	TPM2->MOD = 7500;
	
	//Set LPTPM counter to increment on every counter clock
	//Set prescaler to 1/128
	//Select up-counting mode
	TPM1->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM1->SC |= ((TPM_SC_CMOD(1)) | (TPM_SC_PS(7)));
	TPM1->SC &= ~(TPM_SC_CPWMS_MASK);
	
	TPM2->SC &= ~((TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK));
	TPM2->SC |= ((TPM_SC_CMOD(1)) | (TPM_SC_PS(7)));
	TPM2->SC &= ~(TPM_SC_CPWMS_MASK);
	
	//Select edge-aligned PWM with high-true pulses
	TPM1_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C0SC |= (TPM_CnSC_ELSB(1)) | (TPM_CnSC_MSB(1));
	
	TPM1_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM1_C1SC |= (TPM_CnSC_ELSB(1)) | (TPM_CnSC_MSB(1));
	
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1)) | (TPM_CnSC_MSB(1));
	
	TPM2_C1SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C1SC |= (TPM_CnSC_ELSB(1)) | (TPM_CnSC_MSB(1));
}

void initBuzzer(void)
{
    // Enable PortE
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	// Configure PORTE24 (TPM0_CH0) using MUX
    PORTE->PCR[BUZZER] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[BUZZER] |= PORT_PCR_MUX(3);

    // Enable TPM0
    SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
    // SIM->SOPT2 (common Clock Source) already established in initMotors

    /* set MOD value
     * 48Mhz Clock / 128 Prescalar = effective 375000Hz Clock (slower)
     * I want 50Hz PWM signal and to use Edge-Aligned PWM Signal, hence PWM period = (MOD+1) cycles
     * 375000Hz / 7500 (MOD) gives 50Hz
     */
    TPM0->MOD = 7500;

    // Set CMOD (Clock Mode Selection and Prescalar)
    TPM0->SC &= ~( (TPM_SC_CMOD_MASK) | (TPM_SC_PS_MASK) );
    TPM0->SC |= ( (TPM_SC_CMOD(1) | TPM_SC_PS(7)) );

    // Setting Timer Mode (Edge-aligned PWM, high true pulses) for TPM0_CH0
    TPM0_C0SC &= ~( (TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM0_C0SC |= ( TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1) );
}

void initUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	
	//Enable clock for UART2 and Port E
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	//Configure mode 4 for Port E Pin 22: UART2_TX
	PORTE->PCR[UART_TX] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX] |= PORT_PCR_MUX(4);
	
	//Configure mode 4 for Port E Pin 23: UART2_RX
	PORTE->PCR[UART_TX] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX] |= PORT_PCR_MUX(4);
	
	//Turn off RX and TX, disable UART
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	//Update the baud setting
	//Note: Always write to BDH first, then BDL
	bus_clock = DEFAULT_SYSTEM_CLOCK / 2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDH_SBR(divisor);
	
	//Use default UART2 configuration
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	//Turn on TX and RX, enable UART
	UART2->C2 |= ((UART_C2_TE_MASK) | (UART_C2_RE_MASK));
	
	NVIC_SetPriority(UART2_IRQn, 128);
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	UART2->C2 |= (UART_C2_TIE_MASK | UART_C2_RIE_MASK);
	UART2->C2 |= UART_C2_RIE_MASK;
	Q_Init(&Tx_Q);
	Q_Init(&Rx_Q);
}
