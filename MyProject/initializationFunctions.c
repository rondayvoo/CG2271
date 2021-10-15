#include "definitions.h"
#include "queueFunctions.h"
#include "initializationFunctions.h"

extern Q_T Rx_Q;
extern Q_T Tx_Q;

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
#define TESTING_LED 8               //PTB8
*/
void initLED(void)
{
	// Enable Clock to PORTA, PORTC, PORTD, PORTE
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	
	// Configure MUX settings to make all pins GPIO
	PORTD->PCR[RED_LED] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[RED_LED] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_1] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_1] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_2] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_2] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_3] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_3] |= PORT_PCR_MUX(1);
	
	PORTC->PCR[GREEN_LED_4] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[GREEN_LED_4] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[GREEN_LED_5] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[GREEN_LED_5] |= PORT_PCR_MUX(1);
	
	PORTA->PCR[GREEN_LED_6] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[GREEN_LED_6] |= PORT_PCR_MUX(1);
	
	PORTE->PCR[GREEN_LED_7] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[GREEN_LED_7] |= PORT_PCR_MUX(1);
	
	PORTD->PCR[GREEN_LED_8] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[GREEN_LED_8] |= PORT_PCR_MUX(1);
	
	PORTB->PCR[TESTING_LED] &= ~PORT_PCR_MUX_MASK;
	PORTB->PCR[TESTING_LED] |= PORT_PCR_MUX(1);
	
	// Data Direction Register (Make all to Output)
	PTC->PDDR |= MASK(GREEN_LED_1 | GREEN_LED_2 | GREEN_LED_3 | GREEN_LED_4);
	PTD->PDDR |= MASK(RED_LED | GREEN_LED_5 | GREEN_LED_6 | GREEN_LED_8);
	PTE->PDDR |= MASK(GREEN_LED_7);
	PTB->PDDR |= MASK(TESTING_LED);
	//PTB->PSOR |= MASK(TESTING_LED); 
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
	
	//Select edge-aligned PWM with high-true pulses, for TPM1/TPM2, CH0/CH1
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

	  // Configure PORTE29 (TPM0_CH2) using MUX
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

    // Setting Timer Mode (Edge-aligned PWM, high true pulses) for TPM0_CH2
    TPM0_C2SC &= ~( (TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
    TPM0_C2SC |= ( TPM_CnSC_ELSB(1) | TPM_CnSC_MSB(1) );
}

void initUART2(uint32_t baud_rate) {
	uint32_t divisor, bus_clock;
	
	//Enable clock for UART2 and Port E
	SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;  
	
	//Configure mode 4 for Port E Pin 23: UART2_RX
	PORTE->PCR[UART_RX] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[UART_RX] |= PORT_PCR_MUX(4);
	
	//Turn off RX and TX, disable UART
	UART2->C2 &= ~((UART_C2_TE_MASK) | (UART_C2_RE_MASK));

	//Update the baud setting
	//Note: Always write to BDH first, then BDL
	bus_clock = DEFAULT_SYSTEM_CLOCK / 2;
	divisor = bus_clock / (baud_rate * 16);
	UART2->BDH = UART_BDH_SBR(divisor >> 8);
	UART2->BDL = UART_BDL_SBR(divisor);
	
	//Use default UART2 configuration
	UART2->C1 = 0;
	UART2->S2 = 0;
	UART2->C3 = 0;
	
	NVIC_ClearPendingIRQ(UART2_IRQn);
	NVIC_EnableIRQ(UART2_IRQn);
	NVIC_SetPriority(UART2_IRQn, 0);
	
	//Turn on RX and respective Interrupt
	UART2->C2 |= ( (UART_C2_RE_MASK) | (UART_C2_RIE_MASK));
	
	Q_Init(&Tx_Q);
	Q_Init(&Rx_Q);
}

void init_UART2_Poll (uint32_t baud_rate) {
    // turn on power to UART and Transmitting Pin (Clock Gating)
    SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

    // clear MUX, then set to ALT4 (corresponds to UART2)
    //PORTE->PCR[UART_TX_PORTE22] &= ~PORT_PCR_MUX_MASK;
    //PORTE->PCR[UART_TX_PORTE22] |= PORT_PCR_MUX(4);
    PORTE->PCR[UART_RX] &= ~PORT_PCR_MUX_MASK;
    PORTE->PCR[UART_RX] |= PORT_PCR_MUX(4);

    // disable TX and RX for UART2 (must do before loading UART_divisor)
    UART2->C2 &= ~(UART_C2_RE_MASK | UART_C2_TE_MASK);

    // load values into UART_divisor (BDH and BDL, which controls prescalar division for UART baud rate generation)
    int BUS_CLOCK = (DEFAULT_SYSTEM_CLOCK) / 2;
    int UART_Divisor = BUS_CLOCK / (baud_rate * 16);
    UART2->BDH = UART_BDH_SBR(UART_Divisor >> 8);    // BDL captures lower 8bits, BDH captures remainder upper bits
    UART2->BDL = UART_BDL_SBR(UART_Divisor);

    // disable all uneeded UART settings (we disable Interrupts here also)
    UART2->C1 = 0;
    UART2->S2 = 0;
    UART2->C3 = 0;

    // enable UART TX and RX
    UART2->C2 |= ( (UART_C2_TE_MASK) | (UART_C2_RE_MASK) );
}

uint8_t UART2_RX_Poll (void) {
    // accessing S1 register, and then checking RDRF field
    // wait until Receive Data Register is full (RDRF = 1)
    while ( !(UART2->S1 & UART_S1_RDRF_MASK) );
    return (UART2->D);
}
