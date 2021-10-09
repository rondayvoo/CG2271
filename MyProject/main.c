#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"
#include "stdbool.h"
#include "stdlib.h"

#define MASK(x) (1 << x)

#define RED_LED 0						//Not assigned yet
#define GREEN_LED_1 0				//Not assigned yet
#define GREEN_LED_2 0				//Not assigned yet
#define GREEN_LED_3 0				//Not assigned yet
#define GREEN_LED_4 0				//Not assigned yet
#define GREEN_LED_5 0				//Not assigned yet
#define GREEN_LED_6 0				//Not assigned yet
#define GREEN_LED_7 0				//Not assigned yet
#define GREEN_LED_8 0				//Not assigned yet

#define LEFT_MOTOR_FWD 0		//PTB0 - TPM1_CH0
#define LEFT_MOTOR_RVS 1		//PTB1 - TPM1_CH1
#define RIGHT_MOTOR_FWD 2		//PTB2 - TPM2_CH0
#define RIGHT_MOTOR_RVS 3		//PTB3 - TPM2_CH1

#define BUZZER 0						//Not assigned yet

#define BAUD_RATE 9600
#define UART_TX 22					//PTE22 - TX
#define UART_RX 23					//PTE23 - RX
#define UART2_INT_PRIO 128

#define Q_SIZE 32

typedef struct {
	unsigned char Data[Q_SIZE];
	unsigned int Head; 				// points to oldest data element
	unsigned int Tail; 				// points to next free space
	unsigned int Size; 				// quantity of elements in queue
} Q_T;

typedef enum moveState {
	STOP, FORWARD, BACKWARD, LEFT, RIGHT
} mvState;

/*----------------------------------------------------------------------------
 * Global Variables
 *---------------------------------------------------------------------------*/

bool isConnected = false;
bool runFinished = false;
mvState currMvState = STOP;
bool isSelfDriving = false;
Q_T Tx_Q, Rx_Q;

static int songConnEst[0] = {};
static int songMain[0] = {};
static int songRunFin[0] = {};

/*----------------------------------------------------------------------------
 * LEDs
 *---------------------------------------------------------------------------*/

void greenLedOff()
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

void greenLedOn()
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

void greenLedTwoBlinks()
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

void greenLedRunning()
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

void redLedOff()
{
	PTB->PCOR |= MASK(RED_LED);
}

void redLedOn()
{
	PTB->PSOR |= MASK(RED_LED);
}

void redBlink(int ms)
{
	redLedOn();
	osDelay(ms);
	redLedOff();
}

/*----------------------------------------------------------------------------
 * Movement
 *---------------------------------------------------------------------------*/

void moveStop()
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

/*----------------------------------------------------------------------------
 * Audio
 *---------------------------------------------------------------------------*/

void audioStop()
{
}

void audioConnEst()
{
}

void audioSong(int note)
{
	TPM0->MOD = songMain[note];
	TPM0_C0V = songMain[note] / 2;
	osDelay(100);
}

void audioRunFin()
{
}

/*----------------------------------------------------------------------------
 * Queue Operations
 *---------------------------------------------------------------------------*/

void Q_Init(Q_T * q) 
{
	unsigned int i;
	
	for (i=0; i<Q_SIZE; i++)
		q->Data[i] = 0; // to simplify our lives when debugging
	
	q->Head = 0;
	q->Tail = 0;
	q->Size = 0;
}

int Q_Empty(Q_T * q) 
{
	return q->Size == 0;
}

int Q_Full(Q_T * q) 
{
	return q->Size == Q_SIZE;
}

int Q_Enqueue(Q_T * q, unsigned char d) 
{
	// What if queue is full?
	if (!Q_Full(q)) 
	{
		q->Data[q->Tail++] = d;
		q->Tail %= Q_SIZE;
		q->Size++;
		return 1; // success
	} 
	
	else
		return 0; // failure
}
unsigned char Q_Dequeue(Q_T * q) 
{
	// Must check to see if queue is empty before dequeueing
	unsigned char t = 0;
	
	if (!Q_Empty(q)) 
	{
		t = q->Data[q->Head];
		q->Data[q->Head++] = 0; // to simplify debugging
		q->Head %= Q_SIZE;
		q->Size--;
	}
	
	return t;
}

/*----------------------------------------------------------------------------
 * UART
 *---------------------------------------------------------------------------*/

void UART2_IRQHandler(void) 
{
	NVIC_ClearPendingIRQ(UART2_IRQn);
	
	//IRQ Transmitter
	if (UART2->S1 & UART_S1_TDRE_MASK) 
	{
		// can send another character
		if (!Q_Empty(&Tx_Q)) 
		{
			UART2->D = Q_Dequeue(&Tx_Q);
		} 
		
		else 
		{
			// queue is empty so disable tx
			UART2->C2 &= ~UART_C2_TIE_MASK;
		}
	}
	
	//IRQ Reciever
	if (UART2->S1 & UART_S1_RDRF_MASK) 
	{
		// received a character
		if (!Q_Full(&Rx_Q)) 
		{
			Q_Enqueue(&Rx_Q, UART2->D);
		} 
		
		else 
		{
			// error - queue full.
			while (1);
		}
	}
}

/*----------------------------------------------------------------------------
 * Initialization
 *---------------------------------------------------------------------------*/

void initLED()
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

void initMotors() 
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
	
	TPM2_C0SC &= ~((TPM_CnSC_ELSB_MASK) | (TPM_CnSC_ELSA_MASK) | (TPM_CnSC_MSB_MASK) | (TPM_CnSC_MSA_MASK));
	TPM2_C0SC |= (TPM_CnSC_ELSB(1)) | (TPM_CnSC_MSB(1));
}

void initBuzzer()
{
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

/*----------------------------------------------------------------------------
 * Tasks
 *---------------------------------------------------------------------------*/
void tBrain(void *argument)
{
	unsigned char connected[Q_SIZE] = "something";
	
	for (;;) 
	{
		if (Rx_Q.Data == connected)
		{
			greenLedTwoBlinks();
		}
	}
}

void tMotorControl(void *argument)
{
	for (;;) 
	{
		switch (currMvState)
		{
			case STOP:
				moveStop();
				break;
			case FORWARD:
				moveForward(100);
				break;
			case BACKWARD:
				moveBackward(100);
				break;
			case LEFT:
				moveLeft(100);
				break;
			case RIGHT:
				moveRight(100);
				break;
			default:
				break;
		}
	}
}

void tLED(void *argument)
{
	for (;;) 
	{
		if (currMvState == STOP)
		{
			greenLedOn();
			redBlink(250);
		}
		
		else
		{
			greenLedRunning();
			redBlink(500);
		}
	}
}

void tAudio(void *argument)
{
	bool localIsConnected = false;
	bool localRunFinished = false;
	int currNote = 0;
	
	for (;;) 
	{
		if (isConnected && !localIsConnected)
		{
			audioConnEst();
			localIsConnected = true;
		}
		
		else if (runFinished && !localRunFinished)
		{
			audioRunFin();
			localRunFinished = true;
		}
		
		else
		{
			audioSong(currNote);
			currNote = currNote >= 64 ? 0 : currNote + 1;
		}
	}
}

/*----------------------------------------------------------------------------
 * Main
 *---------------------------------------------------------------------------*/
 
int main (void) {
	// System Initialization
	SystemCoreClockUpdate();
	initLED();
	initMotors();
	initBuzzer();
	initUART2(BAUD_RATE);
 
	osKernelInitialize();                 // Initialize CMSIS-RTOS
	osThreadNew(tBrain, NULL, NULL);
	osThreadNew(tMotorControl, NULL, NULL);
	osThreadNew(tLED, NULL, NULL);
	osThreadNew(tAudio, NULL, NULL);
	osKernelStart();                      // Start thread execution
	for (;;) {}
}
