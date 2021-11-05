#include "definitions.h"
#include "ledFunctions.h"
#include "movementFunctions.h"
#include "audioFunctions.h"
#include "queueFunctions.h"
#include "ultrasonicFunctions.h"
#include "initializationFunctions.h"

/*----------------------------------------------------------------------------
 * Global Variables
 *---------------------------------------------------------------------------*/

volatile uint8_t rx_data = ESP32_MISC_RESERVED;
bool isConnected = false;
volatile int isWaitingState = 1;
bool runFinished = false;
mvState currMvState = STOP;
bool isSelfDriving = false;
Q_T Tx_Q, Rx_Q;

const osThreadAttr_t highPriority = {
	.priority = osPriorityHigh
};

const osThreadAttr_t lowPriority = {
	.priority = osPriorityLow
};

osSemaphoreId_t tBrainSem;
osSemaphoreId_t tMotorControlSem;
osSemaphoreId_t tLEDControlSem;
osSemaphoreId_t tAudioControlSem;

/*----------------------------------------------------------------------------
 * UART Interrupt
 *---------------------------------------------------------------------------*/

void UART2_IRQHandler(void) 
{
	NVIC_ClearPendingIRQ(UART2_IRQn);
	
	//IRQ Reciever
	if (UART2->S1 & UART_S1_RDRF_MASK) 
	{
		// received a character
		// RDRF cleared when reading from UART2->D
		if (!Q_Full(&Rx_Q)) 
		{
			Q_Enqueue(&Rx_Q, UART2->D);
			osSemaphoreRelease(tBrainSem);
		} 
		
		else 
		{
			// error - RX_Q full.
			// make space by discarding all information in RX_Q (assume it is not needed anymore)
			Q_Init(&Rx_Q);
			Q_Enqueue(&Rx_Q, UART2->D);
		}
	}
}

/*----------------------------------------------------------------------------
 * Ultrasonic Interrupts
 *---------------------------------------------------------------------------*/

void PIT_IRQHandler (void)
{
	NVIC_ClearPendingIRQ(PIT_IRQn);
	isWaitingState ^= 1;
	
	if (isWaitingState) {
		PIT->CHANNEL[0].LDVAL = TRIGGER_LDVAL_WAIT;
	} 
	
	else {
		PIT->CHANNEL[0].LDVAL = TRIGGER_LDVAL_PULSE;
	}
	
	PTA->PTOR |= MASK(ULTRASONIC_TRIGGER);      // toggle Ultrasonic Trigger
	PIT->CHANNEL[0].TFLG &= PIT_TFLG_TIF_MASK;  // clear Interrupt Request flag for Channel
}

// triggered on ECHO falling edge
void TPM2_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(TPM2_IRQn);  
	
	// no obstacles within ?cm
	// timer has not overflowed
	if (TPM2_C0V <= 1000 && ~(TPM2_STATUS & TPM_STATUS_TOF_MASK)) {
		//moveStop();
	}
	
	else 
	{
		//moveForward(100);
		TPM2_STATUS &= ~TPM_STATUS_TOF_MASK;
	}
	
	// clear Channel Flag (need to clear for both STATUS and CnSC)
	TPM2_STATUS &= ~TPM_STATUS_CH0F_MASK;
	TPM2_C0SC |= TPM_CnSC_CHF(1);
}

/*----------------------------------------------------------------------------
 * Tasks
 *---------------------------------------------------------------------------*/
void tBrain(void *argument)
{
	for (;;) 
	{
		osSemaphoreAcquire(tBrainSem, osWaitForever);
		rx_data = Q_Dequeue(&Rx_Q);
		
		switch (rx_data)
		{
			case ESP32_MISC_CONNECTED:
			{
				isConnected = true;
				rx_data = ESP32_MISC_RESERVED;
				break;
			}
			
			case ESP32_MOVE_FORWARD:
			{
				currMvState = FORWARD;
				osSemaphoreRelease(tMotorControlSem);
				break;
			}
			
			case ESP32_MOVE_BACK:
			{
				currMvState = BACKWARD;
				osSemaphoreRelease(tMotorControlSem);
				break;
			}
			
			case ESP32_MOVE_LEFT:
			{
				currMvState = LEFT;
				osSemaphoreRelease(tMotorControlSem);
				break;
			}
			
			case ESP32_MOVE_RIGHT:
			{
				currMvState = RIGHT;
				osSemaphoreRelease(tMotorControlSem);
				break;
			}
			
			case ESP32_MOVE_STOP:
			{
				currMvState = STOP;
				osSemaphoreRelease(tMotorControlSem);
				break;
			}
			
			default:
				break;
		}
	}
}

void tMotorControl(void *argument)
{
	for (;;) 
	{
		osSemaphoreAcquire(tMotorControlSem, osWaitForever);
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

void tRedLED(void *argument)
{
	for (;;)
	{
		if (currMvState == STOP)
		{
			redLedOff();
			redBlink(250);
		}
			
		else
		{
			redLedOff();
			redBlink(500);
		}
	}
}

void tGreenLED(void *argument)
{
	int currLit = 0;
	
	for (;;)
	{
		if (currMvState == STOP)
			greenLedOn();
		else
		{
			greenLedRunning(currLit);
			currLit = currLit == 7 ? 0 : currLit + 1;
		}
	}
}

void tLED(void *argument)
{
	for (;;) 
	{
		if (isConnected)
		{
			greenLedTwoBlinks();
			osThreadNew(tRedLED, NULL, &lowPriority);
			osThreadNew(tGreenLED, NULL, &lowPriority);
			return;
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
		
		else if (isConnected)
		{
			audioSong(currNote);
			currNote = currNote + 1 == SONGMAIN_NOTE_COUNT ? 0 : currNote + 1;
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
	initUltrasonic();
	initUART2(BAUD_RATE);	
	startUltrasonic();
	
	/* ----------------- Semaphores ----------------- */
	tBrainSem = osSemaphoreNew(Q_SIZE,0,NULL);
	tMotorControlSem = osSemaphoreNew(1,0,NULL);
	tAudioControlSem = osSemaphoreNew(1,0,NULL);
	
	/* ----------------- Threads/Kernels ----------------- */
	osKernelInitialize();    // Initialize CMSIS-RTOS
	osThreadNew(tBrain, NULL, &highPriority);
	osThreadNew(tMotorControl, NULL, NULL);
	osThreadNew(tLED, NULL, &lowPriority);
	osThreadNew(tAudio, NULL, &lowPriority);
	osKernelStart();                      // Start thread execution
	
	while (1) {
		//moveForward(100);
	}
}
