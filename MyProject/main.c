#include "definitions.h"
#include "ledFunctions.h"
#include "movementFunctions.h"
#include "audioFunctions.h"
#include "queueFunctions.h"
#include "initializationFunctions.h"

/*----------------------------------------------------------------------------
 * Global Variables
 *---------------------------------------------------------------------------*/

volatile uint8_t rx_data = ESP32_MISC_RESERVED;
volatile bool isConnected = false;
bool runFinished = false;
mvState currMvState = STOP;
bool isSelfDriving = false;
Q_T Tx_Q, Rx_Q;
//int musical_notes[7] = {1262,1294,1330,1349,1392,1440,1494};

osSemaphoreId_t tBrainSem;
osSemaphoreId_t tMotorControlSem;
osSemaphoreId_t tLEDControlSem;
osSemaphoreId_t tAudioControlSem;

static void delay (volatile uint32_t nof) {
	while (nof!= 0) {
		__asm("NOP");
		nof--;
	}
}

void long_delay(volatile uint32_t nof) {
    for (int i = 0; i < 100; i++)
        delay(nof);
}


/*----------------------------------------------------------------------------
 * UART
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
				osSemaphoreRelease(tAudioControlSem);
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

void tLED(void *argument)
{
	for (;;) 
	{
		osSemaphoreAcquire(tLEDControlSem, osWaitForever);
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
		osSemaphoreAcquire(tAudioControlSem, osWaitForever);
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
			currNote = currNote == SONGMAIN_NOTE_COUNT ? 0 : currNote + 1;
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
	
	/* ----------------- Semaphores ----------------- */
	tBrainSem = osSemaphoreNew(1,0,NULL);
	tMotorControlSem = osSemaphoreNew(1,0,NULL);
	tAudioControlSem = osSemaphoreNew(1,0,NULL);
	
	/* ----------------- Threads/Kernels ----------------- */
	osKernelInitialize();    // Initialize CMSIS-RTOS
	osThreadNew(tBrain, NULL, NULL);
	osThreadNew(tMotorControl, NULL, NULL);
	osThreadNew(tLED, NULL, NULL);
	osThreadNew(tAudio, NULL, NULL);
	//osKernelStart();                      // Start thread execution
	
	for (;;) {
		audioConnEst();
		/*while (1) {
        for (int i = 0; i < 6; i++) {
            TPM0->MOD = FREQUENCY_TO_MOD(musical_notes[i]);          // I want a frequency corresponding to a musical note (set MOD to do it)
            TPM0_C2V = (FREQUENCY_TO_MOD(musical_notes[i])/ 2);     // to mantain 50% Duty Cycle
					long_delay(0xFFFF);
        }
	}
}*/
	}
}
