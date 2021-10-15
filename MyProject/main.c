#include "definitions.h"
#include "ledFunctions.h"
#include "movementFunctions.h"
#include "audioFunctions.h"
#include "queueFunctions.h"
#include "initializationFunctions.h"

/*----------------------------------------------------------------------------
 * Global Variables
 *---------------------------------------------------------------------------*/

volatile unsigned char rx_data = ESP32_MISC_RESERVED;
volatile bool isConnected = false;
bool runFinished = false;
mvState currMvState = STOP;
bool isSelfDriving = false;
Q_T Tx_Q, Rx_Q;

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
	rx_data = Q_Dequeue(&Rx_Q);

	for (;;) 
	{
		switch (rx_data)
		{
			case ESP32_MISC_CONNECTED:
			{
				greenLedTwoBlinks();
				isConnected = true;
				rx_data = ESP32_MISC_RESERVED;
				break;
			}
			
			case ESP32_MOVE_FORWARD:
			{
				currMvState = FORWARD;
				break;
			}
			
			case ESP32_MOVE_BACK:
			{
				currMvState = BACKWARD;
				break;
			}
			
			case ESP32_MOVE_LEFT:
			{
				currMvState = LEFT;
				break;
			}
			
			case ESP32_MOVE_RIGHT:
			{
				currMvState = RIGHT;
				break;
			}
			
			case ESP32_MOVE_STOP:
			{
				currMvState = STOP;
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
 
	osKernelInitialize();                 // Initialize CMSIS-RTOS
	osThreadNew(tBrain, NULL, NULL);
	osThreadNew(tMotorControl, NULL, NULL);
	osThreadNew(tLED, NULL, NULL);
	osThreadNew(tAudio, NULL, NULL);
	osKernelStart();                      // Start thread execution
	for (;;) {}
}
