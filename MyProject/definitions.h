#ifndef DEFINITIONS
#define DEFINITIONS

#include "RTE_Components.h"
#include CMSIS_device_header
#include "cmsis_os2.h"
#include "stdbool.h"
#include "stdlib.h"

#define MASK(x) (0x1U << x)

/* --------------------------- LEDs -----------------------------------*/
#define RED_LED 7                   //PTD7
#define GREEN_LED_1 12              //PTC12
#define GREEN_LED_2 13              //PTC13
#define GREEN_LED_3 16              //PTC16
#define GREEN_LED_4 17              //PTC17
#define GREEN_LED_5 16              //PTA16
#define GREEN_LED_6 17              //PTA17
#define GREEN_LED_7 31              //PTE31
#define GREEN_LED_8 6               //PTD6

/* --------------------------- Motors -----------------------------------*/
#define LEFT_MOTOR_FWD 0        //PTD0 - TPM0_CH0
#define LEFT_MOTOR_RVS 4        //PTA4 - TPM0_CH1
#define RIGHT_MOTOR_FWD 2       //PTD2 - TPM0_CH2
#define RIGHT_MOTOR_RVS 3       //PTD3 - TPM0_CH3

/* --------------------------- Buzzer/Audio  -----------------------------------*/
#define BUZZER 20               //PTE20 - TPM1_CH0
#define FREQUENCY_TO_MOD(x) (375000 / (x))
#define SONGCONNEST_NOTE_COUNT 32
#define SONGMAIN_NOTE_COUNT 64
#define SONGRUNFIN_NOTE_COUNT 15
#define C4 261
#define D4 293
#define E4 329
#define F4 349
#define G4 392
#define A4 440
#define As4 466
#define B4 493
#define C5 523

/* --------------------------- Ultrasound  -----------------------------------*/
#define ULTRASONIC_TRIGGER 2           // PTA2, PIT (TTL Input, to trigger measurement)
#define ULTRASONIC_ECHO 2              // PTB2 TPM2_CH0 (TTL Output, pulse proportional to distance)
#define ULTRASONIC_EXTRGIN 8           // PTB8, EXTRGIN (for TPMx_CONF)
#define TRIGGER_LDVAL_PULSE 240        // for 20 microsecond activation pulse
#define TRIGGER_LDVAL_WAIT 24000000    // for 2 second wait pulse

/* --------------------------- UART/ESP32  -----------------------------------*/
#define BAUD_RATE 9600
#define UART_TX 22                  //PTE22 - TX
#define UART_RX 23                  //PTE23 - RX
#define UART2_INT_PRIO 128

#define ESP32_LEDRED_ON 0b00000001U
#define ESP32_LEDRED_OFF 0b00000010U
#define ESP32_LEDGREEN_ON 0b00000011U
#define ESP32_LEDGREEN_OFF 0b00000100U 

#define ESP32_MOVE_STOP 0b00110000U
#define ESP32_MOVE_FORWARD 0b00110001U
#define ESP32_MOVE_BACK 0b00110010U
#define ESP32_MOVE_LEFT 0b00110011U
#define ESP32_MOVE_RIGHT 0b00110100U

#define ESP32_MODE_MANUAL 0b11110000U
#define ESP32_MODE_AUTO 0b11110001U

#define ESP32_MISC_RESERVED 0b11000000U
#define ESP32_MISC_CONNECTED 0b11000001U
#define ESP32_MISC_TESTING_ON 0b11000010U
#define ESP32_MISC_TESTING_OFF 0b11000011U

#define Q_SIZE 32
typedef struct {
	uint8_t Data[Q_SIZE];
	unsigned int Head; 				// points to oldest data element
	unsigned int Tail; 				// points to next free space
	unsigned int Size; 				// quantity of elements in queue
} Q_T;

typedef enum moveState {
	STOP, FORWARD, BACKWARD, LEFT, RIGHT
} mvState;

#endif
