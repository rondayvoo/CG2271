#define MASK(x) (1 << x)

/* --------------------------- LEDs -----------------------------------*/
#define RED_LED 0                   //Not assigned yet
#define GREEN_LED_1 0               //Not assigned yet
#define GREEN_LED_2 0               //Not assigned yet
#define GREEN_LED_3 0               //Not assigned yet
#define GREEN_LED_4 0               //Not assigned yet
#define GREEN_LED_5 0               //Not assigned yet
#define GREEN_LED_6 0               //Not assigned yet
#define GREEN_LED_7 0               //Not assigned yet
#define GREEN_LED_8 0               //Not assigned yet

/* --------------------------- Motors -----------------------------------*/
#define LEFT_MOTOR_FWD 0        //PTB0 - TPM1_CH0
#define LEFT_MOTOR_RVS 1        //PTB1 - TPM1_CH1
#define RIGHT_MOTOR_FWD 2       //PTB2 - TPM2_CH0
#define RIGHT_MOTOR_RVS 3       //PTB3 - TPM2_CH1

/* --------------------------- Buzzer/Audio  -----------------------------------*/
#define BUZZER 24               //PTE24 - TPM0_CH0
#define FREQUENCY_TO_MOD(x) (375000 / (x))
#define SONGCONNEST_NOTE_COUNT 15
#define C4 261
#define D4 293
#define E4 329
#define F4 349
#define G4 392
#define A4 440
#define B4 493
#define C5 523

/* --------------------------- UART/ESP32  -----------------------------------*/
#define BAUD_RATE 9600
#define UART_TX 22                  //PTE22 - TX
#define UART_RX 23                  //PTE23 - RX
#define UART2_INT_PRIO 128

#define ESP32_LEDRED_ON 0b00000001
#define ESP32_LEDRED_OFF 0b00000010
#define ESP32_LEDGREEN_ON 0b00000011
#define ESP32_LEDGREEN_OFF 0b00000100 
#define ESP32_MOVE_STOP 0b00110000 
#define ESP32_MOVE_FORWARD 0b00110001
#define ESP32_MOVE_BACK 0b00110010
#define ESP32_MOVE_LEFT 0b00110011 
#define ESP32_MOVE_RIGHT 0b00110100
#define ESP32_MODE_MANUAL 0b11110000
#define ESP32_MODE_AUTO 0b11110001
#define ESP32_MISC_RESERVED 0b00000000 
#define ESP32_MISC_CONNECTED 0b11111111

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