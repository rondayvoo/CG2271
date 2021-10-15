#include "definitions.h"

void initLED(void);
void initMotors(void);
void initBuzzer(void);
void initUART2(uint32_t baud_rate);
void init_UART2_Poll(uint32_t baud_rate);
uint8_t UART2_RX_Poll (void);
