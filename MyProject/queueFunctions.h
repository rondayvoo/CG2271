#include "definitions.h"

void Q_Init(Q_T * q);
int Q_Empty(Q_T * q);
int Q_Full(Q_T * q);
int Q_Enqueue(Q_T * q, unsigned char d);
unsigned char Q_Dequeue(Q_T * q);
