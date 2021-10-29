#include "movementFunctions.c"
#include "definitions.h"

void driveSelf() {
	moveForward(100);
	// if the ultrasonic sensor detects object
	// stop
	moveStop();

	moveLeft(100);
	osDelay(1000);
	moveStop();

	moveForward(100);
	osDelay(1000);
	moveStop();

	moveRight(100);
	osDelay(1000);
	moveStop();

	moveForward(100);
	osDelay(1000);
	moveStop();

	moveRight(100);
	osDelay(1000);
	moveStop();

	moveForward(100);
	osDelay(1000);
	moveStop();

	moveRight(100);
	osDelay(1000);
	moveStop();

	moveForward(100);
	osDelay(1000);
	moveStop();

	moveRight(100);
	osDelay(1000);
	moveStop();

	moveForward(100);
	osDelay(1000);
	moveStop();

	moveLeft(100);
	osDelay(1000);
	moveStop();

	moveForward(100);
	osDelay(1000);
	moveStop();
}