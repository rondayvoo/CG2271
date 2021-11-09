#include "selfDriving.h"
#include "movementFunctions.h"
#include "ultrasonicFunctions.h"
#include "definitions.h"

extern volatile bool objectDetected;

void driveSelf() {
	objectDetected = false;
	moveForward(100);
	while (!objectDetected)
	{
		osDelay(10);
	}
	objectDetected = false;

	moveLeft(100);
	osDelay(600);
/*
	moveForward(100);
	osDelay(1000);

	moveRight(100);
	osDelay(1000);
	moveStop();

	moveForward(100);
	osDelay(1000);

	moveRight(100);
	osDelay(1100);

	moveForward(100);
	osDelay(1000);

	moveRight(100);
	osDelay(1100);

	moveForward(100);
	osDelay(1000);

	moveLeft(100);
	osDelay(600);
	
	startUltrasonic();

	moveForward(100);
	while (!objectDetected)
	{
		osDelay(20);
	}
	stopUltrasonic();
	objectDetected = false;
	*/
	moveStop();
}
