#include "selfDriving.h"
#include "movementFunctions.h"
#include "ultrasonicFunctions.h"
#include "definitions.h"

extern volatile bool objectDetected;

void driveSelf() {
	startUltrasonic();
	
	moveForward(100);
	while (!objectDetected);
	stopUltrasonic();
	objectDetected = false;

	moveLeft(100);
	osDelay(1000);

	moveForward(100);
	osDelay(1000);

	moveRight(100);
	osDelay(1000);
	moveStop();

	moveForward(100);
	osDelay(1000);

	moveRight(100);
	osDelay(1000);

	moveForward(100);
	osDelay(1000);

	moveRight(100);
	osDelay(1000);

	moveForward(100);
	osDelay(1000);

	moveLeft(100);
	osDelay(1000);
	
	startUltrasonic();

	moveForward(100);
	while (!objectDetected);
	stopUltrasonic();
	objectDetected = false;
	moveStop();
}
