#include <Stepper.h>
#include "Direction.h"

// SYSTEM CONFIGURATION: KEYFRAME MOTOR POSITIONS
int positions[] = {35,88,140,188,60,110,160,10}; 

// SYSTEM CONFIGURATION
const int N_STEPS = 200;								// number of motor steps to rotate 360 degrees  (ie if N_STEPS = 200, each step is 1.8 degrees)
const int HALF_N_STEPS = N_STEPS/2;
const int N_POSITIONS = sizeof(positions) / sizeof(int);
const int MOTION_DETECT_DELAY = 2000; 					// in miliseconds - to reduce double triggers
const int LOOP_DELAY = 200; 							// in miliseconds - this interacts with RANGE_SENSOR_READ_INTERVAL_MS to affect motion sensitivity
const int N_MOVEMENTS_TO_TRIGGER_MOTOR = 4; 			// somebody walking past is probably 2 movements (depending on the value of RANGE_SENSOR_READ_INTERVAL_MS)
const int RANGE_SENSOR_DIFFERENCE_THRESHOLD = 12;		// in inches
const int RANGE_SENSOR_READ_INTERVAL_MS = 1500;			// delay between attepts to detect motion
const int RANGE_SENSOR_MAX_RANGE = 85; 					// in inches. to calm sensor jitter, we ignore readings larger than this.
const int STEPPER_SPEED = 3; 							// motor speed (in rpm?)
const int STARTUP_DELAY_BEFORE_INIT_MOTOR = 5000; 		// in miliseconds

// ARDUINO PIN CONFIGURATION
const int pwmA = 3;
const int pwmB = 11;
const int brakeA = 9;
const int brakeB = 8;
const int dirA = 12;
const int dirB = 13;
const int HALL_SENSOR_PIN = 4;
const int RANGE_SENSOR_PIN = 2;

// STATE
unsigned long lastTriggerTime = 0;
unsigned long lastSensorReadTime = 0;
long lastSensorReadValue = 0;
int movementsCounted = 0;
int currentMotorStep = 0;
int toggleCounter = 0;
bool motorRequiresInit = false;
bool motorHomeIsSet = false;
bool firstTime = true;

// PROPERTIES
Stepper motor(N_STEPS, dirA, dirB);

///////////////////////////////////////////////////////////
#pragma mark arduino lifecycle

void setup() {
	Serial.begin(9600);
	lastTriggerTime = lastSensorReadTime = millis();
}

void loop() {
	/*
	// for motor testing only - comment out in production
	if (firstTime){
		delay(STARTUP_DELAY_BEFORE_INIT_MOTOR);
		initMotor();
		findHomePosition();
		firstTime = false;
	} else {
		currentMotorStep = currentMotorStep + 1;
		motor.step(2000);
		Serial.println(currentMotorStep);
		delay(1000000);
	} */
	
	if (motorHomeIsSet){
		if (movementDetected()){
			movementsCounted = movementsCounted + 1;
			Serial.print("Movements before trigger: ");
			Serial.println(N_MOVEMENTS_TO_TRIGGER_MOTOR - movementsCounted);	
			if (movementsCounted >= N_MOVEMENTS_TO_TRIGGER_MOTOR){
				Serial.println("triggering motor");
				triggerMotor();
				movementsCounted = 0;
				lastSensorReadValue = 0;
				// TODO - maybe wait after motor triggered if feedback is that it's too active
			}
			delay(MOTION_DETECT_DELAY); // wait a couple for seconds before we look for further movement
		}
	} else {
		// motor home is not set - this is the first run
		if (firstTime){
			delay(STARTUP_DELAY_BEFORE_INIT_MOTOR);
			initMotor();
			findHomePosition();
			firstTime = false;
		}
		// if motor home isn't set AND it's not the first time, we probably have an error condition - so do nothing....
	}
	delay(LOOP_DELAY);
}


int getNewPosition(){
	return randomPosition(N_POSITIONS,positions);
}

///////////////////////////////////////////////////////////

#pragma mark range sensor
bool movementDetected(){
	bool movement = false;
	unsigned long timeNow = millis();
	if (timeNow > lastSensorReadTime + RANGE_SENSOR_READ_INTERVAL_MS){
		lastSensorReadTime = timeNow;
		long range = readRangeSensor();
		if (range >RANGE_SENSOR_MAX_RANGE) {
			// the range that's returned is erroneous so ignore it
			Serial.print("ignoring a bad range of: ");
			Serial.println(range);
		} else {
			if (lastSensorReadValue != 0){
				int difference = abs(lastSensorReadValue - range);
				/*
				Serial.print("range: ");
				Serial.print(range);
				Serial.print(" difference: ");
				Serial.print(range);
				Serial.print(" threshold: ");
				Serial.println(RANGE_SENSOR_DIFFERENCE_THRESHOLD);
				*/
				if ( difference > RANGE_SENSOR_DIFFERENCE_THRESHOLD){
					Serial.print("movement detected from ");
					Serial.print(lastSensorReadValue);
					Serial.print(" to ");
					Serial.println(range);
					movement = true;
				} 
			} 
			lastSensorReadValue = range;
		}
	}
	return movement;
}

long readRangeSensor(){
	Serial.print(".");
	long duration, inches;
	pinMode(RANGE_SENSOR_PIN, OUTPUT);
	digitalWrite(RANGE_SENSOR_PIN, LOW);
  	delayMicroseconds(2);
  	digitalWrite(RANGE_SENSOR_PIN, HIGH);
  	delayMicroseconds(5);
  	digitalWrite(RANGE_SENSOR_PIN, LOW);

  	pinMode(RANGE_SENSOR_PIN, INPUT);
  	duration = pulseIn(RANGE_SENSOR_PIN, HIGH);
  	inches = microsecondsToInches(duration);
	return inches;
}

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second).  This gives the distance travelled by the ping, outbound
  // and return, so we divide by 2 to get the distance of the obstacle.
  // See: http://www.parallax.com/dl/docs/prod/acc/28015-PING-v1.3.pdf
  return microseconds / 74 / 2;
}

///////////////////////////////////////////////
#pragma mark motor

void testMotor(){
	if (motorRequiresInit) initMotor();
	Serial.println("motor will go 200");
	motor.step(200);
	delay(10000);
}

void triggerMotor(){
	Serial.println("motor triggered");
	int newStep = getNewPosition();
	int driveSteps = shortRoute(currentMotorStep,newStep);
	Serial.print("motor will go ");
	Serial.print(driveSteps);
	Serial.print(" from ");
	Serial.print(currentMotorStep);
	Serial.print(" to ");
	Serial.println(newStep);
	motor.step(driveSteps); // TODO: uncomment to drive!
	currentMotorStep = newStep; // strictly not true until the motor has finished, but should be covered by the delay between triggers
}

void initMotor(){
	motor.setSpeed(STEPPER_SPEED);
	
	// init pins
	pinMode(pwmA, OUTPUT);
	pinMode(pwmB, OUTPUT);
	pinMode(brakeA, OUTPUT);
	pinMode(brakeB, OUTPUT);

	// pwm on
	 digitalWrite(pwmA, HIGH);
	 digitalWrite(pwmB, HIGH);
	
	// brakes off
	digitalWrite(brakeA, LOW);
	digitalWrite(brakeB, LOW);
	motorRequiresInit = true;
}

bool hallSensorAtHome(){
	int hallState = digitalRead(HALL_SENSOR_PIN);
	return (hallState == LOW);
}

void findHomePosition(){
	pinMode(HALL_SENSOR_PIN, INPUT);
	Serial.println("seeking home position");
	int steps_tried = 0;
	while(1){
		if (hallSensorAtHome()) break;
		motor.step(1);
		steps_tried ++;
		if (steps_tried > (N_STEPS*2)){
			Serial.println("tried to find home for 2 full revolutions. giving up. power cycle to try again :(");
			while (1){
				; // infinite loop of sadness
			}
		}
		delay(100);
	}
	Serial.println("found home");
	currentMotorStep = 0;
	motorHomeIsSet = true;
}

///////////////////////////////////////////////
#pragma mark direction

Direction randomDirection(){
	return (random(2) ? CLOCKWISE : ANTICLOCKWISE);
}
int shortRoute(int start, int end){
	int shortdelta = (end - start) % N_STEPS;
	if (shortdelta != shortdelta % HALF_N_STEPS) {
		shortdelta = (shortdelta < 0) ? shortdelta + N_STEPS : shortdelta - N_STEPS;
	}
	return shortdelta;
}
int longRoute(int start, int end){
	int shortdelta = shortRoute(start,end);
	int longdelta = (shortdelta < 0) ? shortdelta + N_STEPS : shortdelta - N_STEPS;
	return longdelta;
}
bool isInChosenDirection(int start, int end, Direction direction){
	int route = shortRoute(start,end);
	if (direction == CLOCKWISE){
		return (route > 0);
	} else {
		return (route < 0);
	}
}
int randomPosition(int nAllPositions, int * allPositions){
	int positionChosen = 0;

	// choose a random direction
	Direction direction = randomDirection();

	// filter positions to select only positions the shortest route to which is in the chosen direction
	int nFilteredPositions = 0;
	int filteredPositions[nAllPositions];
	for (int i=0;i<nAllPositions;i++){
		filteredPositions[i] = 0;
	}
	for (int i=0;i<nAllPositions;i++){
		int candidate = allPositions[i];
		if (isInChosenDirection(currentMotorStep,candidate,direction)){
			filteredPositions[nFilteredPositions] = candidate;
			nFilteredPositions++;
		}
	}
	
	// choose a random position from the filtered list
	if (nFilteredPositions > 0){
		positionChosen = filteredPositions[random(nFilteredPositions)];
	}
	return positionChosen;
}
int realposition(int p){
	int effective = p % N_STEPS;
	if (p < 0) effective = (N_STEPS + effective) % N_STEPS;
	return effective;
}

/*

int randomPosition(){
  int r = (random(N_POSITIONS/2) + 1) * (N_STEPS / N_POSITIONS);
  return random(2) ? r : -r;
}

int randomPosition(){
  int r = random(5) * 25;
  return random(2) ? r : -r;
}

*/
