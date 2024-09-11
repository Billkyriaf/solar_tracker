#include <Arduino.h>
#include <BH1750.h>
#include <BH1750_Soft.h>
#include <Wire.h>

typedef struct sensor {
	BH1750 bh1750;
	int enPin;
	float lux;
} Sensor;

typedef struct softSensor {
	BH1750_Soft bh1750;
	int enPin;
	float lux;
} SoftSensor;


Sensor upSensor;
Sensor rightSensor;
SoftSensor leftSensor;

#define DEBUG 1


// define the pins used by the stepper motors and the sensors
#define UP_SENSOR_EN 2
#define LEFT_SENSOR_EN 3
#define RIGHT_SENSOR_EN 4

#define dirPin_az 8
#define stepPin_az 9
#define dirPin_pol 10
#define stepPin_pol 11

// define the limits of the stepper motors
#define POLAR_LIMIT_U 38000L
#define POLAR_LIMIT_L -38000L
#define AZIMUTH_LIMIT_U 56000L
#define AZIMUTH_LIMIT_L -56000L

#define stepsPerRevolution 6400
#define SPEED_DELAY 400

#define ACC_THRESHOLD 1
#define IRRADIANCE_THRESHOLD 1
#define MOVE_THRESHOLD 10


long unsigned int start = 0;
long unsigned int end = 0;

float upLeftDiff = 0;
float upRightDiff = 0;
float leftRightDiff = 0;

long polar;   // counts the number of steps taken
long azimuth; // counts the number of steps taken

bool polarLimit;    // flag to indicate if the polar limit is reached
bool azimuthLimit;  // flag to indicate if the azimuth limit is reached
bool lowIrradiance;  // flag to indicate if the irradiance is low

uint8_t rxBuffer[2];
uint8_t txBuffer[2];

/**
 * @brief Enables the sensor and configures it to take a high resolution reading
 * 
 * @param sensor  the sensor struct to store the sensor data in
 * @param enPin   the enable pin for the sensor
 */
void setupSensor(Sensor *sensor, int enPin) {

	// read the enable pin
	uint8_t pin = digitalRead(enPin);

	if (pin == HIGH){
		// initialize the sensor
		sensor->bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &Wire);
	} else {
		// initialize the sensor
		sensor->bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
	}

	sensor->enPin = enPin;
}


/**
 * @brief Enables the sensor and configures it to take a high resolution reading
 * 
 * @param sensor  the sensor struct to store the sensor data in
 * @param enPin   the enable pin for the sensor
 */
void setupSensor(SoftSensor *sensor, int enPin) {

	// read the enable pin
	uint8_t pin = digitalRead(enPin);

	if (pin == HIGH){
		// initialize the sensor
		sensor->bh1750.begin(BH1750_Soft::CONTINUOUS_HIGH_RES_MODE, 0x5C, &SWire);
	} else {
		// initialize the sensor
		sensor->bh1750.begin(BH1750_Soft::CONTINUOUS_HIGH_RES_MODE, 0x23, &SWire);
	}

	sensor->enPin = enPin;
}


/**
 * @brief Reads the light level from the sensor and stores it in the sensor struct
 * 
 * @param sensor  the sensor struct to store the light level in
 * @return void
 */
inline void readSensor(Sensor *sensor) {

	// read the light level from the sensor
	sensor->lux = sensor->bh1750.readLightLevel();
}


/**
 * @brief Reads the light level from the sensor and stores it in the sensor struct
 * 
 * @param sensor  the sensor struct to store the light level in
 * @return void
 */
inline void readSensor(SoftSensor *sensor) {
	// read the light level from the sensor
	sensor->lux = sensor->bh1750.readLightLevel();
}

/**
 * @brief Moves the stepper motors in the horizontal position (polar = 0, azimuth = 0)
 * 
 */
void moveHorizontal() {

	// then move the polar stepper motor to the 0 position
	if (polar < 0) {
		// ... turn the motor in the opposite direction
		digitalWrite(dirPin_pol, HIGH);

		// turn the motor 360 degrees (until the azimuth position is 0)
		while (polar < 0) {
			
			start = micros();

			digitalWrite(stepPin_pol, HIGH);  // step the polar stepper motor

			end = micros();

			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

			start = micros();

			digitalWrite(stepPin_pol, LOW);  // step the polar stepper motor

			end = micros();

			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

			// update the azimuth position
			polar++;
		}

	} else if (polar > 0) {
		// ... turn the motor in the opposite direction
		digitalWrite(dirPin_pol, LOW);

		// turn the motor 360 degrees (until the azimuth position is 0)
		while (polar > 0) {
			
			start = micros();

			digitalWrite(stepPin_pol, HIGH);  // step the polar stepper motor

			end = micros();

			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

			start = micros();

			digitalWrite(stepPin_pol, LOW);  // step the polar stepper motor

			end = micros();

			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

			// update the azimuth position
			polar--;
		}
	}


	long unsigned int steps = 0;  // the number of steps taken
	
	
	// first move the azimuth stepper motor to the 0 position
	if (azimuth < 0) {
		// ... turn the motor in the opposite direction
		digitalWrite(dirPin_az, HIGH);
		digitalWrite(dirPin_pol, HIGH);

		// turn the motor 360 degrees (until the azimuth position is 0)
		while (azimuth < 0) {
			
			start = micros();

			digitalWrite(stepPin_az, HIGH);

			// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
			if ((steps % 9 == 0 || steps % 9 == 5) && steps !=0 ) {
				digitalWrite(stepPin_pol, HIGH);  // step the polar stepper motor
			}

			end = micros();

			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed


			start = micros();

			digitalWrite(stepPin_az, LOW);
			
			// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
			if ((steps % 9 == 0 || steps % 9 == 5) && steps !=0 ) {
				digitalWrite(stepPin_pol, LOW);  // step the polar stepper motor
			}

			end = micros();

			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

			// update the azimuth position
			azimuth++;
			steps++;
		}

	} else if (azimuth > 0) {
		// ... turn the motor in the opposite direction
		digitalWrite(dirPin_az, LOW);
		digitalWrite(dirPin_pol, LOW);

		// turn the motor 360 degrees (until the azimuth position is 0)
		while (azimuth > 0) {
			
			start = micros();

			digitalWrite(stepPin_az, HIGH);

			// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
			if ((steps % 9 == 0 || steps % 9 == 5) && steps !=0 ) {
				digitalWrite(stepPin_pol, HIGH);  // step the polar stepper motor
			}

			end = micros();

			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed


			start = micros();

			digitalWrite(stepPin_az, LOW);
			
			// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
			if ((steps % 9 == 0 || steps % 9 == 5) && steps !=0 ) {
				digitalWrite(stepPin_pol, LOW);  // step the polar stepper motor
			}

			end = micros();

			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

			// update the azimuth position
			azimuth--;
			steps++;
		}
	}

	// reset the limit flags
	polarLimit = false;
	azimuthLimit = false;
}

void setup() {
	
	// Declare stepper pins as output:
  	pinMode(stepPin_az, OUTPUT);
  	pinMode(dirPin_az, OUTPUT);

	pinMode(stepPin_pol, OUTPUT);	
	pinMode(dirPin_pol, OUTPUT);

	// Declare sensor enable pins as output:
	pinMode(UP_SENSOR_EN, OUTPUT);
	pinMode(LEFT_SENSOR_EN, OUTPUT);
	pinMode(RIGHT_SENSOR_EN, OUTPUT);

	// Initialize serial communication at 9600 bits per second:
	Serial.begin(9600);

	// Set the communication buffers for the software I2C:
	SWire.setRxBuffer(rxBuffer, 2);
	SWire.setTxBuffer(txBuffer, 2);

	// Initialize the I2C communication:
	Wire.begin();
	SWire.begin();


	// Initialize the sensors' addresses:
	digitalWrite(UP_SENSOR_EN, HIGH);
	digitalWrite(RIGHT_SENSOR_EN, LOW);
	digitalWrite(LEFT_SENSOR_EN, LOW);

	// Setup the sensors
	setupSensor(&upSensor, UP_SENSOR_EN);
	setupSensor(&rightSensor, RIGHT_SENSOR_EN);
	setupSensor(&leftSensor, LEFT_SENSOR_EN);


	// Set the direction pin to HIGH (CW):
  	digitalWrite(dirPin_az, HIGH);
  	digitalWrite(dirPin_pol, HIGH);

	// init the step counters
	polar = 0;
	azimuth = 0;

	// init the limit flags
	polarLimit = false;
	azimuthLimit = false;
	lowIrradiance = false;

	Serial.println("Setup complete!");

	// read the light level from the sensors
	readSensor(&upSensor);
	readSensor(&rightSensor);
	readSensor(&leftSensor);

	Serial.print("Up: ");
	Serial.println(upSensor.lux);

	Serial.print("Right: ");
	Serial.println(rightSensor.lux);

	Serial.print("Left: ");
	Serial.println(leftSensor.lux);
}



void loop(){

	// read the light level from the sensors
	readSensor(&upSensor);
	readSensor(&rightSensor);
	readSensor(&leftSensor);


	if (lowIrradiance){
		// first check if the irradiance is low
		readSensor(&upSensor);
		readSensor(&rightSensor);
		readSensor(&leftSensor);

		if (upSensor.lux > IRRADIANCE_THRESHOLD || leftSensor.lux > IRRADIANCE_THRESHOLD || upSensor.lux > IRRADIANCE_THRESHOLD){
			lowIrradiance = false;

		} else {
			// move to the horizontal position (polar = 0, azimuth = 0)
			moveHorizontal();
		}
	}

	
	if (azimuthLimit){
		// if the azimuth limit is reached make a 360 degree turn in the opposite direction
		Serial.println("Azimuth limit reached!!! ");
		Serial.println();


		// if the azimuth limit is at the max positive value ...
		if (azimuth == AZIMUTH_LIMIT_U){

			// ... turn the motor in the opposite direction
			digitalWrite(dirPin_az, LOW);
			digitalWrite(dirPin_pol, LOW);

			// turn the motor 360 degrees (until the azimuth position is 0)
			for (long i = 0; i < AZIMUTH_LIMIT_U; i++) {

				start = micros();

				digitalWrite(stepPin_az, HIGH);
				
				// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
				if ((i % 9 == 0 || i % 9 == 5) && i !=0 ) {
					digitalWrite(stepPin_pol, HIGH);  // step the polar stepper motor
				}
				end = micros();

				if (end - start < SPEED_DELAY)
					delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed


				start = micros();

				digitalWrite(stepPin_az, LOW);

				// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
				if ((i % 9 == 0 || i % 9 == 5) && i !=0 ) {
					digitalWrite(stepPin_pol, LOW);  // step the polar stepper motor
				}

				end = micros();

				if (end - start < SPEED_DELAY)
					delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

				// update the azimuth position
				azimuth--;
			}
			
		// if the azimuth limit is at the max negative value ...
		} else if (azimuth == AZIMUTH_LIMIT_L){
			
			// ... turn the motor in the opposite direction
			digitalWrite(dirPin_az, HIGH);
			digitalWrite(dirPin_pol, HIGH);

			// turn the motor 360 degrees (until the azimuth position is 0)
			for (long i = 0; i < AZIMUTH_LIMIT_U; i++) {
				
				start = micros();

				digitalWrite(stepPin_az, HIGH);

				// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
				if ((i % 9 == 0 || i % 9 == 5) && i !=0 ) {
					digitalWrite(stepPin_pol, HIGH);  // step the polar stepper motor
				}

				end = micros();

				if (end - start < SPEED_DELAY)
					delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed


				start = micros();

				digitalWrite(stepPin_az, LOW);
				
				// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
				if ((i % 9 == 0 || i % 9 == 5) && i !=0 ) {
					digitalWrite(stepPin_pol, LOW);  // step the polar stepper motor
				}

				end = micros();

				if (end - start < SPEED_DELAY)
					delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

				// update the azimuth position
				azimuth++;
			}

		}
		
		// reset the azimuth limit flag
		azimuthLimit = false;

	} else if (leftSensor.lux - rightSensor.lux > MOVE_THRESHOLD) {
		// the first sensors to be aligned are the left and right sensors
		Serial.println("Left > Right");
		Serial.println();

		// move the polar stepper motor in the direction of the left sensor
		digitalWrite(dirPin_pol, HIGH);
		digitalWrite(dirPin_az, HIGH);

		long unsigned int steps = 0;

		azimuthLimit = false;  // the motor will move so even if the limit was reached the flag will be set to false

		start = micros();

		while (leftSensor.lux - rightSensor.lux > ACC_THRESHOLD) {

			// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
			if ((steps % 9 == 0 || steps % 9 == 5) && steps !=0 ) {
				digitalWrite(stepPin_pol, HIGH);  // step the polar stepper motor
				readSensor(&leftSensor);
			}

			if (steps % 1000 == 0){
				Serial.print("Steps: ");
				Serial.print(azimuth);
				Serial.print(" Left sens: ");
				Serial.print(leftSensor.lux);
			}
			
			digitalWrite(stepPin_az, HIGH);  // step the azimuth stepper motor
			
			end = micros();

			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed
			


			start = micros();

			if ((steps % 9 == 0 || steps % 9 == 5) && steps !=0 ) {
				digitalWrite(stepPin_pol, LOW);  // step the polar stepper motor
				readSensor(&rightSensor);       // read the right sensor
			}

			if (steps % 1000 == 0){
				Serial.print(" Right sens: ");
				Serial.println(rightSensor.lux);
				Serial.println();
			}

			digitalWrite(stepPin_az, LOW);  // step the polar stepper motor
			azimuth++;                      // decrement the azimuth step counter


			end = micros();

			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

			start = micros();

			// if (DEBUG == 1 && steps % 1000 == 0) {

			// 	Serial.print("Left - Right: ");
			// 	Serial.println(leftSensor.lux - rightSensor.lux);
			// 	Serial.println();
			// }

			if (leftSensor.lux - rightSensor.lux < ACC_THRESHOLD) {
				
				Serial.print("Left sens: ");
				Serial.print(leftSensor.lux);			
				Serial.print("  Right sens: ");
				Serial.println(rightSensor.lux);
				Serial.println();
				break;
			}

			steps++;

			if (azimuth == AZIMUTH_LIMIT_U) {
				azimuthLimit = true;

				break;
			}
		}
		
	} else if (leftSensor.lux - rightSensor.lux < -MOVE_THRESHOLD) {

		// move the polar stepper motor in the direction of the right sensor
		Serial.println("Left < Right");
		Serial.println();
		
		
		digitalWrite(dirPin_pol, LOW);
		digitalWrite(dirPin_az, LOW);

		long unsigned int steps = 0;

		azimuthLimit = false;  // the motor will move so even if the limit was reached the flag will be set to false

		start = micros();

		while (leftSensor.lux - rightSensor.lux < -ACC_THRESHOLD) {
			
			// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
			if ((steps % 9 == 0 || steps % 9 == 5) && steps !=0 ) {
				digitalWrite(stepPin_pol, HIGH);  // step the polar stepper motor
				readSensor(&leftSensor);          // read the left sensor
			}

			if (steps % 1000 == 0){
				Serial.print("Steps: ");
				Serial.print(azimuth);
				Serial.print(" Left sens: ");
				Serial.print(leftSensor.lux);
			}
			
			digitalWrite(stepPin_az, HIGH);  // step the azimuth stepper motor
			
			
			end = micros();
			
			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed
			

			start = micros();

			if ((steps % 9 == 0 || steps % 9 == 5) && steps !=0 ) {
				digitalWrite(stepPin_pol, LOW);  // step the polar stepper motor
				readSensor(&rightSensor);        // read the right sensor
			}
			
			if (steps % 1000 == 0){
				Serial.print(" Right sens: ");
				Serial.println(rightSensor.lux);
				Serial.println();
			}

			digitalWrite(stepPin_az, LOW);  // step the polar stepper motor
			azimuth--;                      // increment the azimuth step counter


			end = micros();

			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

			start = micros();

			if (leftSensor.lux - rightSensor.lux > -ACC_THRESHOLD) {
				Serial.print("Left sens: ");
				Serial.print(leftSensor.lux);
				Serial.print("  Right sens: ");
				Serial.println(rightSensor.lux);
				Serial.println();
				break;
			}

			steps++;

			if (azimuth == AZIMUTH_LIMIT_L) {
				azimuthLimit = true;
				break;
			}
		}

	} else if (upSensor.lux - leftSensor.lux > MOVE_THRESHOLD){
		// make the azimuth motor make a 180 degree turn
		Serial.println("Up > Left");
		Serial.println();

		// if the tracker is tilted up simply move the motor down until the sensor reads the same value as the left sensor
		if (polar >= 0){
			polarLimit = false;  // the motor will move so even if the limit was reached the flag will be set to false
			
			long unsigned int steps = 0;

			digitalWrite(dirPin_pol, LOW);
			
			start = micros();

			while (upSensor.lux - leftSensor.lux > ACC_THRESHOLD) {
				
				if (steps % 1000 == 0){
					Serial.print("Steps: ");
					Serial.print(polar);
					Serial.print(" Up sens: ");
					Serial.print(upSensor.lux);
				}
				
				digitalWrite(stepPin_pol, HIGH); // step the azimuth stepper motor
				readSensor(&upSensor);           // read the up sensor
				
				end = micros();
				
				if (end - start < SPEED_DELAY)
					delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed
			

				start = micros();

				if (steps % 1000 == 0){
					Serial.print(" Left sens: ");
					Serial.println(leftSensor.lux);
					Serial.println();
				}

				digitalWrite(stepPin_pol, LOW);  // step the azimuth stepper motor
				polar--;                         // decrement the azimuth step counter

				readSensor(&leftSensor);         // read the left sensor

				end = micros();

				if (end - start < SPEED_DELAY)
					delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

				
				if (upSensor.lux - leftSensor.lux < ACC_THRESHOLD) {
					break;
				}

				if (polar <= 0) {
					// if the tracker becomes horizontal break the loop and in the next loop the azimuth motor will make a 180 degree turn
					break;
				}

				steps++;
			}
			
		} else {
			// make the azimuth motor make a 180 degree turn in the direction that leads closer to the center
			if (azimuth > 0){
				digitalWrite(dirPin_az, LOW);
				digitalWrite(dirPin_pol, LOW);

				for (long i = 0; i < AZIMUTH_LIMIT_U / 2; i++) {
					
					start = micros();

					digitalWrite(stepPin_az, HIGH);

					// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
					if ((i % 9 == 0 || i % 9 == 5) && i !=0 ) {
						digitalWrite(stepPin_pol, HIGH);  // step the polar stepper motor
					}

					end = micros();

					if (end - start < SPEED_DELAY)
						delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

					
					start = micros();

					digitalWrite(stepPin_az, LOW);

					// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
					if ((i % 9 == 0 || i % 9 == 5) && i !=0 ) {
						digitalWrite(stepPin_pol, LOW);  // step the polar stepper motor
					}

					end = micros();
					
					if (end - start < SPEED_DELAY)
						delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

					azimuth--;
				}

			} else {
				digitalWrite(dirPin_az, HIGH);
				digitalWrite(dirPin_pol, HIGH);

				for (long i = 0; i < AZIMUTH_LIMIT_U / 2; i++) {
					
					start = micros();
					
					digitalWrite(stepPin_az, HIGH);
					
					// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
					if ((i % 9 == 0 || i % 9 == 5) && i !=0 ) {
						digitalWrite(stepPin_pol, HIGH);  // step the polar stepper motor
					}

					end = micros();

					if (end - start < SPEED_DELAY)
						delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

					
					start = micros();
					
					digitalWrite(stepPin_az, LOW);
					
					// make the polar stepper motor step 2 steps every 9 steps of the azimuth stepper motor
					if ((i % 9 == 0 || i % 9 == 5) && i !=0 ) {
						digitalWrite(stepPin_pol, LOW);  // step the polar stepper motor
					}

					end = micros();

					if (end - start < SPEED_DELAY)
						delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

					azimuth++;
				}
			}
		}

	} else if (upSensor.lux - leftSensor.lux < -MOVE_THRESHOLD && polar < POLAR_LIMIT_U){
		// if the upper sensor is less than the left sensor then move the azimuth stepper motor up unless the polar limit is reached
		// if the polar limit is reached then the tracker keeps the position it is


		// move the azimuth stepper motor up
		Serial.println("Up < Left");
		Serial.println();


		digitalWrite(dirPin_pol, HIGH);

		long unsigned int steps = 0;

		polarLimit = false;  // the motor will move so even if the limit was reached the flag will be set to false

		start = micros();

		while (upSensor.lux - leftSensor.lux < -ACC_THRESHOLD) {

			if (steps % 1000 == 0){
				Serial.print("Steps: ");
				Serial.print(polar);
				Serial.print(" Up sens: ");
				Serial.print(upSensor.lux);
			}

			digitalWrite(stepPin_pol, HIGH);  // step the azimuth stepper motor
			readSensor(&upSensor);           // read the up sensor
			
			end = micros();

			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed
			

			start = micros();

			if (steps % 1000 == 0){
				Serial.print(" Left sens: ");
				Serial.println(leftSensor.lux);
				Serial.println();
			}

			digitalWrite(stepPin_pol, LOW);  // step the azimuth stepper motor
			polar++;                         // increment the azimuth step counter

			readSensor(&leftSensor);         // read the left sensor

			end = micros();

			if (end - start < SPEED_DELAY)
				delayMicroseconds(SPEED_DELAY - (end - start));  // delay to make the motor move at a constant speed

			start = micros();

			if (upSensor.lux - leftSensor.lux > -ACC_THRESHOLD) {
				break;
			}

			steps++;

			if (polar == POLAR_LIMIT_U) {
				polarLimit = true;
				break;
			}
		}
		
	} else if (!polarLimit) {
		Serial.println("Optimal position reached!");

		// now that the optimal position is reached check if the irradiance is low
		if (upSensor.lux < IRRADIANCE_THRESHOLD) {
			Serial.println("Low irradiance!!!");

			lowIrradiance = true;
		}

		delay(10000);

	} else if (polarLimit){
		Serial.println("Polar limit reached!!! Suboptimal position restricted by the frame!!! ");

		// now that a stable position is reached check if the irradiance is low
		if (upSensor.lux < IRRADIANCE_THRESHOLD) {
			Serial.println("Low irradiance!!!");

			lowIrradiance = true;
		}

		delay(10000);

	} else {
		Serial.println("Error!!!");

		delay(10000);
	}
}