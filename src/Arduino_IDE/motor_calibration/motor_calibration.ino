#define dirPin 8
#define stepPin 9
#define stepsPerRevolution 200

#define dirPin2 10
#define stepPin2 11

void setup() {

  Serial.begin(9600);
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
}

unsigned long direction;
unsigned long azimuth;


bool isAzLimit(unsigned long current_steps){
  if (current_steps <= 51200 && current_steps >= 0)
    return true;
  else
    return false; 
}

bool isDirLimit(unsigned long current_steps){
  if (current_steps <= 56000 && current_steps >= 0)
    return true;
  else
    return false; 
}


void loop() {
  // Set the spinning direction clockwise:
  digitalWrite(dirPin, HIGH);
  digitalWrite(dirPin2, HIGH);


  for (unsigned int i = 0; i < 25600; i++) {
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(1000);
  }


  // Spin the stepper motor 1 revolution slowly:
  for (unsigned int i = 0; i < stepsPerRevolution * 32 * 8.75; i++) {

    // These four lines result in 1 step:
    if ((i % 9 == 0 || i % 9 == 5) && i !=0 ){
      // for every 9 steps of the direction motor make one step on the azimuth motor
      digitalWrite(stepPin2, HIGH);
    }
    digitalWrite(stepPin, HIGH);
    
    delayMicroseconds(400);
    
    if ((i % 9 == 0 || i % 9 == 5) && i !=0 ){
      digitalWrite(stepPin2, LOW);
    }
    digitalWrite(stepPin, LOW);
    
    delayMicroseconds(400);
  }

  delay(5000);

  digitalWrite(dirPin, LOW);
  digitalWrite(dirPin2, LOW);


  // Spin the stepper motor 1 revolution slowly:
  for (unsigned int i = 0; i < stepsPerRevolution * 32 * 8.75; i++) {

    // These four lines result in 1 step:
    if ((i % 9 == 0 || i % 9 == 5) && i !=0 ){
      // for every 9 steps of the direction motor make one step on the azimuth motor
      digitalWrite(stepPin2, HIGH);
    }
    digitalWrite(stepPin, HIGH);
    
    delayMicroseconds(400);
    
    if ((i % 9 == 0 || i % 9 == 5) && i !=0 ){
      digitalWrite(stepPin2, LOW);
    }
    digitalWrite(stepPin, LOW);
    
    delayMicroseconds(400);
  }

  for (unsigned int i = 0; i < 25600; i++) {
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(400);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(400);
  }

  delay(5000);

  
}
