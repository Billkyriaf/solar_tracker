#include <BH1750.h>
#include <Wire.h>

#define PIN4 4
#define PIN3 3

BH1750 sensor3;
BH1750 sensor4;

void setup() {
  pinMode(PIN3, OUTPUT);
  pinMode(PIN4, OUTPUT);

  Serial.begin(9600);

	// Initialize the I2C communication:
	Wire.begin();

  digitalWrite(PIN3, HIGH);
  digitalWrite(PIN4, LOW);

  sensor3.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &Wire);
  sensor4.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);

}

float light3;
float light4;

void loop() {
  light3 = sensor3.readLightLevel();
  light4 = sensor4.readLightLevel();

	Serial.print("Light3: ");
	Serial.print(light3);
  Serial.print("   Light4: ");
  Serial.println(light4);
	delay(1000);

}
