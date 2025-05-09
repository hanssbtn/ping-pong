#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <LiquidCrystal_I2C.h>

#define SDA_PIN 21
#define SCL_PIN 22

// Create an instance of the ADXL345 accelerometer
LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Pins for the sensors and LED
const int soundSensorPin = 35;	// GPIO36 (VP) for analog input
const int irSensorPin = 25;		 // GPIO25 for digital input
const int ledPin = 2;					 // GPIO2 for the LED

// Thresholds
const float impactThreshold = 0; // Adjust based on testing for accelerometer
const int soundThreshold =	10;		 // Adjust based on testing for sound sensor

// State variables
bool ballTouched = false;			 // Tracks if the ball has touched the table
int playerScore = 0;						// Tracks the player's score
unsigned long ballTouchTime = 0; // Tracks the time when the ball touched the table
const unsigned long timeout = 1000; // Timeout period 

// LED blinking variables
bool blinkLed = false;					// State to indicate blinking
unsigned long blinkStartTime = 0;
const unsigned long blinkDuration = 1000; // LED blinking duration
const int blinkInterval = 50;	// Blinking interval 
bool ledState = false;					// Tracks LED state

// Debounce variables
bool lastIRState = LOW;				 // Tracks the last stable state of the IR sensor
unsigned long lastDebounceTime = 0; // Time when the last stable state was recorded
const unsigned long debounceDelay = 10; // Debounce delay

void setup() {
	Serial.begin(115200);

	// Initialize I2C communication
	Wire.begin(SDA_PIN, SCL_PIN);

	// Initialize the ADXL345 accelerometer
	if (!accel.begin()) {
		Serial.println("ADXL345 not detected. Check connections.");
		while (1);
	}
	Serial.println("ADXL345 Initialized.");

	// Set accelerometer range and data rate
	accel.setRange(ADXL345_RANGE_2_G);
	accel.setDataRate(ADXL345_DATARATE_100_HZ);

	// Initialize sensor pins
	pinMode(soundSensorPin, INPUT);
	pinMode(irSensorPin, INPUT);
	pinMode(ledPin, OUTPUT);

	// Initialize LCD
	lcd.init();
	lcd.backlight();

	// Countdown before game start
	for (int i = 3; i > 0; i--) {
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Game starts in ");
	lcd.setCursor(0, 1);
	lcd.print(i);
	delay(1000);
	}

	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("Game Started!");
	delay(1000);
	lcd.clear();

	// Turn on the LED to indicate the system is initialized
	digitalWrite(ledPin, HIGH);
}

void loop() {
	//	 Read accelerometer data
	sensors_event_t event;
	accel.getEvent(&event);

	// Compute magnitude of acceleration vector
	float magnitude = sqrt(pow(event.acceleration.x, 2) +
						 pow(event.acceleration.y, 2) +
						 pow(event.acceleration.z, 2));

	// Read sound sensor data
	int soundValue = analogRead(soundSensorPin);

	// Check if the ball has touched the table
	if (!ballTouched && magnitude > impactThreshold && soundValue > soundThreshold) {
	
	ballTouched = true;					// Set the ball touched state
	ballTouchTime = millis();		// Record the time the ball touched the table
	Serial.println("Ball has touched the table!");
	// Serial.print("Impact Magnitude: ");
	// Serial.println(magnitude);
	Serial.print("Sound Value: ");
	Serial.println(soundValue);
	}

 // Read IR sensor state
	bool currentIRState = !digitalRead(irSensorPin); // Inverted logic

	if (currentIRState != lastIRState) {
		lastDebounceTime = millis();	// Reset debounce timer
	}

	if ((millis() - lastDebounceTime) > debounceDelay) {	
		if (currentIRState == HIGH && ballTouched) {	// Only count if the ball has touched the table before
				playerScore++;
				Serial.println("Player scored a point!");
				Serial.print("Current Score: ");
				Serial.println(playerScore);
				ballTouched = false;	// Reset ball touch state

				// Start LED blinking
				blinkLed = true;
				blinkStartTime = millis();

				// Update LCD score
				lcd.clear();
				lcd.setCursor(0, 0);
				lcd.print("Score: ");
				lcd.print(playerScore);
			}
		}
	}

	lastIRState = currentIRState;	// Store last IR sensor state


	// Reset system if timeout occurs
	if (ballTouched && (millis() - ballTouchTime > timeout)) {
		Serial.println("Timeout: No goal detected. Resetting system.");
		ballTouched = false;
	}

	// Handle LED blinking
	if (blinkLed) {
		if (millis() - blinkStartTime < blinkDuration) {
		if ((millis() - blinkStartTime) % (2 * blinkInterval) < blinkInterval) {
			digitalWrite(ledPin, HIGH);
		} else {
			digitalWrite(ledPin, LOW);
		}
		} else {
			blinkLed = false;
			digitalWrite(ledPin, HIGH);
		}
	}

	// Debugging output
	Serial.print("Magnitude: ");
	Serial.print(magnitude);
	Serial.print(", Sound: ");
	Serial.print(soundValue);
	Serial.print(", IR Sensor: ");
	Serial.println(!digitalRead(irSensorPin));

	delay(10);
}