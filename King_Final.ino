#include <Servo.h>

// Motor Driver Pins (L298N or similar)
const int motorLeftForwardPin = 6;  // Left motor forward pin
const int motorLeftBackwardPin = 7; // Left motor backward pin
const int motorRightForwardPin = 8; // Right motor forward pin
const int motorRightBackwardPin = 9; // Right motor backward pin

// If using PWM-based motor control (optional)
const int enableLeftMotor = 5;   // ENA pin (L298N) for left motor speed control
const int enableRightMotor = 10; // ENB pin (L298N) for right motor speed control

// Water Pump Pin
const int waterPumpPin = 2; // Pin to control the water pump

// Servo Motor Pin (for controlling water nozzle)
const int servoPin = 3; // Pin connected to the servo motor

// Fire Sensor Pins
const int fireSensor1Pin = 11;  // Left fire sensor
const int fireSensor2Pin = 12;  // Center fire sensor
const int fireSensor3Pin = 13;  // Right fire sensor

// Servo object
Servo waterServo;  // Create a Servo object to control the water nozzle

// Fire detection flag (to track if fire was detected)
bool fireDetected = false; 

void setup() {
  // Begin serial communication for debugging
  Serial.begin(9600);

  // Initialize motor pins as output
  pinMode(motorLeftForwardPin, OUTPUT);
  pinMode(motorLeftBackwardPin, OUTPUT);
  pinMode(motorRightForwardPin, OUTPUT);
  pinMode(motorRightBackwardPin, OUTPUT);

  // Enable motor power (if using PWM to control speed)
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(enableRightMotor, OUTPUT);
  analogWrite(enableLeftMotor, 255); // Full speed for left motor (0-255)
  analogWrite(enableRightMotor, 255); // Full speed for right motor

  // Initialize water pump control pin as output
  pinMode(waterPumpPin, OUTPUT);
  digitalWrite(waterPumpPin, LOW); // Water pump OFF initially

  // Attach the servo to the pin, setting the default position to 90 degrees
  waterServo.attach(servoPin);
  waterServo.write(90); // Default nozzle position at 90 degrees (centered)

  // Initialize fire sensor pins as input
  pinMode(fireSensor1Pin, INPUT); 
  pinMode(fireSensor2Pin, INPUT);
  pinMode(fireSensor3Pin, INPUT);

  // Print message to serial monitor to indicate robot is starting
  Serial.println("Robot Starting...");

  // Start moving forward initially
  moveForward(); 
}

void loop() {
  // Read fire sensor values (LOW means fire detected)
  bool fire1 = digitalRead(fireSensor1Pin) == LOW; // Fire detected on the left sensor?
  bool fire2 = digitalRead(fireSensor2Pin) == LOW; // Fire detected on the center sensor?
  bool fire3 = digitalRead(fireSensor3Pin) == LOW; // Fire detected on the right sensor?

  // Print the fire sensor readings to the serial monitor (for debugging)
  Serial.print("Fire Sensor 1: "); Serial.print(fire1);
  Serial.print(" | Fire Sensor 2: "); Serial.print(fire2);
  Serial.print(" | Fire Sensor 3: "); Serial.println(fire3);

  // If any fire sensor detects fire
  if (fire1 || fire2 || fire3) {
    fireDetected = true;  // Set the fire detection flag
    stopMotors(); // Stop the robot before spraying water

    // Adjust Water Nozzle Direction Based on Fire Position
    if (fire1) {
      Serial.println("🔥 Fire on the LEFT! Turning Nozzle Left.");
      waterServo.write(150); // Turn nozzle left if fire is on the left
    } 
    else if (fire2) {
      Serial.println("🔥 Fire in the CENTER! Keeping Nozzle Centered.");
      waterServo.write(90);  // Keep nozzle centered if fire is in the center
    } 
    else if (fire3) {
      Serial.println("🔥 Fire on the RIGHT! Turning Nozzle Right.");
      waterServo.write(28);  // Turn nozzle right if fire is on the right
    }

    // Allow the servo to adjust its position
    delay(500);

    // Turn ON the water pump
    digitalWrite(waterPumpPin, HIGH);
    Serial.println("🚿 Water Pump Activated!");

  } else {
    // If no fire is detected, turn OFF the water pump
    if (fireDetected) {
      Serial.println("✅ Fire Cleared! Stopping Water Pump.");
      digitalWrite(waterPumpPin, LOW); // Turn OFF water pump
      fireDetected = false; // Reset fire flag
    }

    // Continue moving forward if no fire is detected
    Serial.println("No Fire Detected. Continuing...");
    moveForward();
  }

  delay(100); // Small delay to ensure stable readings and actions
}

// Function to move the robot forward
void moveForward() {
  Serial.println("🚗 Moving Forward...");

  // Set motor pins to move forward
  digitalWrite(motorLeftForwardPin, HIGH);
  digitalWrite(motorLeftBackwardPin, LOW);
  digitalWrite(motorRightForwardPin, HIGH);
  digitalWrite(motorRightBackwardPin, LOW);
}

// Function to stop the robot
void stopMotors() {
  Serial.println("🛑 Stopping Motors...");

  // Stop both motors by setting both forward and backward pins to LOW
  digitalWrite(motorLeftForwardPin, LOW);
  digitalWrite(motorLeftBackwardPin, LOW);
  digitalWrite(motorRightForwardPin, LOW);
  digitalWrite(motorRightBackwardPin, LOW);
}
