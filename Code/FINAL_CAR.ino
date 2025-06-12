#include <Robojax_L298N_DC_motor.h>
#include "BluetoothSerial.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Motor 1 (OUT1 and OUT2) settings
#define IN1 4
#define IN2 5
#define ENA  2 // this pin must be PWM enabled pin

// Motor 2 (OUT3 and OUT4) settings
#define IN3 28
#define IN4 19
#define ENB 23 // this pin must be PWM enabled pin

const int CW = 1; // Clockwise direction
const int CCW = 0; // Counter-clockwise direction

#define motor1 1 // OUT1 and OUT2 combined
#define motor2 2 // OUT3 and OUT4 combined

// Create motor object for L298N
Robojax_L298N_DC_motor motor(IN1, IN2, ENA, IN3, IN4, ENB, true);

// Initialize Bluetooth serial object
BluetoothSerial SerialBT;

// Initialize LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD address 0x27, 16x2 display

// Define pins for ultrasonic sensors
#define TRIG_FRONT 26
#define ECHO_FRONT 27
#define TRIG_REAR 32
#define ECHO_REAR 33

// Define pins for IR sensors
#define IR_LEFT 13
#define IR_RIGHT 25

// Define buzzer pin
#define BUZZER 12

// Speed Sensor Pin
#define SPEED_SENSOR 4

volatile unsigned long lastPulseTime = 0;
volatile int pulseCount = 0;

// Variables for speed and temperature
float currentSpeed = 0.0;
int randomTemperature = 0;
unsigned long startTime = 0;

String bluetoothCommand = "";

void IRAM_ATTR countPulse() {
  unsigned long now = millis();
  if (now - lastPulseTime > 5) { // Debounce (minimum time between pulses)
    pulseCount++;
    lastPulseTime = now;
  }
}

void setup() {
  Serial.begin(115200); // Serial monitor for debugging
  SerialBT.begin("ESP32_BT_MOTOR"); // Set the Bluetooth name
  Serial.println("Bluetooth Started. Waiting for commands...");

  // Initialize LCD
  lcd.init();
  lcd.backlight();

  // Initialize ultrasonic sensors
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_REAR, OUTPUT);
  pinMode(ECHO_REAR, INPUT);

  // Initialize IR sensors
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Initialize buzzer
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  // Initialize Speed Sensor
  pinMode(SPEED_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR), countPulse, FALLING);

  motor.begin(); // Initialize motors
  randomSeed(analogRead(0)); // Seed for random temperature generation
}

long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
  if (duration == 0) return 999; // If no echo received, return 999
  long distance = duration / 29 / 2; // Convert time to distance in cm
  return distance;
}

void calculateSpeed() {
  static unsigned long lastCalcTime = 0;
  unsigned long now = millis();
  unsigned long elapsedTime = now - lastCalcTime;

  // Calculate speed: pulses per second -> speed
  if (elapsedTime >= 1000) { // Every 1 second
    currentSpeed = pulseCount * 10.0; // Assuming each pulse = 10 cm of movement
    pulseCount = 0;
    lastCalcTime = now;
  }
}

void updateTemperature() {
  randomTemperature = random(23, 24); // Generate random temperature between 23 and 26
}

void moveMotors(int speed) {
  motor.rotate(motor1, speed, CW); // Motor 1 forward
  motor.rotate(motor2, speed, CW); // Motor 2 forward
}

void stopMotors() {
  motor.brake(motor1);
  motor.brake(motor2);
  startTime = 0; // Reset start time
}

bool isStopped = false;

void loop() {
  // Measure distances using ultrasonic sensors
  long distanceFront = measureDistance(TRIG_FRONT, ECHO_FRONT);
  long distanceRear = measureDistance(TRIG_REAR, ECHO_REAR);

  // Debugging: Print distance values to the serial monitor
  Serial.print("Front Distance: ");
  Serial.println(distanceFront);
  Serial.print("Rear Distance: ");
  Serial.println(distanceRear);

  // Read IR sensor values
  int leftIR = digitalRead(IR_LEFT);
  int rightIR = digitalRead(IR_RIGHT);

  // Update speed and temperature
  calculateSpeed();
  updateTemperature();

  // Bluetooth control
  if (SerialBT.available()) {
    bluetoothCommand = SerialBT.readStringUntil('\n');
    bluetoothCommand.trim(); // Remove extra spaces or newline characters
  }

  // Handle Bluetooth commands
  if (bluetoothCommand.startsWith("F")) { // Move forward
  if (distanceFront >= 8) { // Only move if safe
    int speed = getSpeedFromCommand(bluetoothCommand);

    // Check if this is the first time receiving the command
    if (startTime == 0) {
      startTime = millis(); // Record the start time
      motor.rotate(motor1, speed*0.6, CW); // Start motors at full speed
      motor.rotate(motor2, speed*0.6, CW);
      isStopped = false; 
    } else if (millis() - startTime > 1000) { // After 2 seconds
      motor.rotate(motor1, speed * 0.5, CW); // Reduce speed to 50%
      motor.rotate(motor2, speed * 0.5, CW);
      isStopped = false; 
    }
  } else {
    stopMotors();
    isStopped = true; 
  }
} else if (bluetoothCommand.startsWith("B")) { // Move backward
  if (distanceRear >= 8) { // Only move if safe
    int speed = getSpeedFromCommand(bluetoothCommand);

    // Check if this is the first time receiving the command
    if (startTime == 0) {
      startTime = millis(); // Record the start time
      motor.rotate(motor1, speed, CCW); // Start motors at full speed
      motor.rotate(motor2, speed, CCW);
      isStopped = false; 
    } else if (millis() - startTime > 2000) { // After 2 seconds
      motor.rotate(motor1, speed * 0.5, CCW); // Reduce speed to 50%
      motor.rotate(motor2, speed * 0.5, CCW);
      isStopped = false; 
    }
  } else {
    stopMotors();
    isStopped = true; 
  }
}

if (isStopped && distanceFront >= 8) {
    moveMotors(30*0.6); // Default speed to resume forward motion
    isStopped = false; // Reset stop flag
    Serial.println("Resuming forward motion after clearance detected.");
  }


  // Adjust display and alerts based on distance
  lcd.clear(); // Clear LCD before updating
  digitalWrite(BUZZER, LOW); // Turn off buzzer

  if (distanceFront < 8) {
    stopMotors();
    lcd.setCursor(0, 0);
    lcd.print("STOP!");
    lcd.setCursor(0, 1);
    lcd.print(distanceFront);
    lcd.print(" cm");
    digitalWrite(BUZZER, HIGH); // Long buzzer alert
    isStopped = true; 
  } else if (distanceRear < 8) {
    lcd.setCursor(0, 0);
    lcd.print("Safety Alert!");
    lcd.setCursor(0, 1);
    lcd.print(distanceRear);
    lcd.print(" cm");
    digitalWrite(BUZZER, HIGH); // Long buzzer alert
    isStopped = true; 
  }else if(distanceRear < 25){
    lcd.setCursor(0, 0);
    lcd.print("Safety alert!");
    lcd.print(" cm");
    lcd.setCursor(0, 1);
    lcd.print(distanceRear);
  } else if ((distanceFront >= 8 && distanceFront < 25) || (distanceRear >= 8 && distanceRear < 25)) {
    moveMotors(30*0.5); 
    lcd.setCursor(0, 0);
    lcd.print("Alert!");
    //lcd.setCursor(0, 0);
    lcd.print("S:");
    lcd.print(11);
    lcd.print(" cm/s");
    if (distanceFront >= 8 && distanceFront < 25) {
      lcd.setCursor(0, 1);
      lcd.print("Front: ");
      lcd.print(distanceFront);
      lcd.print(" cm");
    } else {
      lcd.setCursor(0, 1);
      lcd.print("Rear: ");
      lcd.print(distanceRear);
      lcd.print(" cm");
    }
    Serial.println("Safety alert activated: Reduced speed!");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Speed: ");
    lcd.print(20);
    lcd.print(" cm/s");
    lcd.setCursor(0, 1);
    lcd.print("Temp: ");
    lcd.print(randomTemperature);
    lcd.print(" C");
  }

  // Blind spot alerts
  if (leftIR == LOW) {
    lcd.setCursor(0, 0);
    lcd.print("Car on Left!");
    digitalWrite(BUZZER, HIGH);
  } else if (rightIR == LOW) {
    lcd.setCursor(0, 0);
    lcd.print("Car on Right!");
    digitalWrite(BUZZER, HIGH);
  }

  delay(500); // Update readings every 500ms
}


// Function to parse speed from the command
int getSpeedFromCommand(String command) {
  int speed = 50; // Default speed
  int index = command.indexOf(' ');
  if (index != -1) {
    speed = command.substring(index + 1).toInt();
    speed = constrain(speed, 0, 100); // Ensure speed is between 0 and 100
  }
  return speed;
}