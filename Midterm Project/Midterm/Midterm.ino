/*
 * Autonomous Robotic Systems - Midterm Project submitted by Anish Anand
 * Electric Skateboard Remote Control
 */

#include <Servo.h>
#include <Stepper.h>
#include <Wire.h> // For I2C Communication

// --- PIN DEFINITIONS ---

// Shift Register (74HC595)
const int LATCH_PIN = 4;  // ST_CP
const int CLOCK_PIN = 5;  // SH_CP
const int DATA_PIN = 6;   // DS

// DC Motor (Speed)
const int DC_MOTOR_PIN = 3; // PWM Pin

// Servo (Speedometer)
const int SERVO_PIN = 9;

// Joystick
const int JOY_X_PIN = A0; // Steering in Drone Mode
const int JOY_Y_PIN = A1; // Speed in Drone Mode
const int JOY_BTN_PIN = 2; // Toggle Mode

// Ultrasonic Sensor
const int TRIG_PIN = 7;
const int ECHO_PIN = 8;

// Lighting Controls
const int LDR_PIN = A2;
const int POT_PIN = A3;

// Stepper Motor (H-Bridge)
const int STEPS_PER_REV = 200; 
Stepper steeringStepper(STEPS_PER_REV, 10, 11, 12, 13); 

// --- ADXL345 SETTINGS ---
const int ADXL_ADDR = 0x53; // I2C Address 
const int POWER_CTL = 0x2D; // Power Control Register
const int DATA_FORMAT = 0x31;
const int DATAX0 = 0x32;    // Starting Data Register

// --- GLOBAL VARIABLES ---

Servo speedServo;
int currentSpeedLevel = 0; 
int motorPWM = 0;          
bool droneMode = false;    // False = Tilt Mode, True = Drone Mode
unsigned long lastDebounceTime = 0;
int buttonState;
int lastButtonState = HIGH;

// 7-Segment Hex Codes (Common Cathode)
// Digits 0-9 
byte digitCodes[] = {
  0b00111111, // 0
  0b00000110, // 1
  0b01011011, // 2
  0b01001111, // 3
  0b01100110, // 4
  0b01101101, // 5
  0b01111101, // 6
  0b00000111, // 7
  0b01111111, // 8
  0b01101111  // 9
};

void setup() {
  Serial.begin(9600);

  // Pin Modes
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(DC_MOTOR_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(JOY_BTN_PIN, INPUT_PULLUP);
  
  speedServo.attach(SERVO_PIN);
  steeringStepper.setSpeed(60); 

  // --- ADXL345 INITIALIZATION ---
  Wire.begin();
  
  // Put ADXL345 into Measurement Mode
  writeRegister(ADXL_ADDR, POWER_CTL, 0x08); 
  
  // Optional: Set range to +/- 4g (usually better for gestures)
  // Register 0x31 (DATA_FORMAT): 0x01 is +/- 4g
  writeRegister(ADXL_ADDR, DATA_FORMAT, 0x01);

  // Initial State
  updateDisplays(0, false);
}

void loop() {
  handleModeSwitch(); 

  int speedChangeDirection = 0; 
  bool isFalling = false;
  
  // --- READ ADXL345 ---
  int16_t ax, ay, az;
  readAccel(ax, ay, az);

  // --- FALL DETECTION LOGIC ---
  if (abs(ax) > 500 || abs(ay) > 500) { 
    isFalling = true; 
  }

  if (isFalling) {
    currentSpeedLevel = 0; // Emergency Stop
  } else {
    
    if (droneMode) {
      // --- DRONE MODE ---
      int joyY = analogRead(JOY_Y_PIN);
      int joyX = analogRead(JOY_X_PIN);
      
      // Speed (Y-Axis)
      if (joyY > 600) speedChangeDirection = 1;
      else if (joyY < 400) speedChangeDirection = -1;
      
      if (joyX > 800) steeringStepper.step(10); 
      if (joyX < 200) steeringStepper.step(-10);

    } else {
      // --- TILT MODE (Normal) ---
      if (ax > 60) { 
        speedChangeDirection = 1; 
      } else if (ax < -60) { 
        speedChangeDirection = -1; 
      }
    }

    // Apply Speed Change
    static int updateDelay = 0;
    if (speedChangeDirection != 0) {
      updateDelay++;
      if (updateDelay > 5) { // Slow down the rate of change
        currentSpeedLevel += speedChangeDirection;
        updateDelay = 0;
      }
    }
    
    currentSpeedLevel = constrain(currentSpeedLevel, 0, 9);
  }

  // --- OBSTACLE DETECTION ---
  long distance = readDistance();
  if (distance < 50 && distance > 0) { 
    int maxSafeSpeed = map(distance, 5, 50, 0, 9);
    if (currentSpeedLevel > maxSafeSpeed) {
      currentSpeedLevel = maxSafeSpeed;
    }
  }

  // --- DRIVE OUTPUTS ---
  motorPWM = map(currentSpeedLevel, 0, 9, 0, 255);
  analogWrite(DC_MOTOR_PIN, motorPWM);
  
  int servoAngle = map(currentSpeedLevel, 0, 9, 0, 180);
  speedServo.write(servoAngle); 

  // --- LIGHTING ---
  bool lightOn = calculateLightStatus();
  updateDisplays(currentSpeedLevel, lightOn);

  delay(50); 
}

// --- HELPER FUNCTIONS ---

void writeRegister(int deviceAddress, int regAddress, int val) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(val);
  Wire.endTransmission();
}

void readAccel(int16_t &ax, int16_t &ay, int16_t &az) {
  Wire.beginTransmission(ADXL_ADDR);
  Wire.write(DATAX0); 
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL_ADDR, 6, true);
  
  ax = Wire.read() | (Wire.read() << 8);
  ay = Wire.read() | (Wire.read() << 8);
  az = Wire.read() | (Wire.read() << 8);
}

void handleModeSwitch() {
  int reading = digitalRead(JOY_BTN_PIN);
  if (reading != lastButtonState) lastDebounceTime = millis();
  
  if ((millis() - lastDebounceTime) > 50) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) droneMode = !droneMode;
    }
  }
  lastButtonState = reading;
}

long readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2; 
}

bool calculateLightStatus() {
  int lightVal = analogRead(LDR_PIN);
  int potVal = analogRead(POT_PIN);
  
  // Potentiometer acts as the reference threshold
  int threshold = potVal; 
  return (lightVal < threshold);
}

void updateDisplays(int number, bool lightStatus) {
  byte segments = digitCodes[number];
  if (lightStatus) segments |= 0b10000000; 

  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, segments);
  digitalWrite(LATCH_PIN, HIGH);
}