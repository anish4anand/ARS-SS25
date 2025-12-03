//==== Include Libraries ====
#include <SPI.h>
#include <RF24.h>
#include <Stepper.h>
#include <IRremote.h>

//==== Define Pins ====
#define STEPS_PER_REV 100
Stepper stepper(STEPS_PER_REV, A0, A1, A2, A3);

#define IR_RECEIVE_PIN 6

const int dataPin = 2;
const int clockPin = 4;
const int latchPin = 7;

#define BUTTON_PIN A4

//==== Global Variables ====
bool irResetRequested = false;
int currentBreachDisplay = 0;
unsigned long lastReceivedTime = 0;
unsigned long dataTimeout = 3000;
bool steppingEnabled = false;

//==== RF Communication ====
RF24 radio(9, 8);
const byte address[6] = "00001";

//==== Patterns for 8x8 Matrix ====
const byte numbers[5][8] = {
  {B00111100, B01000010, B01000110, B01001010, B01010010, B01100010, B00111100, B00000000},
  {B00011000, B00111000, B00011000, B00011000, B00011000, B00011000, B01111110, B00000000},
  {B00111100, B01000010, B00000010, B00000100, B00001000, B00010000, B01111110, B00000000},
  {B00111100, B01000010, B00000010, B00011100, B00000010, B01000010, B00111100, B00000000},
  {B00000100, B00001100, B00010100, B00100100, B01000100, B01111110, B00000100, B00000000}
};

const byte alertPattern[8] = {
  B11111111,
  B10000001,
  B10000001,
  B10000001,
  B10000001,
  B10000001,
  B10000001,
  B11111111
};

//==== Struct for Receiving Data ====
struct SensorData {
  int16_t breachCount;
  uint16_t ldrValue;
  float ultrasonicDistance;
  bool movementDetected;
  float temperature;
  bool soundDetected;
  float humidity;
  float accelX;
  float accelY;
  float accelZ;
  int16_t year;
  int16_t month;
  int16_t day;
  int8_t hour;
  int8_t minute;
  int8_t second;
};

//==== Setup ====
void setup() {
  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  // RF Setup
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(60);
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.startListening();

  stepper.setSpeed(60);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  Serial.println("Dojo Ready – Waiting for Ninja");
}

//==== Main Loop ====
void loop() {
  handleIR();
  handleRF();
  refreshDisplay();

  if (steppingEnabled) {
    if (millis() - lastReceivedTime <= dataTimeout) {
      stepper.step(1);
    } else {
      steppingEnabled = false;
    }
  }
}

//==== IR Remote Handler ====
void handleIR() {
  if (IrReceiver.decode()) {
    if (IrReceiver.decodedIRData.decodedRawData == 0xF30CFF00) {
      irResetRequested = true;
      Serial.println("IR Reset Queued");
    }
    IrReceiver.resume();
  }
}

//==== RF Receiver ====
void handleRF() {
  if (radio.available()) {
    SensorData data;
    radio.read(&data, sizeof(data));

    Serial.println("Data Received from LockerNinja:");
    Serial.print("Temp: "); Serial.println(data.temperature);
    Serial.print("Humidity: "); Serial.println(data.humidity);
    Serial.print("Accel X: "); Serial.println(data.accelX);
    Serial.print("Accel Y: "); Serial.println(data.accelY);
    Serial.print("Accel Z: "); Serial.println(data.accelZ);

    // Fix: original PDF had wrong syntax "if abs("
    if (abs(data.accelY) > 0.80 || abs(data.accelX) > 0.80) {
      Serial.println("--------!!! Ninja Kidnapped !!!----------");
    }

    Serial.print("LDR Value: "); Serial.println(data.ldrValue);
    Serial.print("Breach Count: "); Serial.println(data.breachCount);
    Serial.print("Ultrasonic Distance: "); Serial.println(data.ultrasonicDistance);
    Serial.print("Movement Detected: "); Serial.println(data.movementDetected ? "Yes" : "No");
    Serial.print("Sound Detected: "); Serial.println(data.soundDetected ? "Yes" : "No");

    if (data.breachCount >= 4) {
      Serial.println("MAX BREACH — ALERT");
    }

    currentBreachDisplay = data.breachCount;
    lastReceivedTime = millis();
    steppingEnabled = true;

    bool resetTriggered = false;

    // IR reset
    if (irResetRequested) {
      const char ack[] = "1";
      radio.writeAckPayload(1, ack, sizeof(ack));

      Serial.println("Sent ack: 1 (IR Reset)");
      currentBreachDisplay = 0;
      irResetRequested = false;
      resetTriggered = true;
    }

    // Button reset
    if (digitalRead(BUTTON_PIN) == LOW) {
      const char ack[] = "1";
      radio.writeAckPayload(1, ack, sizeof(ack));

      Serial.println("Sent ack: 1 (Button Reset)");
      currentBreachDisplay = 0;
      resetTriggered = true;
    }

    if (!resetTriggered) {
      const char ack[] = "0";
      radio.writeAckPayload(1, ack, sizeof(ack));
    }

    Serial.println("----------------------------------");
  }
}

//==== LED Matrix Refresh ====
void refreshDisplay() {
  for (byte row = 0; row < 8; row++) {
    digitalWrite(latchPin, LOW);

    byte rowData = (1 << (7 - row));
    byte columnData;

    if (currentBreachDisplay >= 0 && currentBreachDisplay <= 3) {
      columnData = ~reverseBits(numbers[currentBreachDisplay][row]);
    } else {
      columnData = ~reverseBits(alertPattern[row]);
    }

    shiftOut(dataPin, clockPin, LSBFIRST, columnData);
    shiftOut(dataPin, clockPin, LSBFIRST, rowData);

    digitalWrite(latchPin, HIGH);
    delayMicroseconds(80);
  }
}

//==== Bit Reversal ====
byte reverseBits(byte b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}
