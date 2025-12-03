//========= Locker Ninja (Transmitter) ===============//
// Team 8: Vigneshwar Subbammal Rohini Murali, Matr. No.: 3766396
//         Anish Anand, Matr. No.: 3765957

//============ Include Libraries ============//
#include <Wire.h>
#include "uRTCLib.h"
#include <dht.h>
#include <SPI.h>
#include <RF24.h>
#include <Servo.h>

//============ Define Pins ============//
#define DHT11_PIN 2
#define LDR_PIN A0
#define TRIG_PIN 10
#define ECHO_PIN 4
#define MAX_BREACH_COUNT 4
#define DISTANCE_THRESHOLD 50
#define SOUND_PIN 3

dht DHT;
uRTCLib rtc(0x68);
int ADXL345 = 0x53;  // I2C address of ADXL345

//============ RF24 ============//
RF24 radio(9, 8);    // CE, CSN
const byte address[6] = "00001";

//============ Servos ============//
Servo servoStatus;
Servo servoLevel;

//============ Data Structure ============//
#pragma pack(push, 1)
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
#pragma pack(pop)

SensorData data;

//============ Global Variables ============//
uint16_t lastLDR = 0;
uint8_t breachCount = 0;
bool maxBreachTriggered = false;
bool emailTriggered = false;

//============ Setup ============//
void setup() {
  Serial.begin(9600);

  Wire.begin();
  URTCLIB_WIRE.begin();

  pinMode(LDR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SOUND_PIN, INPUT);

  // ADXL345 Setup
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2D);
  Wire.write(8);  // Measurement mode
  Wire.endTransmission();

  // RF24 Setup
  radio.begin();
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(60);
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.stopListening();

  // Servos
  servoStatus.attach(5);
  servoLevel.attach(6);
  servoStatus.write(0);
  servoLevel.write(0);
}

//============ Loop ============//
void loop() {
  readAllSensors();
  sendDataToDojo();
  checkEmailTrigger();

  delay(2000);
}

//============ Sensor Aggregation ============//
void readAllSensors() {
  readRTC();
  readDHT11();
  readAccelerometer();
  readLDR();
  readUltrasonic();
  readSoundSensor();
}

//============ RTC ============//
void readRTC() {
  rtc.refresh();

  data.year = rtc.year();
  data.month = rtc.month();
  data.day = rtc.day();
  data.hour = rtc.hour();
  data.minute = rtc.minute();
  data.second = rtc.second();

  Serial.print("Date & Time: ");
  Serial.print(data.year); Serial.print("-");
  Serial.print(data.month); Serial.print("-");
  Serial.print(data.day); Serial.print(" ");
  Serial.print(data.hour); Serial.print(":");
  Serial.print(data.minute); Serial.print(":");
  Serial.println(data.second);
}

//============ Email Trigger ============//
void checkEmailTrigger() {
  if (!emailTriggered &&
      data.year == 25 &&
      data.month == 7 &&
      data.day == 17 &&
      data.hour == 12 &&
      data.minute == 42 &&
      data.second < 3) {

    Serial.println("EMAIL_TRIGGER");
    emailTriggered = true;
  }
}

//============ DHT11 ============//
void readDHT11() {
  DHT.read11(DHT11_PIN);
  data.temperature = DHT.temperature;
  data.humidity = DHT.humidity;
}

//============ Accelerometer ============//
void readAccelerometer() {
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32);
  Wire.endTransmission(false);

  Wire.requestFrom(ADXL345, 6, true);

  data.accelX = (Wire.read() | (Wire.read() << 8)) / 256.0;
  data.accelY = (Wire.read() | (Wire.read() << 8)) / 256.0;
  data.accelZ = (Wire.read() | (Wire.read() << 8)) / 256.0;
}

//============ LDR ============//
void readLDR() {
  int ldr = analogRead(LDR_PIN);
  data.ldrValue = ldr;

  if (ldr > 800 && lastLDR <= 800 && breachCount < MAX_BREACH_COUNT) {
    breachCount++;
    if (breachCount == MAX_BREACH_COUNT) {
      maxBreachTriggered = true;
    }
  }

  lastLDR = ldr;
  data.breachCount = breachCount;

  servoStatus.write(breachCount == 0 ? 0 : 90);

  switch (breachCount) {
    case 0: servoLevel.write(0); break;
    case 1: servoLevel.write(45); break;
    case 2: servoLevel.write(90); break;
    case 3: servoLevel.write(135); break;
    default: servoLevel.write(180); break;
  }
}

//============ Ultrasonic ============//
void readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 20000);
  float distance = duration * 0.034 / 2.0;

  data.ultrasonicDistance = distance;
  data.movementDetected = (distance > 0 && distance < DISTANCE_THRESHOLD);

  Serial.print("Ultrasonic Distance: ");
  Serial.print(distance);
  Serial.print(" cm | Movement: ");
  Serial.println(data.movementDetected ? "Yes" : "No");
}

//============ Sound Sensor ============//
void readSoundSensor() {
  int soundState = digitalRead(SOUND_PIN);
  data.soundDetected = (soundState == HIGH);

  Serial.print("Sound Detected: ");
  Serial.println(data.soundDetected ? "Yes" : "No");
}

//============ Send Data to Dojo (Receiver) ============//
void sendDataToDojo() {
  bool sent = radio.write(&data, sizeof(data));
  Serial.println(sent ? "Data sent to Dojo" : "Send failed");

  if (radio.isAckPayloadAvailable()) {
    char ack[32] = "";
    radio.read(ack, sizeof(ack));

    Serial.print("AckPayload from Dojo: ");
    Serial.println(ack);

    // Reset breach count
    if (strcmp(ack, "1") == 0) {
      breachCount = 0;
      maxBreachTriggered = false;

      Serial.println("Breach count reset by Dojo");

      servoStatus.write(0);
      servoLevel.write(0);
    }
  } else {
    Serial.println("No ackPayload received.");
  }
}
