#include <smartTimer.h>
#include <math.h>
#include <Wire.h>
#include <LSM303.h>

LSM303 accMag;

// Declaration of pins for sensors and actuators

const char UART_PIN_TX = 18;
const char UART_PIN_RX = 19;

const char US_PIN_IN[] = { 45, 47, 49, 51 }; // Connected to ECHO pins of ultrasound sensors
const char US_PIN_OUT[] = { 46, 48, 50, 52 }; // Connected to TRIGGER pins of ultrasound sensors

const char IR_PIN[] = { 8, 7, 6, 5 };

// Pins 6 & 7 for left obstacle DC Motor and Pins 8 & 9 for right obstacle DC Motor
const char DC_PIN_LEFT[] = { 6, 8 };
const char DC_PIN_RIGHT[] = { 7, 9 };

// Declaration of poll periods (in ms)

const int UART_WRITE_PERIOD = 500;
const int UART_READ_PERIOD = 500;
const int IMU_READ_PERIOD = 50;
const int SENSOR_READ_PERIOD = 1000;
const int DC_WRITE_PERIOD = 1000;

// Declaration of packet codes 

const char SYN = 0;
const char SYNACK = 1;
const char ACK = 2;
const char DATA = 3;

// Declaration of indices

const char SYNACK_INDEX = 0;

const char PACKET_CODE_INDEX = 0;
const char PAYLOAD_SIZE_INDEX = 1;

const char MAG_HEADING_INDEX = 0;
const char BAR_VALUE_INDEX = 1;

static char US_LEFT = 0, US_FRONT_TOP = 1, US_FRONT_BOTTOM = 2, US_RIGHT = 3;
static char IR_LEFT = 0, IR_FRONT_LEFT = 1, IR_FRONT_RIGHT = 2, IR_RIGHT = 3;
static char DC_LEFT = 0, DC_RIGHT = 1;

// Declaration of thresholds

const float ACCELERATION_THRESHOLD = 19500;
const float DECELERATION_THRESHOLD = 14000;

const float MAG_THRESHOLD = 180;
const float MAG_NORMALIZER = 360;

const float US_THRESHOLD_DISTANCE = 100;
const float US_MINIMUM_DISTANCE = 10;
const float IR_THRESHOLD_DISTANCE = 70;
const float IR_MINIMUM_DISTANCE = 10;

long lastAccelerationTime;
long pedoValue = 0;
bool flag = false;

float usValue[] = { NAN, NAN, NAN, NAN };
float irValue[] = { NAN, NAN, NAN, NAN };
float imuValue[] = { NAN, NAN };

float barEg = NAN;

char sendBuffer[15]; // For transmitting data onto Arduino's USART

// For Handshake protocol between Arduino and RPi

boolean is_SYN_Received = false;
boolean is_ACK_Received = false;
unsigned long synAckSendTime = millis();

CSmartTimer *timer1;
CSmartTimer *timer2;

void initializePins();
void initializeTimers();
void uartWrite();
void uartRead();
void imuRead();
void imuAccRead();
void imuMagRead();
void imuBarRead();
void usRead();
void irRead();
void dcWrite();
void dcTurnOff(int id);
void dcRotateLeft(int id);
void dcRotateRight(int id);

float calculateMagnitude(float x, float y, float z);
unsigned long pulse(int triggerPin, int echoPin);
float calculateDistance(unsigned long duration);

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial1.flush();

  Wire.begin();
  while (!accMag.init()) {
    Serial.println("Failed to autodetect accmag!");
  }
  accMag.enableDefault();
  Serial.println("Successful");
  // Serial.flush();

  accMag.m_min = (LSM303::vector<int16_t>){-1832, -2342, -1505};
  accMag.m_max = (LSM303::vector<int16_t>){+2278, +2042, +2726};
  
  initializePins();
  initializeTimers();
}

void loop() {
  // Do nothing
}

void initializePins() {
  for (int i = 0; i < sizeof(US_PIN_IN) / sizeof(US_PIN_IN[0]); i++) {
    pinMode(US_PIN_IN[i], INPUT);
    pinMode(US_PIN_OUT[i], OUTPUT);
  }
  
  for (int i = 0; i < sizeof(DC_PIN_LEFT) / sizeof(DC_PIN_LEFT[0]); i++) {
    pinMode(DC_PIN_LEFT[i], OUTPUT);
    pinMode(DC_PIN_RIGHT[i], OUTPUT);
  }
}

void initializeTimers() {
  timer1 = new CSmartTimer(STIMER1);
  timer2 = new CSmartTimer(STIMER2);
  
  timer1 -> attachCallback(uartWrite, UART_WRITE_PERIOD);
  timer1 -> attachCallback(uartRead, UART_READ_PERIOD);
  timer1 -> attachCallback(imuRead, IMU_READ_PERIOD);

  timer2 -> attachCallback(irRead, SENSOR_READ_PERIOD);
  timer2 -> attachCallback(usRead, SENSOR_READ_PERIOD);
  timer2 -> attachCallback(dcWrite, DC_WRITE_PERIOD);
  
  timer1 -> startTimer();
  timer2 -> startTimer();
}

void uartWrite() {
  char payloadSize = 0;
  char packetCode = DATA;
  char checksum = 0;

  if (is_SYN_Received == true && is_ACK_Received == false) {
    if (millis() - synAckSendTime >= 1000) { 
      sendBuffer[SYNACK_INDEX] = SYNACK;
      Serial1.write(sendBuffer, 1);
      Serial1.flush();
      Serial.println("Sent SYNACK to RPi");
      Serial.flush();
      synAckSendTime = millis();
    }
  } else if (is_ACK_Received == true && imuValue[MAG_HEADING_INDEX] == imuValue[MAG_HEADING_INDEX]) {
    char pedoOffset = 2;
    memcpy(sendBuffer + pedoOffset, &pedoValue, sizeof(pedoValue));
    payloadSize += sizeof(pedoValue);

    char imuOffset = pedoOffset + payloadSize;
    memcpy(sendBuffer + imuOffset, imuValue, sizeof(imuValue));
    payloadSize += sizeof(imuValue);
     
    for (int j = pedoOffset; j <= payloadSize + 1; j++) {
      checksum ^= sendBuffer[j];
    }

    sendBuffer[PACKET_CODE_INDEX] = packetCode;
    sendBuffer[PAYLOAD_SIZE_INDEX] = payloadSize;
    sendBuffer[payloadSize + 2] = '\r';
    sendBuffer[payloadSize + 3] = checksum;    

    Serial1.write(sendBuffer, payloadSize + 4);
    Serial1.flush();
  }
}

void uartRead() {
  if (Serial1.available()) {
    int rpiPacket = Serial1.read();

    if (rpiPacket == SYN) {
      is_SYN_Received = true;
      is_ACK_Received = false;
      pedoValue = 0;
      Serial.println("Received SYN from RPi");
      Serial.flush();
    } else if (rpiPacket == ACK) {
      is_ACK_Received = true;
      Serial.println("Received ACK from RPi");
      Serial.flush();
    }
  }
}

void imuRead() {
  sei(); // enable all Arduino interrupts for I2C interface utilized by IMU sensors
  imuAccRead();
  imuMagRead();
  imuBarRead();
}

void imuAccRead() {
  accMag.read();

  float accX = (float) accMag.a.x;
  float accY = (float) accMag.a.y;
  float accZ = (float) accMag.a.z;

  float accMagnitude = calculateMagnitude(accX, accY, accZ);

  if (flag && (millis() - lastAccelerationTime >= 1000)) {
    flag = false;
  }

  if (accMagnitude >= ACCELERATION_THRESHOLD) {
    if (!flag) {
      pedoValue++;
      flag = true;
      lastAccelerationTime = millis();
    }
  } else if (accMagnitude <= DECELERATION_THRESHOLD) {
    flag = false;
  }
}

void imuMagRead() {
  accMag.read();
  imuValue[MAG_HEADING_INDEX] = accMag.heading();
  if (imuValue[MAG_HEADING_INDEX] > MAG_THRESHOLD) {
     imuValue[MAG_HEADING_INDEX] = imuValue[MAG_HEADING_INDEX] - MAG_NORMALIZER;
  }
}

void imuBarRead() {
  imuValue[BAR_VALUE_INDEX] = barEg;
}

void usRead() {
  for (int i = 0; i < sizeof(US_PIN_IN) / sizeof(US_PIN_IN[0]); i++) {
    unsigned long echoDuration = pulse(US_PIN_OUT[i], US_PIN_IN[i]);
    usValue[i] = calculateDistance(echoDuration);
  }

  Serial.println("US:");
  for (int i = 0; i < sizeof(US_PIN_IN) / sizeof(US_PIN_IN[0]); i++) {
    Serial.println(usValue[i]);
  }
  
  Serial.println();
  Serial.flush();
}

void irRead() {
  for (int i = 0; i < sizeof(IR_PIN) / sizeof(IR_PIN[0]); i++) {
    int divisor = analogRead(IR_PIN[i]);
    if (divisor <= 3) {
      irValue[i] = INFINITY;
    } else {
      irValue[i] = 6787.0 / (divisor - 3) - 4;
    }
  }

  Serial.println("IR:");
  for (int i = 0; i < sizeof(IR_PIN) / sizeof(IR_PIN[0]); i++) {
    Serial.println(irValue[i]);
  }

  Serial.println();
  Serial.flush();
}

void dcWrite() {
  sei(); // enable all Arudino interrupts for delay() utilized
  
  dcTurnOff(DC_LEFT);
  dcTurnOff(DC_RIGHT);
  
  if ((usValue[US_LEFT] < US_THRESHOLD_DISTANCE && usValue[US_LEFT] > US_MINIMUM_DISTANCE) || (irValue[IR_LEFT] < IR_THRESHOLD_DISTANCE && irValue[IR_LEFT] > IR_MINIMUM_DISTANCE)) {
    dcRotateLeft(DC_LEFT);
  }
  if ((usValue[US_RIGHT] < US_THRESHOLD_DISTANCE && usValue[US_RIGHT] > US_MINIMUM_DISTANCE) || (irValue[IR_RIGHT] < IR_THRESHOLD_DISTANCE && irValue[IR_RIGHT] > IR_MINIMUM_DISTANCE)) {
    dcRotateRight(DC_RIGHT);
  }
  if ((usValue[US_FRONT_TOP] < US_THRESHOLD_DISTANCE && usValue[US_FRONT_TOP] > US_MINIMUM_DISTANCE) || 
      (usValue[US_FRONT_BOTTOM] < US_THRESHOLD_DISTANCE && usValue[US_FRONT_BOTTOM] > US_MINIMUM_DISTANCE) || 
      (irValue[IR_FRONT_LEFT] < IR_THRESHOLD_DISTANCE && irValue[IR_FRONT_LEFT] > IR_MINIMUM_DISTANCE) || 
      (irValue[IR_FRONT_RIGHT] < IR_THRESHOLD_DISTANCE && irValue[IR_FRONT_RIGHT] > IR_MINIMUM_DISTANCE)) {
    dcRotateBoth();
  }
}

void dcTurnOff(int id) {
  analogWrite(DC_PIN_LEFT[id], 0);
  analogWrite(DC_PIN_RIGHT[id], 0);
}

void dcRotateLeft(int id) {
  analogWrite(DC_PIN_LEFT[id], 255);
  analogWrite(DC_PIN_RIGHT[id], 0);
  delay(250);
  dcTurnOff(id);
}

void dcRotateRight(int id) {
  analogWrite(DC_PIN_LEFT[id], 0);
  analogWrite(DC_PIN_RIGHT[id], 255);
  delay(250);
  dcTurnOff(id);
}

void dcRotateBoth() {
  analogWrite(DC_PIN_LEFT[DC_LEFT], 255);
  analogWrite(DC_PIN_RIGHT[DC_LEFT], 0);
  analogWrite(DC_PIN_LEFT[DC_RIGHT], 0);
  analogWrite(DC_PIN_RIGHT[DC_RIGHT], 255);
  delay(250);
  dcTurnOff(DC_LEFT);
  dcTurnOff(DC_RIGHT);
}

// Helper functions

float calculateMagnitude(float x, float y, float z) {
  return sqrt(pow(x,2) + pow(y,2) + pow(z,2));
}

unsigned long pulse(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(triggerPin, LOW);

  return pulseIn(echoPin, HIGH);
}

float calculateDistance(unsigned long duration) {
  return duration / 58.2;
}
