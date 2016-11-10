#include <smartTimer.h>
#include <math.h>
#include <Wire.h>
#include <LSM303.h>

LSM303 imu;

// Declaration of pins for sensors and actuators

const char UART_PIN_TX = 18;
const char UART_PIN_RX = 19;

const char US_LEFT = 4, US_FRONT = 2, US_RIGHT = 0, US_LEG_LEFT = 5, US_LEG_RIGHT = 1, US_ARM = 3;

const char US_PIN_IN[] = { 42, 44, 46, 48, 50, 52 }; // Connected to ECHO pins of ultrasound sensors
const char US_PIN_OUT[] = { 43, 45, 47, 49, 51, 53 }; // Connected to TRIGGER pins of ultrasound sensors

const int US_MIN_THRESHOLD[] = { 10, 10, 10, 10, 10, 10 };
const int US_MAX_THRESHOLD[] = { 80, 120, 120, 100, 80, 120 };

const char VIB_LEFT = 3, VIB_FRONT = 2, VIB_RIGHT = 1, VIB_ARM = 0;

const char VIB_PIN[] = { 7, 9, 11, 13 };

// Declaration of poll periods (in ms)

const int UART_WRITE_PERIOD = 500;
const int UART_READ_PERIOD = 500;
const int IMU_READ_PERIOD = 50;
const int SENSOR_READ_PERIOD = 1000;

// Declaration of packet codes 

const char SYN = 0;
const char SYNACK = 1;
const char ACK = 2;
const char DATA = 3;

// Declaration of indices

const char SYNACK_INDEX = 0;

const char PACKET_CODE_INDEX = 0;

const char MAG_HEADING_INDEX = 0;
const char BAR_VALUE_INDEX = 1;

// Declaration of thresholds

const float ACCELERATION_THRESHOLD = 21000;
const float DECELERATION_THRESHOLD = 13000;

const float MAG_THRESHOLD = 180;
const float MAG_NORMALIZER = 360;

long lastAccelerationTime;
long pedoValue = 0;
bool flag = false;

float usValue[] = { NAN, NAN, NAN, NAN, NAN, NAN };
float imuValue[] = { NAN, NAN };

float barEg = 45.5;

char sendBuffer[18]; // For transmitting data onto Arduino's USART

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
void vibWrite();
void vibTurnOff();
void vibTurnOn();

float calculateMagnitude(float x, float y, float z);
unsigned long pulse(int triggerPin, int echoPin);
float calculateDistance(unsigned long duration);

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial1.flush();

  Wire.begin();
  while (!imu.init()) {
    Serial.println("Failed to autodetect accmag!");
  }
  imu.enableDefault();
  Serial.println("Successful");
  // Serial.flush();

  imu.m_min = (LSM303::vector<int16_t>){-1832, -2342, -1505};
  imu.m_max = (LSM303::vector<int16_t>){+2278, +2042, +2726};
  
  initializePins();
  initializeTimers();
}

void loop() {
  // Do nothing
}

void initializePins() {
  int i;
  for (i = 0; i < sizeof(US_PIN_IN) / sizeof(US_PIN_IN[0]); i++) {
    pinMode(US_PIN_IN[i], INPUT);
    pinMode(US_PIN_OUT[i], OUTPUT);
  }
  
  for (i = 0; i < sizeof(VIB_PIN) / sizeof(VIB_PIN[0]); i++) {
    pinMode(VIB_PIN[i], OUTPUT);
  }
}

void initializeTimers() {
  timer1 = new CSmartTimer(STIMER1);
  timer2 = new CSmartTimer(STIMER2);
  
  timer1 -> attachCallback(uartWrite, UART_WRITE_PERIOD);
  timer1 -> attachCallback(uartRead, UART_READ_PERIOD);
  timer1 -> attachCallback(imuRead, IMU_READ_PERIOD);

  timer2 -> attachCallback(usRead, SENSOR_READ_PERIOD);
  
  timer1 -> startTimer();
  timer2 -> startTimer();
}

void uartWrite() {
  char payloadSize = 0;
  char packetCode = DATA;
  long checksum = 0;

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
    char pedoOffset = 1;
    memcpy(sendBuffer + pedoOffset, &pedoValue, sizeof(pedoValue));
    payloadSize += sizeof(pedoValue);

    char imuOffset = pedoOffset + sizeof(pedoValue);
    memcpy(sendBuffer + imuOffset, imuValue, sizeof(imuValue));
    payloadSize += sizeof(imuValue);
     
    checksum = pedoValue ^ (long) imuValue[MAG_HEADING_INDEX] ^ (long) imuValue[BAR_VALUE_INDEX]; // Checksum obtained by XORing 4 byte quantities

    sendBuffer[PACKET_CODE_INDEX] = packetCode;
    
    char checksumOffset = imuOffset + sizeof(imuValue);
    memcpy(sendBuffer + checksumOffset, &checksum, sizeof(checksum));
    payloadSize += sizeof(checksum);   

    Serial1.write(sendBuffer, payloadSize + 1);
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
  imu.read();
  imuAccRead();
  imuMagRead();
  imuBarRead();
}

void imuAccRead() {
  float accX = (float) imu.a.x;
  float accY = (float) imu.a.y;
  float accZ = (float) imu.a.z;

  float accMagnitude = calculateMagnitude(accX, accY, accZ);

  if (flag && (millis() - lastAccelerationTime >= 500)) {
    flag = false;
  }

  if (accMagnitude >= ACCELERATION_THRESHOLD) {
    if (!flag) {
      pedoValue++;
      flag = true;
      lastAccelerationTime = millis();
    }
  } else if (flag && accMagnitude <= DECELERATION_THRESHOLD) {
    flag = false;
  }
}

void imuMagRead() {
  imuValue[MAG_HEADING_INDEX] = imu.heading();
  if (imuValue[MAG_HEADING_INDEX] > MAG_THRESHOLD) {
     imuValue[MAG_HEADING_INDEX] = imuValue[MAG_HEADING_INDEX] - MAG_NORMALIZER;
  }
}

void imuBarRead() {
  imuValue[BAR_VALUE_INDEX] = barEg;
}

void usRead() {
  int i = 0;
  for (i = 0; i < sizeof(US_PIN_IN) / sizeof(US_PIN_IN[0]); i++) {
    unsigned long echoDuration = pulse(US_PIN_OUT[i], US_PIN_IN[i]);
    usValue[i] = calculateDistance(echoDuration);
  }
  
  vibWrite();
}

void vibWrite() {
  vibTurnOff(VIB_LEFT);
  vibTurnOff(VIB_FRONT);
  vibTurnOff(VIB_RIGHT);
  vibTurnOff(VIB_ARM);
  
  if (usValue[US_LEFT] < US_MAX_THRESHOLD[US_LEFT] && usValue[US_LEFT] > US_MIN_THRESHOLD[US_LEFT]) {
    vibTurnOn(VIB_LEFT);
  }

  if (usValue[US_LEG_LEFT] < US_MAX_THRESHOLD[US_LEG_LEFT] && usValue[US_LEG_LEFT] > US_MIN_THRESHOLD[US_LEG_LEFT]) {
    vibTurnOn(VIB_LEFT);
  }
  
  if (usValue[US_RIGHT] < US_MAX_THRESHOLD[US_RIGHT] && usValue[US_RIGHT] > US_MIN_THRESHOLD[US_RIGHT]) {
    vibTurnOn(VIB_RIGHT);
  }

  if (usValue[US_LEG_RIGHT] < US_MAX_THRESHOLD[US_LEG_RIGHT] && usValue[US_LEG_RIGHT] > US_MIN_THRESHOLD[US_LEG_RIGHT]) {
    vibTurnOn(VIB_RIGHT);
  }
  
  if (usValue[US_FRONT] < US_MAX_THRESHOLD[US_FRONT] && usValue[US_FRONT] > US_MIN_THRESHOLD[US_FRONT]) {
    vibTurnOn(VIB_FRONT);
  }

  if (usValue[US_ARM] < US_MAX_THRESHOLD[US_ARM] && usValue[US_ARM] > US_MIN_THRESHOLD[US_ARM]) {
    vibTurnOn(VIB_ARM);
  }
}

void vibTurnOff(char id) {
  digitalWrite(VIB_PIN[id], LOW);
}

void vibTurnOn(char id) {
  digitalWrite(VIB_PIN[id], HIGH);
}

// Helper functions

float calculateMagnitude(float x, float y, float z) {
  return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

unsigned long pulse(int triggerPin, int echoPin) {
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);

  digitalWrite(triggerPin, LOW);

  return pulseIn(echoPin, HIGH, 40000);
}

float calculateDistance(unsigned long duration) {
  return duration / 58.2;
}
