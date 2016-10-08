#include <smartTimer.h>
#include <math.h>
#include <Wire.h>
#include <LSM303.h>

LSM303 accMag;

// Declaration of Sensors and Actuators pins

const int UART_PIN_TX = 18;
const int UART_PIN_RX = 19;

const int US_PIN_IN[] = { 23, 25, 27, 29 }; // Connected to ECHO pins of ultrasound sensors
const int US_PIN_OUT[] = { 24, 26, 28, 39 }; // Connected to TRIGGER pins of ultrasound sensors

const int IR_PIN[] = { 2, 3, 4, 5 };

const int DC_1_PIN = 6;
const int DC_2_PIN = 7;

// Declaration of poll periods (in ms)

const int UART_WRITE_PERIOD = 300;
const int UART_READ_PERIOD = 500;
const int US_PERIOD = 200;
const int IR_PERIOD = 200;
const int DC_PERIOD = 200;
const int IMU_PERIOD = 300;


// Declaration of packet codes 

const char SYN = 0;
const char SYNACK = 1;
const char ACK = 2;
const char NACK = 3;
const char DATA = 4;

// Declaration of indices for IMU data

const char ACC_X_INDEX = 0;
const char ACC_Y_INDEX = 1;
const char ACC_Z_INDEX = 2;
const char GYR_X_INDEX = 3;
const char GYR_Y_INDEX = 4;
const char GYR_Z_INDEX = 5;
const char MAG_HEADING_INDEX = 6;
const char BAR_INDEX = 7;

float usValue[] = { NAN, NAN, NAN, NAN };
float irValue[] = { NAN, NAN, NAN, NAN };
float dcValue = NAN;
float imuValue[] = { NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN };

float accXEg = 25.9;
float accYEg = 30.7;
float accZEg = 39.2;
float gyrXEg = 12.9;
float gyrYEg = 31.6;
float gyrZEg = 29.1;
float magHeadingEg = 49.3;
float barEg = 57.3;

char sendBuffer[75]; // For transmitting data onto Arduino's USART

// For Handshake protocol between Arduino and RPi

boolean is_SYN_Received = false;
boolean is_ACK_Received = false;
unsigned long synAckSendTime = millis();

CSmartTimer *timer1;
CSmartTimer *timer2;
CSmartTimer *timer3;

void initializePins();
void initializeTimers();
void uartDataWrite();
void uartRead();
void usRead();
void irRead();
void dcWrite();
void imuRead();
void imuAccRead();
void imuGyrRead();
void imuMagRead();
void imuBarRead();

unsigned long pulse(int triggerPin, int echoPin);
float calculateDistance(unsigned long duration);

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  Serial1.flush();

  Wire.begin();
  while (!accMag.init()) {
    Serial.println("Failed to autodetect compass!");
  }
  accMag.enableDefault();
  Serial.println("Successful");

  accMag.m_min = (LSM303::vector<int16_t>){-1832, -2342, -1505};
  accMag.m_max = (LSM303::vector<int16_t>){+2278, +2042, +2726};

//  Serial.println(millis());
//  accMag.read();
//  Serial.println(millis());
  
  initializePins();
  initializeTimers();
}

void loop() {
  // Do nothing
}

void initializePins() {
//  for (int i = 0; i < sizeof(US_PIN_IN) / sizeof(int); i++) {
//    pinMode(US_PIN_IN[i], INPUT);
//    pinMode(US_PIN_OUT[i], OUTPUT);
//  }

//  for (int i = 0; i < sizeof(IR_PIN) / sizeof(int); i++) {
//    pinMode(IR_PIN[i], INPUT);
//  }
}

void initializeTimers() {
  timer1 = new CSmartTimer(STIMER1);
  timer1 -> attachCallback(uartDataWrite, UART_WRITE_PERIOD);
  timer1 -> attachCallback(uartRead, UART_READ_PERIOD);
//  timer1 -> attachCallback(imuRead, IMU_PERIOD);
  
//  timer2 = new CSmartTimer(STIMER2);
//  timer2 -> attachCallback(usRead, US_PERIOD);
//  timer2 -> attachCallback(irRead, IR_PERIOD);
//  timer2 -> attachCallback(dcWrite, DC_PERIOD);
  
  timer1 -> startTimer();
//  timer2 -> startTimer();
}

void uartDataWrite() {
  char payloadSize = 0;
  char packetCode = DATA;
  char checksum = 0;

  if (is_SYN_Received == true && is_ACK_Received == false) {
    if (millis() - synAckSendTime >= 1000) { 
      sendBuffer[0] = SYNACK;
      Serial1.write(sendBuffer, 1);
      synAckSendTime = millis();
    }
  } else if (is_ACK_Received == true) {
//    Serial.print("Entered data write");

    accMag.read();
    Serial.println(accMag.a.x);
    imuValue[ACC_X_INDEX] = (float) accMag.a.x;
    imuValue[ACC_Y_INDEX] = (float) accMag.a.y;
    imuValue[ACC_Z_INDEX] = (float) accMag.a.z;

    imuValue[MAG_HEADING_INDEX] = accMag.heading();
    
    memcpy(sendBuffer + 2, imuValue, sizeof(imuValue));
    payloadSize += sizeof(imuValue);
    
    for (int i = 0; i < sizeof(imuValue) / sizeof(float); i++) {
      imuValue[i] = NAN; 
    }
     
    for (int j = 2; j <= payloadSize + 1; j++) {
      checksum ^= sendBuffer[j];
    }

    sendBuffer[0] = packetCode;
    sendBuffer[1] = payloadSize;
    sendBuffer[payloadSize + 2] = '\r';
    sendBuffer[payloadSize + 3] = checksum;    

    Serial1.write(sendBuffer, payloadSize + 4);
//    Serial.print("Sent data");
  }
}

void uartRead() {
  if (Serial1.available()) {
    int rpiPacket = Serial1.read();

    if (rpiPacket == SYN) {
      is_SYN_Received = true;
    } else if (rpiPacket == ACK) {
      is_ACK_Received = true;
    }
  }
}

void usRead() {
  for (int i = 0; i < sizeof(usValue) / sizeof(float); i++) {
    unsigned long echoDuration = pulse(US_PIN_OUT[i], US_PIN_IN[i]);
    usValue[i] = calculateDistance(echoDuration);
  }

  Serial.println("US:");
  for (int i = 0; i < sizeof(usValue) / sizeof(float); i++) {
    Serial.println(usValue[i]);
  }

  Serial.println();
}

void irRead() {
  for (int i = 0; i < sizeof(irValue) / sizeof(float); i++) {
    int divisor = analogRead(IR_PIN[i]);
    if (divisor <= 3) {
      irValue[i] = NAN;
    } else {
      irValue[i] = 6786.0 / (divisor - 3.0) - 4.0;
    }
  }

  Serial.println("IR:");
  for (int i = 0; i < sizeof(irValue) / sizeof(float); i++) {
    Serial.println(irValue[i]);
  }

  Serial.println();
}

void dcWrite() {
  
}

void imuRead() {
  imuAccRead();
  imuGyrRead();
  imuMagRead();
  imuBarRead();
}

void imuAccRead() {
//  accMag.read();
  
//  Serial.print("Acc X: ");
//  Serial.println(accMag.a.x);
//  Serial.print("Acc Y: ");
//  Serial.println(accMag.a.y);
//  Serial.print("Acc Z: ");
//  Serial.println(accMag.a.z);
//  imuValue[ACC_X_INDEX] = (float) accMag.a.x;
//  imuValue[ACC_Y_INDEX] = (float) accMag.a.y;
//  imuValue[ACC_Z_INDEX] = (float) accMag.a.z;

  imuValue[ACC_X_INDEX] = accXEg;
  imuValue[ACC_Y_INDEX] = accYEg;
  imuValue[ACC_Z_INDEX] = accZEg;
  accXEg += 5;
  accYEg += 5;
  accZEg += 5;
}

void imuGyrRead() {
  imuValue[GYR_X_INDEX] = gyrXEg;
  imuValue[GYR_Y_INDEX] = gyrYEg;
  imuValue[GYR_Z_INDEX] = gyrZEg;
  gyrXEg += 5;
  gyrYEg += 5;
  gyrZEg += 5;
}

void imuMagRead() {
//  accMag.read();
//  imuValue[MAG_HEADING_INDEX] = accMag.heading();

  imuValue[MAG_HEADING_INDEX] = magHeadingEg;
  magHeadingEg += 5;
}

void imuBarRead() {
  imuValue[BAR_INDEX] = barEg;
  barEg += 5;
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
