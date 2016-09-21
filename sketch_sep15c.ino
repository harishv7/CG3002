#include <smartTimer.h>
#include <math.h>

// Sensors and actuators pins declaration

const int UART_PIN_TX = 18;
const int UART_PIN_RX = 19;

const int US_PIN_IN[] = { 23, 25, 27, 29 }; // Connected to ECHO pins of ultrasound sensors
const int US_PIN_OUT[] = { 24, 26, 28, 39 }; // Connected to TRIGGER pins of ultrasound sensors

const int IR_PIN[] = { 2, 3, 4, 5 };

const int DC_1_PIN = 6;
const int DC_2_PIN = 7;

const int IMU_PIN_SDA = 20;
const int IMU_PIN_SCL = 21;

// Poll periods declaration (in ms)

const int UART_PERIOD = 500;
const int US_PERIOD = 500;
const int IR_PERIOD = 2000;
const int DC_PERIOD = 100;
const int IMU_ACC_PERIOD = 10;
const int IMU_MAG_PERIOD = 320;
const int IMU_GYR_PERIOD = 10;
const int IMU_BAR_PERIOD = 1000;

CSmartTimer *timer1;
CSmartTimer *timer2;
CSmartTimer *timer3;

float usValue[] = { NAN, NAN, NAN, NAN };
float irValue[] = { NAN, NAN, NAN, NAN };
float dcValue = NAN;
float imuAccValue_X = NAN;
float imuAccValue_Y = NAN;
float imuAccValue_Z = NAN;
float imuMagValue_X = NAN;
float imuMagValue_Y = NAN;
float imuMagValue_Z = NAN;
float imuGyrValue_X = NAN;
float imuGyrValue_Y = NAN;
float imuGyrValue_Z = NAN;
float imuBarValue = NAN;

char sendBuffer[75]; // For transmitting data onto Arduino's USART

void initializePins();
void initializeTimers();
void uartWrite();
void uartRead();
void usRead();
void irRead();
void dcWrite();
void imuAccRead();
void imuMagRead();
void imuGyrRead();
void imuBarRead();

unsigned long pulse(int triggerPin, int echoPin);
float calculateDistance(unsigned long duration);

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);

  initializePins();
  initializeTimers();
}

void loop() {
  // Do nothing
}

void initializePins() {
  for (int i = 0; i < sizeof(US_PIN_IN) / sizeof(int); i++) {
    pinMode(US_PIN_IN[i], INPUT);
    pinMode(US_PIN_OUT[i], OUTPUT);
  }

//  for (int i = 0; i < sizeof(IR_PIN) / sizeof(int); i++) {
//    pinMode(IR_PIN[i], INPUT);
//  }
}

void initializeTimers() {
  timer1 = new CSmartTimer(STIMER1);
  timer1 -> attachCallback(uartWrite, UART_PERIOD);
  timer1 -> attachCallback(uartRead, UART_PERIOD);

  timer2 = new CSmartTimer(STIMER2);
  timer2 -> attachCallback(usRead, US_PERIOD);
  timer2 -> attachCallback(irRead, IR_PERIOD);
  timer2 -> attachCallback(dcWrite, DC_PERIOD);

  timer3 = new CSmartTimer(STIMER3);
  timer3 -> attachCallback(imuAccRead, IMU_ACC_PERIOD);
  timer3 -> attachCallback(imuMagRead, IMU_MAG_PERIOD);
  timer3 -> attachCallback(imuGyrRead, IMU_GYR_PERIOD);
  timer3 -> attachCallback(imuBarRead, IMU_BAR_PERIOD);
  
  timer1 -> startTimer();
  timer2 -> startTimer();
  timer3 -> startTimer();
}

void uartWrite() {
  char checksum = 0;
  char size = 0;
  
  if (usValue[0] != NAN) {
    memcpy(sendBuffer + 1, usValue, sizeof(usValue));
    size += sizeof(usValue);
    
    for (int i = 0; i < sizeof(usValue) / sizeof(float); i++) {
      usValue[i] = NAN; 
    }
     
    for (int j = 1; j <= size; j++) {
      checksum ^= sendBuffer[j];
    }

    sendBuffer[0] = size;
    sendBuffer[size+1] = checksum;
    sendBuffer[size+2] = '\r';    

    Serial1.write(sendBuffer, size + 3);
  }

//  if (irValue[0] != NAN) {
//    memcpy(buf+1, usValue, sizeof(irValue) / sizeof(float));
//  }
}

void uartRead() {
//  if (Serial1.available()) {
//    int receivedByte = Serial1.read();
//  }
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
//    for (int i = 0; i < sizeof(irValue) / sizeof(float); i++) {
//      int divisor = analogRead(IR_PIN[i]);
//      if (divisor <= 3) {
//        irValue[i] = NAN;
//      } else {
//        irValue[i] = 6786.0 / (divisor - 3.0) - 4.0;
//      }
//    }
//
//    Serial.println("IR:");
//    for (int i = 0; i < sizeof(irValue) / sizeof(float); i++) {
//      Serial.println(irValue[i]);
//    }
//
//    Serial.println();
}

void dcWrite() {
  
}

void imuAccRead() {
  
}

void imuMagRead() {
  
}

void imuGyrRead() {
  
}

void imuBarRead() {
  
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
