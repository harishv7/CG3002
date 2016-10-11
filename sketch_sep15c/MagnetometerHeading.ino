#include <Wire.h>
#include <LSM303.h>

LSM303 compass;

#define echoPin1 5 // Echo Pin
#define trigPin1 4 // Trigger Pin
#define echoPin2 7 // Echo Pin
#define trigPin2 6 // Trigger Pin
#define echoPin3 9 // Echo Pin
#define trigPin3 8 // Trigger Pin

float ACCELERATION_THRESHOLD = 20000;
float DECELERATION_THRESHOLD = 14000;
long lastAccelerationTime;

int maximumRange = 200; // Maximum range needed
int motorRange = 80; // Range to trigger the DC motor
int minimumRange = 10; // Minimum range needed
long duration1, distance1; // Duration used to calculate distance
long duration2, distance2;
long duration3, distance3;

int motorPin1 =  2; 
int motorPin2 =  3;

float newValueX, newValueY, newValueZ;
float newMag;
bool flag = false;

int stepCount = 0;

float IR1, IR2, IR3, IR4;
int maximumRangeIR = 80;
int minimumRangeIR = 10;

float readIR(byte pin) {
  int temp;
  temp = analogRead(pin);

  if (temp < 3) return -1;
  return (6787.0 / ((float)temp - 3.0)) - 4.0;
}

float calculateMagnitude(float x, float y, float z) {
  return sqrt(pow(x,2) + pow(y,2) + pow(z,2));
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  if (!compass.init()) {
    Serial.println("Failed to autodetect compass!");
    while (1);
  }
  compass.enableDefault();

  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  
  pinMode(motorPin1, OUTPUT); 
  pinMode(motorPin2, OUTPUT); 
  /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-1832, -2342, -1505};
  compass.m_max = (LSM303::vector<int16_t>){+2278, +2042, +2726};
}

long pulse(int trigger, int echo) {
  /* The following trigPin/echoPin cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 
 digitalWrite(trigger, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigger, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigger, LOW);
 //return pulseIn(echo, HIGH);
 return 0;
}

long calculateDistance(long duration) {
  return duration/58.2;
}

void printDistanceUltrasound(long distance, int num) {
  if (distance >= maximumRange || distance <= minimumRange){
 /* Send a negative number to computer and Turn LED ON 
 to indicate "out of range" */
 Serial.println("No object detected");
 }
 else if (distance >= minimumRange && distance <= motorRange) {
     /* Send the distance to the computer using Serial protocol, and
     turn LED OFF to indicate successful reading. */
    // Serial.println("Ultrasound " + (num == 1) ? "middle" : (num == 2) ? "left" : "right" + ": " + distance + " cm.");
     Serial.print("Ultrasound ");
     Serial.print((num == 1) ? "middle" : (num == 2) ? "left" : "right");
     Serial.print(": ");
     Serial.print(distance);
     Serial.println(" cm.");
//     if (num == 1) {
//        rotateLeftFull(500);
//        rotateRightFull(500);
//      }
 } else {
  Serial.print("Ultrasound ");
  Serial.print((num == 1) ? "middle" : (num == 2) ? "left" : "right");
  Serial.print(": ");
  Serial.print(distance);
  Serial.println(" cm.");
 }
}

void printDistanceIR(float distance, int num) {
  if (distance >= minimumRangeIR && distance <= maximumRangeIR) {
    Serial.print("IR ");
    Serial.print(num);
    Serial.print(": ");
    Serial.print(distance);
    Serial.println(" cm.");
  } else {
    Serial.println("Out of range");
  }
}

void rotateLeftFull(int length){
  digitalWrite(motorPin1, HIGH); //rotates motor
//  digitalWrite(motorPin3, HIGH); 
  digitalWrite(motorPin2, LOW); 
//  digitalWrite(motorPin4, LOW); // set the Pin motorPin2 LOW
  delay(length); //waits
  digitalWrite(motorPin1, LOW);    // set the Pin motorPin1 LOW
//  digitalWrite(motorPin3, LOW);
}

void rotateRightFull(int length){
  digitalWrite(motorPin2, HIGH); //rotates motor
//  digitalWrite(motorPin4, HIGH);
  digitalWrite(motorPin1, LOW);    // set the Pin motorPin1 LOW
//  digitalWrite(motorPin3, LOW); 
  delay(length); //waits
  digitalWrite(motorPin2, LOW);    // set the Pin motorPin2 LOW
//  digitalWrite(motorPin4, LOW);
}

void loop() {
  compass.read();

  newValueX = (float) compass.a.x;
  newValueY = (float) compass.a.y;
  newValueZ = (float) compass.a.z;

  newMag = calculateMagnitude(newValueX, newValueY, newValueZ);

  if (flag && (millis() - lastAccelerationTime >= 1000)) {
    flag = false;
  }

  if (newMag >= ACCELERATION_THRESHOLD) {
    if (!flag) {
      stepCount++;
      flag = true;
      lastAccelerationTime = millis();
    }
  } else if (newMag <= DECELERATION_THRESHOLD) {
    flag = false;
  }
    
  float heading = compass.heading();

  duration1 = pulse(trigPin1, echoPin1);
  duration2 = pulse(trigPin2, echoPin2);
  duration3 = pulse(trigPin3, echoPin3);
  distance1 = calculateDistance(duration1);
  distance2 = calculateDistance(duration2);
  distance3 = calculateDistance(duration3);
  
//  printDistanceUltrasound(distance1, 1);
//  printDistanceUltrasound(distance2, 2);
//  printDistanceUltrasound(distance3, 3);

  IR1 = readIR(0);
  IR2 = readIR(1);
  IR3 = readIR(2);
  IR4 = readIR(3);

//  printDistanceIR(IR1, 1);
//  printDistanceIR(IR2, 2);
//  printDistanceIR(IR3, 3);
//  printDistanceIR(IR4, 4);
//  
  Serial.print("Current Direction: ");
  if (heading > 180 && heading <= 360) {
    Serial.println(heading - 360);
  } else {
    Serial.println(heading);
  }
//  if ((heading >= 337.5 && heading <= 360) || (heading >= 0 && heading <= 22.5)) {
//    Serial.println("N");
//  } else if (heading > 22.5 && heading <= 67.5) {
//    Serial.println("NE");
//  } else if (heading > 67.5 && heading <= 112.5) {
//    Serial.println("E");
//  } else if (heading > 112.5 && heading <= 157.5) {
//    Serial.println("SE");
//  } else if (heading > 157.5 && heading <= 202.5) {
//    Serial.println("S");
//  } else if (heading > 202.5 && heading <= 247.5) {
//    Serial.println("SW");
//  } else if (heading > 247.5 && heading <= 292.5) {
//    Serial.println("W");
//  } else {
//    Serial.println("NW");
//  } 
  //Serial.println(newMag);
  Serial.println(stepCount);

  delay(50);
}
