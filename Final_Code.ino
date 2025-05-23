// TODO AUTHORS

// Libraries
#include <Servo.h>

// Global Values
int upPosition = 0;
int downPosition = 145;
int binRotateTime = 120;
int sweepMoveTime = 820;

volatile long countA = 0;
volatile long countB = 0;
volatile long countC = 0;
volatile long countD = 0;

// Start Button Pin
int startButton = 47;

// Wheel Pins
int pwmPinBL = 53;
int directionPinBL = 52;
int fgPinBL = 19;

int pwmPinBR = 48;
int directionPinBR = 49;
int fgPinBR = 20;

int pwmPinFL = 24;
int directionPinFL = 21;
int fgPinFL = 3;

int pwmPinFR = 25;
int directionPinFR = 23;
int fgPinFR = 2;

// Arm Servo Pin
Servo armServo;
int armServoPin = 46;

// Sweep Pins
int leftA = 35;
int leftB = 36;
int rightA = 38;
int rightB = 37;

// Bin Pins
int leftBin = 18; 
int rightBin = 45; 

//Sensor Pin
int sensorTrig = 17;
int sensorEcho = 16;

void encoderFL_ISR() { countA++; }
void encoderFR_ISR() { countB++; }
void encoderBL_ISR() { countC++; }
void encoderBR_ISR() { countD++; }

void setup() {
  // Start Serial
  Serial.begin(9600);

  // Setup Start Button
  pinMode(startButton, INPUT);

  // Setup Wheels
  pinMode(pwmPinBL, OUTPUT);
  pinMode(directionPinBL, OUTPUT);

  pinMode(pwmPinBR, OUTPUT);
  pinMode(directionPinBR, OUTPUT);

  pinMode(pwmPinFL, OUTPUT);
  pinMode(directionPinFL, OUTPUT);

  pinMode(pwmPinFR, OUTPUT);
  pinMode(directionPinFR, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(fgPinFL), encoderFL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(fgPinFR), encoderFR_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(fgPinBL), encoderBL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(fgPinBR), encoderBR_ISR, RISING);

  // Setup Arm Servo
  armServo.attach(armServoPin);
  armServo.write(0);

  // Setup Sweep Pins
  pinMode(leftA, OUTPUT);
  pinMode(leftB, OUTPUT);
  pinMode(rightA, OUTPUT);
  pinMode(rightB, OUTPUT);

  // Setup Bin Pins
  pinMode(leftBin, OUTPUT);
  pinMode(rightBin, OUTPUT);

  // Stop all wheel motors
  analogWrite(pwmPinBR, 255);
  analogWrite(pwmPinBL, 255);
  analogWrite(pwmPinFR, 255);
  analogWrite(pwmPinFL, 255);

  // Set Up Sensor
  pinMode(sensorEcho, INPUT);
  pinMode(sensorTrig, OUTPUT);

  delay(1000);
}

void loop() {

  while(digitalRead(startButton) == LOW) {
    delay(5);
  }

  delay(1000);

  right(220);
  sweepOpen();
  armDown();
  left(40);
  delay(500);
  sweepClose();
  armUp();

  delay(500);
  right(180);
  sweepOpen();
  armDown();
  left(40);
  delay(500);
  sweepClose();
  armUp();

  delay(500);
  right(170);
  sweepOpen();
  armDown();
  left(40);
  delay(500);
  sweepClose();
  armUp();

  delay(1000);

  //left(150);

  //delay(1000);

  armServo.detach();

  delay(1000);
  delay(1000);
  forward(100);
  delay(1000);
  forward(50);
  delay(1000);
  forward(10);
  delay(1000);
  backward(100);
  delay(1000);

  forward(4500); 
  backward(100);
  delay(50);
  right(50);
  backward(0);
  delay(50);
  backward(150);
  backward(100);
  delay(500);
  delay(3000);

  backward(100);

  delay(500);

  backward(100);

  right(200);
  delay(1000);
  forwardEnd();
  delay(1000);
  backward(30);
  delay(1000);
  left(400);
  delay(1000);

  rotateBin(3000); // Deposit
  delay(1000);

  right(20);
  delay(200);
  backward(5);

  while(true) {
    rotateBinWhileButtonPressed();
  }

  }

void forwardEncoded(long targetTicks) {
  countA = countB = countC = countD = 0;

  digitalWrite(directionPinBL, LOW);
  digitalWrite(directionPinBR, HIGH);
  digitalWrite(directionPinFL, LOW);
  digitalWrite(directionPinFR, HIGH);

  const int pwmCycle = 1000; // microseconds per PWM cycle
  unsigned long lastUpdate = millis();

  // Start duty cycles at mid-power
  int dutyA = 950, dutyB = 950, dutyC = 950, dutyD = 950;

  while (countA < targetTicks || countB < targetTicks || countC < targetTicks || countD < targetTicks) {
    unsigned long now = millis();
    if (now - lastUpdate >= 10) { // adjust every 10ms
      lastUpdate = now;

      long maxCount = max(max(countA, countB), max(countC, countD));

      int errorA = maxCount - countA;
      int errorB = maxCount - countB;
      int errorC = maxCount - countC;
      int errorD = maxCount - countD;

      dutyA = constrain(800 + errorA * 3, 900, 1000);
      dutyB = constrain(800 + errorB * 3, 900, 1000);
      dutyC = constrain(800 + errorC * 3, 900, 1000);
      dutyD = constrain(800 + errorD * 3, 900, 1000);
    }

    // Individual PWM pulses
    digitalWrite(pwmPinFL, HIGH); delayMicroseconds(dutyA); digitalWrite(pwmPinFL, LOW);
    digitalWrite(pwmPinFR, HIGH); delayMicroseconds(dutyB); digitalWrite(pwmPinFR, LOW);
    digitalWrite(pwmPinBL, HIGH); delayMicroseconds(dutyC); digitalWrite(pwmPinBL, LOW);
    digitalWrite(pwmPinBR, HIGH); delayMicroseconds(dutyD); digitalWrite(pwmPinBR, LOW);

    delayMicroseconds(pwmCycle - max(max(dutyA, dutyB), max(dutyC, dutyD)));
  }

  // Stop all motors
  analogWrite(pwmPinBR, 255);
  analogWrite(pwmPinBL, 255);
  analogWrite(pwmPinFR, 255);
  analogWrite(pwmPinFL, 255);
}

void rightEncoded(long targetTicks) {
  countA = countB = countC = countD = 0;

  digitalWrite(directionPinBL, HIGH);
  digitalWrite(directionPinBR, HIGH);
  digitalWrite(directionPinFL, LOW);
  digitalWrite(directionPinFR, LOW);

  const int pwmCycle = 1000; // microseconds per PWM cycle
  unsigned long lastUpdate = millis();

  // Start duty cycles at mid-power
  int dutyA = 150, dutyB = 150, dutyC = 150, dutyD = 150;

  while (countA < targetTicks || countB < targetTicks || countC < targetTicks || countD < targetTicks) {
    unsigned long now = millis();
    if (now - lastUpdate >= 10) { // adjust every 10ms
      lastUpdate = now;

      long maxCount = max(max(countA, countB), max(countC, countD));

      int errorA = maxCount - countA;
      int errorB = maxCount - countB;
      int errorC = maxCount - countC;
      int errorD = maxCount - countD;

      dutyA = constrain(150 + errorA * 10, 100, 1000);
      dutyB = constrain(150 + errorB * 10, 100, 1000);
      dutyC = constrain(150 + errorC * 10, 100, 1000);
      dutyD = constrain(150 + errorD * 10, 100, 1000);
    }

    // Individual PWM pulses
    digitalWrite(pwmPinFL, HIGH); delayMicroseconds(dutyA); digitalWrite(pwmPinFL, LOW);
    digitalWrite(pwmPinFR, HIGH); delayMicroseconds(dutyB); digitalWrite(pwmPinFR, LOW);
    digitalWrite(pwmPinBL, HIGH); delayMicroseconds(dutyC); digitalWrite(pwmPinBL, LOW);
    digitalWrite(pwmPinBR, HIGH); delayMicroseconds(dutyD); digitalWrite(pwmPinBR, LOW);

    delayMicroseconds(pwmCycle - max(max(dutyA, dutyB), max(dutyC, dutyD)));

    delay(1);
  }

  // Stop all motors
  analogWrite(pwmPinBR, 255);
  analogWrite(pwmPinBL, 255);
  analogWrite(pwmPinFR, 255);
  analogWrite(pwmPinFL, 255);
}

void forward(unsigned long duration) {
  // Set all directions to forward
  digitalWrite(directionPinBL, LOW);
  digitalWrite(directionPinBR, HIGH);
  digitalWrite(directionPinFL, LOW);
  digitalWrite(directionPinFR, HIGH);

  // Drive all motors at full speed (255)
  analogWrite(pwmPinBR, 0);
  analogWrite(pwmPinBL, 0);
  analogWrite(pwmPinFR, 0);
  analogWrite(pwmPinFL, 0);

  // Measure how long the signal stays HIGH
  unsigned long pulseDurationBL = pulseIn(fgPinBL, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationBL > 0) {
    Serial.print("Pulse Duration: ");
    Serial.print(pulseDurationBL);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationFR = pulseIn(fgPinFR, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationFR > 0) {
    Serial.print("Pulse Duration: ");
    Serial.print(pulseDurationFR);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationFL = pulseIn(fgPinBL, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationBL > 0) {
    Serial.print("Pulse Duration: ");
    Serial.print(pulseDurationBL);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationBR = pulseIn(fgPinFR, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationFR > 0) {
    Serial.print("Pulse Duration: ");
    Serial.print(pulseDurationFR);
    Serial.println(" microseconds");
  }

  delay(duration);

  /// Stop all motors
  analogWrite(pwmPinBR, 255);
  analogWrite(pwmPinBL, 255);
  analogWrite(pwmPinFR, 255);
  analogWrite(pwmPinFL, 255);
}

void backward(unsigned long duration) {
  // Set all directions to forward
  digitalWrite(directionPinBL, HIGH);
  digitalWrite(directionPinBR, LOW);
  digitalWrite(directionPinFL, HIGH);
  digitalWrite(directionPinFR, LOW);

  // Drive all motors at full speed (0)
  analogWrite(pwmPinBR, 0);
  analogWrite(pwmPinBL, 0);
  analogWrite(pwmPinFR, 0);
  analogWrite(pwmPinFL, 0);

  // Measure how long the signal stays HIGH
  unsigned long pulseDurationBL = pulseIn(fgPinBL, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationBL > 0) {
    Serial.print("Pulse Duration: ");
    Serial.print(pulseDurationBL);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationFR = pulseIn(fgPinFR, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationFR > 0) {
    Serial.print("Pulse Duration: ");
    Serial.print(pulseDurationFR);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationFL = pulseIn(fgPinBL, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationBL > 0) {
    Serial.print("Pulse Duration: ");
    Serial.print(pulseDurationBL);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationBR = pulseIn(fgPinFR, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationFR > 0) {
    Serial.print("Pulse Duration: ");
    Serial.print(pulseDurationFR);
    Serial.println(" microseconds");
  }

  // Run for the specified duration
  delay(duration);

  // Stop all motors
  analogWrite(pwmPinBR, 255);
  analogWrite(pwmPinBL, 255);
  analogWrite(pwmPinFR, 255);
  analogWrite(pwmPinFL, 255);

  
}

void right(unsigned long duration) {
  // Set all directions to forward
  digitalWrite(directionPinBL, HIGH);
  digitalWrite(directionPinBR, HIGH);
  digitalWrite(directionPinFL, LOW);
  digitalWrite(directionPinFR, LOW);

  // Drive all motors at full speed (0)
  analogWrite(pwmPinBR, 0);
  analogWrite(pwmPinBL, 0);
  analogWrite(pwmPinFR, 0);
  analogWrite(pwmPinFL, 0);

  // Measure how long the signal stays HIGH
  unsigned long pulseDurationBL = pulseIn(fgPinBL, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationBL > 0) {
    Serial.print("Pulse Duration BL: ");
    Serial.print(pulseDurationBL);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationFR = pulseIn(fgPinFR, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationFR > 0) {
    Serial.print("Pulse Duration FR: ");
    Serial.print(pulseDurationFR);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationFL = pulseIn(fgPinBL, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationBL > 0) {
    Serial.print("Pulse Duration BL: ");
    Serial.print(pulseDurationBL);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationBR = pulseIn(fgPinFR, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationFR > 0) {
    Serial.print("Pulse Duration FR: ");
    Serial.print(pulseDurationFR);
    Serial.println(" microseconds");
  }

  // Run for the specified duration
  delay(duration);

  // Stop all motors
  analogWrite(pwmPinBL, 255);
  analogWrite(pwmPinBR, 255);
  analogWrite(pwmPinFL, 255);
  analogWrite(pwmPinFR, 255);
}

void left(unsigned long duration) {
  // Set all directions to forward
  digitalWrite(directionPinBL, LOW);
  digitalWrite(directionPinBR, LOW);
  digitalWrite(directionPinFL, HIGH);
  digitalWrite(directionPinFR, HIGH);

  // Drive all motors at full speed (0)
  analogWrite(pwmPinBR, 0);
  analogWrite(pwmPinBL, 0);
  analogWrite(pwmPinFR, 0);
  analogWrite(pwmPinFL, 0);

  // Measure how long the signal stays HIGH
  unsigned long pulseDurationBL = pulseIn(fgPinBL, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationBL > 0) {
    Serial.print("Pulse Duration BL: ");
    Serial.print(pulseDurationBL);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationFR = pulseIn(fgPinFR, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationFR > 0) {
    Serial.print("Pulse Duration FR: ");
    Serial.print(pulseDurationFR);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationFL = pulseIn(fgPinBL, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationBL > 0) {
    Serial.print("Pulse Duration BL: ");
    Serial.print(pulseDurationBL);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationBR = pulseIn(fgPinFR, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationFR > 0) {
    Serial.print("Pulse Duration BR: ");
    Serial.print(pulseDurationFR);
    Serial.println(" microseconds");
  }

  delay(duration);

  // Stop all motors
  analogWrite(pwmPinBR, 255);
  analogWrite(pwmPinBL, 255);
  analogWrite(pwmPinFR, 255);
  analogWrite(pwmPinFL, 255);
}

void rotateLeft(unsigned long duration) {
  // Set all directions to forward
  digitalWrite(directionPinBL, HIGH);
  digitalWrite(directionPinBR, HIGH);
  digitalWrite(directionPinFL, HIGH);
  digitalWrite(directionPinFR, HIGH);

  // Drive all motors at full speed (0)
  analogWrite(pwmPinBR, 0);
  analogWrite(pwmPinBL, 0);
  analogWrite(pwmPinFR, 0);
  analogWrite(pwmPinFL, 0);

  // Measure how long the signal stays HIGH
  unsigned long pulseDurationBL = pulseIn(fgPinBL, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationBL > 0) {
    Serial.print("Pulse Duration: ");
    Serial.print(pulseDurationBL);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationFR = pulseIn(fgPinFR, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationFR > 0) {
    Serial.print("Pulse Duration: ");
    Serial.print(pulseDurationFR);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationFL = pulseIn(fgPinBL, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationBL > 0) {
    Serial.print("Pulse Duration: ");
    Serial.print(pulseDurationBL);
    Serial.println(" microseconds");
  }
  // Measure how long the signal stays HIGH
  unsigned long pulseDurationBR = pulseIn(fgPinFR, HIGH);  // Measure HIGH pulse duration in microseconds
  // Print the measured pulse duration
  if (pulseDurationFR > 0) {
    Serial.print("Pulse Duration: ");
    Serial.print(pulseDurationFR);
    Serial.println(" microseconds");
  }

  // Run for the specified duration
  delay(duration);

  // Stop all motors
  analogWrite(pwmPinBR, 255);
  analogWrite(pwmPinBL, 255);
  analogWrite(pwmPinFR, 255);
  analogWrite(pwmPinFL, 255);

  
}

void armDown() {
  for (int pos = upPosition; pos <= downPosition; pos++) {
    armServo.write(pos);
    delay(10);
  }
}

void armUp() {
  for (int pos = downPosition; pos >= upPosition; pos--) {
    armServo.write(pos);
    delay(10);
  }
}

void armAngle(int angle) {
  for (int pos = upPosition; pos <= angle; pos++) {
    armServo.write(pos);
    delay(10);
  }
}

void sweepOpen() {
  digitalWrite(leftA, HIGH);
  digitalWrite(rightA, HIGH);

  delay(sweepMoveTime);

  digitalWrite(leftA, LOW);
  digitalWrite(rightA, LOW);
}

void sweepClose() {
  digitalWrite(leftB, HIGH);
  digitalWrite(rightB, HIGH);

  delay(sweepMoveTime + 25);

  digitalWrite(leftB, LOW);
  digitalWrite(rightB, LOW);
}

void rotateBin(int rotateTime) {
  unsigned long startTime = millis();

  while (millis() - startTime < rotateTime) {
    // Turn motors ON briefly
    digitalWrite(leftBin, HIGH);
    digitalWrite(rightBin, HIGH);
    delay(20);  // ON time

    // Then turn motors OFF for longer
    digitalWrite(leftBin, LOW);
    digitalWrite(rightBin, LOW);
    delay(80);  // OFF time
  }

  // Ensure motors are OFF at the end
  digitalWrite(leftBin, LOW);
  digitalWrite(rightBin, LOW);
}


void rotateBinWhileButtonPressed() {
  if (digitalRead(startButton) == HIGH) {
    // Simulated slow PWM: brief ON, then OFF
    digitalWrite(leftBin, HIGH);
    digitalWrite(rightBin, HIGH);
    delay(30);  // Motor ON time

    digitalWrite(leftBin, LOW);
    digitalWrite(rightBin, LOW);
    delay(70);  // Motor OFF time
  } else {
    // Button not pressed, keep motors off
    digitalWrite(leftBin, LOW);
    digitalWrite(rightBin, LOW);
    delay(10);  // Small delay to avoid CPU overuse
  }
}


long readDistanceCM() {
  digitalWrite(sensorTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(sensorTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorTrig, LOW);

  long duration = pulseIn(sensorEcho, HIGH, 30000);  // Timeout after 30ms
  long distance = duration * 0.034 / 2;
  Serial.print(distance);
  return distance;
}

void forwardEnd() {
  // Set motor direction: FORWARD
  digitalWrite(directionPinBL, LOW);
  digitalWrite(directionPinBR, HIGH);
  digitalWrite(directionPinFL, LOW);
  digitalWrite(directionPinFR, HIGH);

  const int pwmOn = 200;    // microseconds ON
  const int pwmOff = 0;   // microseconds OFF

  while (true) {
    long distance = readDistanceCM();
    if(distance > 20 && distance < 400) {
      break;
    }
    analogWrite(pwmPinBR, 0);
    analogWrite(pwmPinBL, 0);
    analogWrite(pwmPinFR, 0);
    analogWrite(pwmPinFL, 0);

    delay(30);

    analogWrite(pwmPinBR, 255);
    analogWrite(pwmPinBL, 255);
    analogWrite(pwmPinFR, 255);
    analogWrite(pwmPinFL, 255);

    delay(500);
    
  }

  analogWrite(pwmPinBR, 255);
  analogWrite(pwmPinBL, 255);
  analogWrite(pwmPinFR, 255);
  analogWrite(pwmPinFL, 255);
}
