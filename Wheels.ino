//BLUE - PWM - orange
//BLACK - POWER -ve
//YELLOW - DIRECTION - purple
//GREEN - FG - grey
//RED - POWER +v


int pwmPinBL = 2;
int directionPinBL = 23;
int fgPinBL = 25;

int pwmPinBR = 3;
int directionPinBR = 22;
int fgPinBR = 24;

int pwmPinFL = 12;
int directionPinFL = 51;
int fgPinFL = 49;

int pwmPinFR = 13;
int directionPinFR = 50;
int fgPinFR = 48;

void setup() {
  // Set the direction and PWM pins as outputs
  Serial.begin(9600);

  pinMode(pwmPinBL, OUTPUT);
  pinMode(directionPinBL, OUTPUT);
  pinMode(fgPinBL, INPUT);

  pinMode(pwmPinBR, OUTPUT);
  pinMode(directionPinBR, OUTPUT);
  pinMode(fgPinBR, INPUT);

  pinMode(pwmPinFL, OUTPUT);
  pinMode(directionPinFL, OUTPUT);
  pinMode(fgPinFL, INPUT);

  pinMode(pwmPinFR, OUTPUT);
  pinMode(directionPinFR, OUTPUT);
  pinMode(fgPinFR, INPUT);

  /// Stop all motors
  analogWrite(pwmPinBR, 255);
  analogWrite(pwmPinBL, 255);
  analogWrite(pwmPinFR, 255);
  analogWrite(pwmPinFL, 255);

  delay(5000);
}

void loop() {
  right(450);
  delay(2000);
  right(70);
  delay(2000);
  right(70);
  delay(2000);
  forward(3050);
  backward(20);
  delay(5000);
  forward(400);
  delay(5000);
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
  analogWrite(pwmPinBR, 255);
  analogWrite(pwmPinBL, 255);
  analogWrite(pwmPinFR, 255);
  analogWrite(pwmPinFL, 255);
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
