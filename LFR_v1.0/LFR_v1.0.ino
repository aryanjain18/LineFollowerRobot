#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#include <SparkFun_TB6612.h>

// Enter Line Details
bool isBlackLine = false;           // Keep 1 in case of black line. In case of white line change this to 0
unsigned int lineThickness = 25;    // Enter line thickness in mm. Works best for thickness between 10 & 35
unsigned int numSensors = 7;        // Enter number of sensors as 5 or 7
bool brakeEnabled = 0;

#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

// These constants are used to allow you to make your motor configuration
// line up with function names like forward. Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors. The library will allow you to initialize as many
// motors as you have memory for. If you are using functions like forward
// that take 2 motors as arguments you can either write new functions or
// call the function more than once.
Motor motor2 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor1 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfSpeed = 150;     // Left and right wheel speeds for line following
int currentSpeed = 50; // Initial speed

float Kp = 0.06; // Proportional gain
float Kd = 1.5;  // Derivative gain
float Ki = 0;    // Integral gain

int onLine = 1;
int minValues[7], maxValues[7], threshold[7], sensorValue[7], sensorArray[7];
bool brakeFlag = 0;

void setup() {
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  Serial.begin(9600);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);
  lineThickness = constrain(lineThickness, 10, 35); // Ensure line thickness is within bounds
}

void loop() {
  while (digitalRead(11)) {} // Wait for button press
  delay(1000);               // Delay for stability
  calibrate();               // Calibrate sensors
  while (digitalRead(12)) {} // Wait for button press
  delay(1000);               // Delay for stability

  while (1) { // Infinite loop
    readLine(); // Read line sensors
    if (currentSpeed < lfSpeed) currentSpeed++; // Accelerate if below desired speed
    if (onLine == 1) {  // If on line, perform line following
      linefollow();
      digitalWrite(13, HIGH); // Indicate line following
      brakeFlag = 0; // Reset brake flag
    } else { // If not on line
      digitalWrite(13, LOW); // Turn off indicator LED
      if (error > 0) { // If error is positive (turning left)
        if (brakeEnabled == 1 && brakeFlag == 0) { // If brake is enabled and not already applied
          motor1.drive(0); // Brake motor 1
          motor2.drive(0); // Brake motor 2
          delay(30); // Wait for stability
        }
        motor1.drive(-100); // Turn left
        motor2.drive(150); // Turn right
        brakeFlag = 1; // Set brake flag
      } else { // If error is negative (turning right)
        if (brakeEnabled == 1 && brakeFlag == 0) { // If brake is enabled and not already applied
          motor1.drive(0); // Brake motor 1
          motor2.drive(0); // Brake motor 2
          delay(30); // Wait for stability
        }
        motor1.drive(150); // Turn right
        motor2.drive(-100); // Turn left
        brakeFlag = 1; // Set brake flag
      }
    }
  }
}

void linefollow() {
  if (numSensors == 7) {
    error = (3 * sensorValue[0] + 2 * sensorValue[1] + sensorValue[2] - sensorValue[4] - 2 * sensorValue[5] - 3 * sensorValue[6]);
  }
  if (numSensors == 5) {
    error = (3 * sensorValue[1] + sensorValue[2] - sensorValue[4] - 3 * sensorValue[5]);
  }
  if (lineThickness > 22) {
    error = error * -1; // Reverse error if line thickness is greater than 22mm
  }
  if (isBlackLine) {
    error = error * -1; // Reverse error if it's a black line
  }

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = currentSpeed - PIDvalue;
  rsp = currentSpeed + PIDvalue;

  if (lsp > 255) lsp = 255; // Limit speed to maximum
  if (lsp < 0) lsp = 0;     // Ensure speed is not negative
  if (rsp > 255) rsp = 255; // Limit speed to maximum
  if (rsp < 0) rsp = 0;     // Ensure speed is not negative

  motor1.drive(lsp); // Drive left motor
  motor2.drive(rsp); // Drive right motor
}

void calibrate() {
  for (int i = 0; i < 7; i++) {
    minValues[i] = analogRead(i); // Initialize minimum sensor values
    maxValues[i] = analogRead(i); // Initialize maximum sensor values
  }

  for (int i = 0; i < 10000; i++) { // Iterate for calibration
    motor1.drive(40); // Drive motors for calibration
    motor2.drive(-40); // Drive motors for calibration

    for (int i = 0; i < 7; i++) { // Iterate through sensors
      if (analogRead(i) < minValues[i]) {
        minValues[i] = analogRead(i); // Update minimum sensor values
      }
      if (analogRead(i) > maxValues[i]) {
        maxValues[i] = analogRead(i); // Update maximum sensor values
      }
    }
  }

  for (int i = 0; i < 7; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2; // Calculate threshold for each sensor
    Serial.print(threshold[i]); // Print threshold values
    Serial.print(" ");
  }
  Serial.println();

  motor1.drive(0); // Stop left motor
  motor2.drive(0); // Stop right motor
}

void readLine() {
  onLine = 0; // Reset line detection flag
  if (numSensors == 7) {
    for (int i = 0; i < 7; i++) {
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000); // Map sensor values
      sensorValue[i] = constrain(sensorValue[i], 0, 1000); // Constrain sensor values
      if (sensorValue[i] > 700) onLine = 1; // Check if sensor is on line
    }
  }
  if (numSensors == 5) {
    for (int i = 1; i < 6; i++) {
      sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000); // Map sensor values
      sensorValue[i] = constrain(sensorValue[i], 0, 1000); // Constrain sensor values
      if (sensorValue[i] > 700) onLine = 1; // Check if sensor is on line
    }
  }
}
