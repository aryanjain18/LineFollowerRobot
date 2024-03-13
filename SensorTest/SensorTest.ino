/*
  Sample code to test the line sensor in the LFR robot
*/

void setup() {
  // Initialize serial communication at 9600 baud rate
  Serial.begin(9600);
}

void loop() {
  // Read analog values from line sensors
  int sensorValue0 = analogRead(A0);
  int sensorValue1 = analogRead(A1);
  int sensorValue2 = analogRead(A2);
  int sensorValue3 = analogRead(A3);
  int sensorValue4 = analogRead(A4);
  int sensorValue5 = analogRead(A5);
  int sensorValue6 = analogRead(A6);
  
  // Print out the analog values of each sensor
  Serial.print(sensorValue0);
  Serial.print("   ");
  Serial.print(sensorValue1);
  Serial.print("   ");
  Serial.print(sensorValue2);
  Serial.print("   ");
  Serial.print(sensorValue3);
  Serial.print("   ");
  Serial.print(sensorValue4);
  Serial.print("   ");
  Serial.print(sensorValue5);
  Serial.print("   ");
  Serial.println(sensorValue6);
  
  // Delay for 50 milliseconds before next reading
  delay(50);
}
