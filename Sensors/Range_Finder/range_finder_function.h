int sensorPin = A3;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor
char buffer[16];

void displayUltrasonicSensorReadings(void) {
	
  // read the value from the sensor with scaling factor for max read distance 5000mm:
  sensorValue = analogRead(sensorPin)*(5000/1023);
  // delay the print for x ms:
  delay(500);
    Serial.print("DISTANCE: ");  Serial.print(sensorValue); Serial.println("mm");
}
