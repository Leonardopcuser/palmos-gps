int sensorPin = A3;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // read the value from the sensor with scaling factor for max read distance 5000mm:
  sensorValue = analogRead(sensorPin)*(5000/1023);
  // delay the print for x ms:
  delay(500);
  Serial.println(sensorValue);
  return sensorValue;
}