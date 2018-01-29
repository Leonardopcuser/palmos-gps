#include <SHT1x.h>

// Specify data and clock connections and instantiate SHT1x object
#define dataPin  12
#define clockPin 13
SHT1x sht1x(dataPin, clockPin);

/* Displays Soil Sensor Reading when Called*/

void displaySoilSensorReadings(void)

{

  float temp_c;
  float temp_f;
  float humidity;

  // Read values from the sensor
  temp_c = sht1x.readTemperatureC();
  temp_f = sht1x.readTemperatureF();
  humidity = sht1x.readHumidity();

  // Print the values to the serial port
  Serial.print("Temperature: ");
  Serial.print(temp_c, DEC);
  Serial.print("C / ");
  Serial.print(temp_f, DEC);
  Serial.print("F. Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

}
