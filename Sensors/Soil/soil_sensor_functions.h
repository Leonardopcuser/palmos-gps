#include <SHT1x.h>

// Specify data and clock connections and instantiate SHT1x object
#define dataPin  12
#define clockPin 13
SHT1x sht1x(dataPin, clockPin);

/* Displays Soil Sensor Reading when Called*/

bool sensor_debug = true;

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
    if (sensor_debug) {
        Serial.println("SOIL:");
        Serial.print("\tTemperature (ºC): ");
        Serial.println(temp_c, DEC);
        Serial.print("\tTemperature (ºF): ");
        Serial.println(temp_f, DEC);
  Serial.print("\tHumidity:\t  ");
  Serial.print(humidity);
  Serial.println("%");
    }

}
