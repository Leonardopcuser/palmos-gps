// Main code for Sensor Readings //


// 9 DOF Sensor Libraries from Adafruit with I2C through Wire
#include <Wire.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_FXOS8700.h>


// Soil temp and moisture sensor Libraries from Adafruit
#include <SHT1x.h>

// GPS Library from Adafruit
#include <Adafruit_GPS.h>

// Independent Modules with relevant functions
#include "Sensors/Accelerometer/accelerometer_display_functions.h"
#include "Sensors/Soil/soil_sensor_functions.h"
#include "GPS/gps_reading_functions.h"
#include "Sensors/Range_Finder/range_finder_functions.h"

// Various definitions for timing etc.

#define FREQUENCY 1000 // Number of Readings/transmission per hour
#define GPSSerial Serial1 // Serial port used for GPS transmission. Careful if using ESP 8266, Serial is used for bootloading

#define HOUR_CONSTANT 3600000 // Number of ms in an hour, used to calculate delay in ms

// Connect to the GPS on the hardware port
// Adafruit_GPS GPS(&GPSSerial);

#define GPS_FIX_TIMEOUT 5 // Number Of attempts at getting a GPS Fix
#define GPSECHO false

int time_delay = HOUR_CONSTANT / FREQUENCY;
uint32_t timer = millis();

void setup(void)
{

  int i = 1;
  bool fix_timeout = false;

  if (debug) Serial.begin(9600);
    
  // Initialize GPS Module
  GPS.begin(9600);

  /* Wait for the Serial Monitor */
    if (debug) {
  while (!Serial) {
    delay(1);
  }
    }

  if (debug) Serial.println("Sensor & GPS Reading Begins"); Serial.println("");
  if (debug) Serial.print("Frequency of Data Aquisition: "); Serial.print(FREQUENCY); Serial.println("/h");
  if (debug) Serial.print("Time Delay: "); Serial.print(time_delay); Serial.println(" ms");

  /* Initialise the 9 DOF sensor */
  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    if (debug) Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }


  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // RMC and GGA format, including altitude for Parsing
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

}


void loop(void)
{

  GPSMainRead();

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis()) timer = millis();
     
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > time_delay) {

  timer = millis(); // reset the timer
    if (debug) {
        displayGPSRTCReadings();
        displayGPSPositionReadings();
        displaySoilSensorReadings();
        displayAccSensorReading();
    }
  }

}
