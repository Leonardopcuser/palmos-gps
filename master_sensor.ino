// Main code for Sensor Node Reading and Transmitting //
// Written by Karim Abdallah. July 19 2017. Save the Date.
//
// v 1.0. Includes sensor readings, encoding of data according to KSK Standard, and transmitting via LoRa at 900 Mhz.
// Ain't she a beauty.
// Libraries and inspiration courtesy of Adafruit and various partners.

//#include <TimerOne.h>
#include <math.h>
#include <avr/dtostrf.h>

// RF and SPI Libraries
#include <SPI.h>
#include <RH_RF95.h>

// 9 DOF Sensor Libraries from Adafruit with I2C through Wire
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>c

// Soil temp and moisture sensor Libraries from Adafruit
#include <SHT1x.h>

// GPS Library from Adafruit
#include <Adafruit_GPS.h>

// Independant Modules with relevant functions
#include "Sensors/Accelerometer/accelerometer_display_functions.h"
#include "Sensors/Soil/soil_sensor_functions.h"
#include "GPS/gps_reading_functions.h"
#include "Sensors/Range_Finder/range_finder_function.h"

// Sensor Tag: modify to attribute the "ID" of the sensor (A, B, C...)
// Will think of a sexier way to name it but however brilliant KS is, he's not the best at sexy codenames...
#define SENSOR_ID "A" // "A"

// Pin definition for RF Radio (based on Adafruit wiring recommendations and Adalogger M0 Pinout)
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"

#define FREQUENCY 200 // Number of Readings/transmission per hour
#define GPSSerial Serial1 // Serial port used for GPS transmission. Careful if using ESP 8266, Serial is used for bootloading

#define HOUR_CONSTANT 3600000 // Number of ms in an hour, used to calculate delay in ms

#define RF95_FREQ 915.0 // Defines the frequency of radio. Must be same as receiver.
#define RF_TIMEOUT 1000 // Defines timeout timeout time of RF transmission. -> GPS Data works best with 1000 timeout

#define GPS_FIX_TIMEOUT 5 // Number Of attempts at getting a GPS Fix
#define GPSECHO false

//#define GPS_DELAY 5 // Multiplier for how longer should the time delay for the GPS data be as a multiple of Sensor data GPS delay

RH_RF95 rf95(RFM95_CS, RFM95_INT); // Initilizing the Radio

int time_delay = HOUR_CONSTANT / FREQUENCY;
uint32_t timer = millis();

bool debug = true; // global variable to switch between normal vs debugger mode.
bool WAIT_FOR_SERIAL = false;
bool foundMaster = false;

long packetNumber = 0;

#include <LEDEffect.h>
LEDEffect led(13);

void setup() {
  initializeSensor();
}

void loop() {
  led.breath(20);
  led.update();

  GPSMainRead();

  if (debug) printSensorRelayConnection();

  if (timer > millis()) timer = millis();
  if (millis() - timer > time_delay) {
    handleData();
  }
}

