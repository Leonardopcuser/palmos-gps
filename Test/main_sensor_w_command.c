// Main code for Sensor Node Reading and Transmitting //
// Written by Karim Abdallah. July 19 2017. Save the Date.
//
// v 1.0. Includes sensor readings, encoding of data according to KSK Standard, and transmitting via LoRa at 900 Mhz.
// Ain't she a beauty.
// Libraries and inspiration courtesy of Adafruit and various partners.


#include <math.h>

// RF and SPI Libraries
#include <SPI.h>
#include <RH_RF95.h>

// 9 DOF Sensor Libraries from Adafruit with I2C through Wire
#include <Wire.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_FXOS8700.h>

// Soil temp and moisture sensor Libraries from Adafruit
#include <SHT1x.h>

// GPS Library from Adafruit
#include <Adafruit_GPS.h>

// Independant Modules with relevant functions
#include "Sensors\Accelerometer\accelerometer_display_functions.h"
#include "Sensors\Soil\soil_sensor_functions.h"
#include "GPS\gps_reading_functions.h"

// Various definitions for timing etc.

// Pin definition for RF Radio (based on Adafruit wiring recommendations and Adalogger M0 Pinout)
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"

#define FREQUENCY 1000 // Number of Readings/transmission per hour
#define GPSSerial Serial1 // Serial port used for GPS transmission. Careful if using ESP 8266, Serial is used for bootloading

#define HOUR_CONSTANT 3600000 // Number of ms in an hour, used to calculate delay in ms

#define RF95_FREQ 915.0 // Defines the frequency of radio. Must be same as receiver.
#define RF_TIMEOUT 1000 // Defines timeout timeout time of RF transmission. -> GPS Data works best with 1000

#define GPS_FIX_TIMEOUT 5 // Number Of attempts at getting a GPS Fix
#define GPSECHO false

RH_RF95 rf95(RFM95_CS, RFM95_INT); // Initilizing the Radio

int time_delay = HOUR_CONSTANT / FREQUENCY;
uint32_t timer = millis();
uint32_t timer_2 = millis();

void setup(void)
{

  // Radio Setup
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

    /* Wait for the Serial Monitor */
  while (!Serial) {
    delay(1);
  }

  Serial.begin(9600);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

    /*Initializing the RF Radio */
  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }

    /* Initialise the 9 DOF sensor */
  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }

  //int i = 1;
  //bool fix_timeout = false;
    
  // Initialize GPS Module
  GPS.begin(9600);

  Serial.println("Sensor & GPS Transmission"); Serial.println("");
  Serial.print("Frequency of Transmission: "); Serial.print(FREQUENCY); Serial.println("/h");
  Serial.print("Time Delay: "); Serial.print(time_delay); Serial.println(" ms");


  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // RMC and GGA format, including altitude for Parsing
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

  rf95.setTxPower(23, false); //rf95
  
  /* Display some basic information on 9 DOF this sensor */
  // displaySensorDetails();

  // Waiting for GPS Fix

  /* while (!fix_timeout){

    Serial.print("Attempt number "); Serial.print(i); Serial.println(" at finding a GPS fix");

    if (GPS.fix){
      Serial.println("Fix Found");
      Serial.print("GPS Coordinates: "); readGPSPositionData();
      delay(1000);
      fix_timeout = true;
    }
    else {
      Serial.println("Fix not Found");
      if (i==GPS_FIX_TIMEOUT)
        fix_timeout = true;
        i++;
    }
    delay(1000);
  }
  */

  printMenu();
}

void printMenu(void)
{

  Serial.println("---------------------------------");
  Serial.println("Welcome to Control Version of Sensor Node");
  Serial.println("Select from Options:");
  Serial.println("s) Send Sensor Data");
  Serial.println("g) Send GPS Data");
  Serial.println("k) Print out cool shit");


}
// Converts a Float to a char string, with specified decimal points. If no decimal point count given, returns 2 dec points. 

char* ftoa(float data, int dec_count = 2)
{
    // Converts x acceleration data to string to 4 dec points
  float fNum = data;
  // Serial.print("Acceleration: "); Serial.println(fNum);
  int wholeNum = fNum;
  // Serial.print("Whole value: "); Serial.println(wholeNum);
  int decimalNum = (int)((fNum - wholeNum) * pow(10,dec_count));
  // Serial.print("Decimal value: "); Serial.println(decimalNum);
  char wholeNumStr[10];
  char decNumStr[5];
  char decPtStr[] = ".";
  char *numStr = (char *)malloc(17);
  itoa(wholeNum, wholeNumStr, 10);
  // Serial.print("Whole Value Char: "); Serial.println(wholeNumStr);
  itoa(decimalNum, decNumStr, 10);
  // Serial.print("Decimal Value Char: "); Serial.println(decNumStr);
  strcpy(numStr, wholeNumStr);
  strcat(numStr, decPtStr);
  strcat(numStr, decNumStr);

  //Serial.print("Char Value: "); Serial.println(numStr);

  return numStr;
}

char* timePacket(int hours, int minutes, int seconds) // Takes in time data from GPS and converts it to KSK standardized string.
{

// KSK standard: dash "/" delineates hours from minutes from seconds, end of packet type delineated by comma ","

  char hoursStr[2];
  char minutesStr[2];
  char secondsStr[2];
  char dash[] = "/";
  char comma[] = ",";

  char *timeStr = (char*) malloc(10);

  itoa(hours, hoursStr, 10);
  itoa(minutes, minutesStr, 10);
  itoa(seconds, secondsStr, 10);

  strcpy(timeStr, hoursStr);
  strcat(timeStr, dash);
  strcat(timeStr, minutesStr);
  strcat(timeStr, dash);
  strcat(timeStr, secondsStr);
  strcat(timeStr, comma);

  return timeStr;

}

char* datePacket(int month, int day, int year) // Takes in time data from GPS and converts it to KSK standardized string.
{

// KSK standard: dash "/" delineates hours from minutes from seconds, end of packet type delineated by comma ","

  // Serial.print("Year before ItoA: "); Serial.println(year);

  char monthStr[2];
  char dayStr[2];
  char yearStr[4];
  char dash[] = "/";
  char comma[] = ",";

  char *dateStr = (char*) malloc(10);

  itoa(month, monthStr, 10);
  itoa(day, dayStr, 10);
  itoa(year, yearStr, 10);
  // Serial.print("Converted Date: "); Serial.println(yearStr);

  strcpy(dateStr, monthStr);
  strcat(dateStr, dash);
  strcat(dateStr, dayStr);
  strcat(dateStr, dash);
  strcat(dateStr, yearStr);
  strcat(dateStr, comma);

  return dateStr;

}

void sendRadiopacket(char* radiopacket)
{

  Serial.print("Sending "); Serial.println(radiopacket);
  // radiopacket[19] = 0;
  
  Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, 60);
 
  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
 
  Serial.println("Waiting for reply..."); delay(10);
  if (rf95.waitAvailableTimeout(RF_TIMEOUT))
  { 
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
  //delay(seconds);


}

void loop(void)
{

  char command = Serial.read();
  // Serial.println(command);

  switch (command) {

    case 's': {

      GPSMainRead();
      displayGPSRTCReadings();
      // displayGPSPositionReadings();
      displaySoilSensorReadings();
      displayAccSensorReading();

      sensors_event_t aevent, mevent;

    /* Get a new sensor event */
      accelmag.getEvent(&aevent, &mevent);

      Serial.println("Sending sensor Data and Timestamp");

      char radiopacket[60];
      memset(radiopacket, 0, strlen(radiopacket)/sizeof(char));

      char comma[] = ",";
      char* timeData = timePacket(GPS.hour, GPS.minute, GPS.seconds);
      char* dateData = datePacket(GPS.month, GPS.day, GPS.year);
      char* tempCelcius = ftoa(sht1x.readTemperatureC(), 4);
      char* humidityData = ftoa(sht1x.readHumidity(), 4);
      char* acc_x = ftoa(aevent.acceleration.x, 4);
      char* acc_y = ftoa(aevent.acceleration.y, 4);
      char* acc_z = ftoa(aevent.acceleration.z, 4);

      strcpy(radiopacket, timeData);
      strcat(radiopacket, dateData);
      strcat(radiopacket, tempCelcius);
      strcat(radiopacket, comma);
      strcat(radiopacket, humidityData);
      strcat(radiopacket, comma);
      strcat(radiopacket, acc_x);
      strcat(radiopacket, comma);
      strcat(radiopacket, acc_y);
      strcat(radiopacket, comma);
      strcat(radiopacket, acc_z);
      strcat(radiopacket, comma);
      strcat(radiopacket, "S");

      sendRadiopacket(radiopacket);

      free(timeData);
      free(dateData);
      free(tempCelcius);
      free(humidityData);
      free(acc_x);
      free(acc_y);
      free(acc_z);

      printMenu();

    }


    case 'g': {

      timer_2 = millis(); // reset the timer
      displayGPSRTCReadings();
      displayGPSPositionReadings();
      // displaySoilSensorReadings();
      // displayAccSensorReading();
      
      // sensors_event_t aevent, mevent;

      /* Get a new sensor event */
      // accelmag.getEvent(&aevent, &mevent);

      Serial.println("Sending GPS Data");

      char radiopacket[60];
      memset(radiopacket, 0, strlen(radiopacket)/sizeof(char));
      
      char comma[] = ",";
      char* timeData = timePacket(GPS.hour, GPS.minute, GPS.seconds);
      char* dateData = datePacket(GPS.month, GPS.day, GPS.year); 
      char* latitudeCoord = ftoa(GPS.latitude, 4);
      // char latPolar = GPS.lat; // uncomment when testing with GPS signal and remove "S"
      char latPolar[] = "N"; // comment when testing with GPS signal
      char* longitudeCoord = ftoa(GPS.longitude, 4);
      // char lonPolar = GPS.lon; // uncomment when testing with GPS signal
      char lonPolar[] = "W"; // comment when testing with GPS signal
      char* altitude = ftoa(GPS.altitude, 2);

      strcpy(radiopacket, timeData);
      //strcpy(radiopacket, "G");
      strcat(radiopacket, dateData);
      strcat(radiopacket, latitudeCoord);
      // strcat(radiopacket, &latPolar);
      strcat(radiopacket, latPolar);
      strcat(radiopacket, comma);
      strcat(radiopacket, longitudeCoord);
      strcat(radiopacket, lonPolar);
      // strcat(radiopacket, lonPolar);  
      strcat(radiopacket, comma);
      strcat(radiopacket, altitude);
      strcat(radiopacket, comma);

      sendRadiopacket(radiopacket);

      free(timeData);
      free(dateData);
      free(latitudeCoord);
      free(longitudeCoord);
      free(altitude);

      printMenu();
    }

    case 'k': {

      Serial.println("Cool shit (cooler shit to come...)");

      printMenu();
    }

  }


}
