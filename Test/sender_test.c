#include <math.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_Sensor.h> 
#include <Adafruit_FXOS8700.h>
#include <SHT1x.h>
#include <Adafruit_GPS.h>
#include "Sensors\Accelerometer\accelerometer_display_functions.h"
#include "Sensors\Soil\soil_sensor_functions.h"
#include "GPS\gps_reading_functions.h"
#define RFM95_RST     11   // "A"
#define RFM95_CS      10   // "B"
#define RFM95_INT     6    // "D"

#define FREQUENCY 1000 // Number of Readings/transmission per hour
#define GPSSerial Serial1 // Serial port used for GPS transmission. Careful if using ESP 8266, Serial is used for bootloading

#define HOUR_CONSTANT 3600000 // Number of ms in an hour, used to calculate delay in ms

#define PACKET_TIMEOUT 3000

#define RF95_FREQ 915.0 // Defines the frequency of radio. Must be same as receiver.

#define GPS_FIX_TIMEOUT 5 // Number Of attempts at getting a GPS Fix
#define GPSECHO false

RH_RF95 rf95(RFM95_CS, RFM95_INT); // Initilizing the Radio

int time_delay = HOUR_CONSTANT / FREQUENCY;
uint32_t timer = millis();

#include <pt.h>   // include protothread library

#define LEDPIN 13  // LEDPIN is a constant 

static struct pt pt1, pt2; // each protothread needs one of these

void setup() {
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
  PT_INIT(&pt1);  // initialise the two
  PT_INIT(&pt2);  // protothread variables
}


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

  char monthStr[2];
  char dayStr[2];
  char yearStr[4];
  char dash[] = "/";
  char comma[] = ",";

  char *dateStr = (char*) malloc(10);

  itoa(month, monthStr, 10);
  itoa(day, dayStr, 10);
  itoa(year, yearStr, 10);

  strcpy(dateStr, monthStr);
  strcat(dateStr, dash);
  strcat(dateStr, dayStr);
  strcat(dateStr, dash);
  strcat(dateStr, yearStr);
  strcat(dateStr, comma);

  return dateStr;

}



static int protothread1_gps(struct pt *pt, int interval) {
    static unsigned long timestamp = 0;
    PT_BEGIN(pt);
    while(1) { // never stop 
    
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis(); // take a new timestamp'

    
    GPSMainRead();
    displayGPSRTCReadings();
    char GPS_radiopacket[60];
    memset(GPS_radiopacket, 0, sizeof(GPS_radiopacket)/sizeof(char));
    char comma[] = ",";
    char* timeData = timePacket(GPS.hour, GPS.minute, GPS.seconds);
    char* dateData = datePacket(GPS.month, GPS.day, GPS.year);
    char* latitudeCoord = ftoa(GPS.latitude, 4);
    char latPolar = GPS.lat;
    char* longitudeCoord = ftoa(GPS.longitude, 4);
    char lonPolar = GPS.lon;
    char* altitude = ftoa(GPS.altitude, 2);

    strcat(GPS_radiopacket, timeData);
    strcat(GPS_radiopacket, dateData);
    strcat(GPS_radiopacket, latitudeCoord);
    strcat(GPS_radiopacket, &latPolar);
    strcat(GPS_radiopacket, comma);
    strcat(GPS_radiopacket, longitudeCoord);
    strcat(GPS_radiopacket, &lonPolar);
    strcat(GPS_radiopacket, comma);
    strcat(GPS_radiopacket, altitude);
    strcat(GPS_radiopacket, "\0");
    Serial.print("Sending GPS Packet: "); 
    Serial.println(GPS_radiopacket);
    Serial.println("Sending..."); delay(10);
    rf95.send((uint8_t *)GPS_radiopacket, 50);
    Serial.println("Waiting for packet to complete..."); delay(10);
    rf95.waitPacketSent();
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    Serial.println("Waiting for reply..."); delay(10);
    if (rf95.waitAvailableTimeout(PACKET_TIMEOUT))
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

    free(timeData);
    free(dateData);
    free(latitudeCoord);
    free(longitudeCoord);
    free(altitude);
    
  }
  PT_END(pt);
}
/* exactly the same as the protothread1 function */
static int protothread2_sensor(struct pt *pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis();
    displaySoilSensorReadings();
    displayAccSensorReading();
    sensors_event_t aevent, mevent;
    accelmag.getEvent(&aevent, &mevent);

    char Sensor_radiopacket[60];
    char comma[] = ",";
    char* tempCelcius = ftoa(sht1x.readTemperatureC(), 4);
    char* humidityData = ftoa(sht1x.readHumidity(), 4);
    char* acc_x = ftoa(aevent.acceleration.x, 4);
    char* acc_y = ftoa(aevent.acceleration.y, 4);
    char* acc_z = ftoa(aevent.acceleration.z, 4);
    memset(Sensor_radiopacket, 0, sizeof( Sensor_radiopacket)/sizeof(char));
    strcat(Sensor_radiopacket, "S");
    strcat(Sensor_radiopacket, tempCelcius);
    strcat(Sensor_radiopacket, comma);
    strcat(Sensor_radiopacket, humidityData);
    strcat(Sensor_radiopacket, comma);
    strcat(Sensor_radiopacket, acc_x);
    strcat(Sensor_radiopacket, comma);
    strcat(Sensor_radiopacket, acc_y);
    strcat(Sensor_radiopacket, comma);
    strcat(Sensor_radiopacket, acc_z);
    strcat(Sensor_radiopacket, "\0");

    Serial.print("Sending Sensor Packet: "); Serial.println(Sensor_radiopacket);
    Serial.println("Sending..."); delay(10);
    rf95.send((uint8_t *)Sensor_radiopacket, 60);
    Serial.println("Waiting for packet to complete..."); delay(10);
    rf95.waitPacketSent();

    Serial.println("Waiting for reply..."); delay(10);
    if (rf95.waitAvailableTimeout(PACKET_TIMEOUT))
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
    
  }
  PT_END(pt);
  
  free(tempCelcius);
  free(humidityData);
  free(acc_x);
  free(acc_y);
  free(acc_z);
}


void loop() {
  protothread1_gps(&pt1, 1000); // schedule the two protothreads
  protothread2_sensor(&pt2, 1000);
  
}
