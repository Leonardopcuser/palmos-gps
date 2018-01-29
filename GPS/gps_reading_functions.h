    // Test code for Ultimate GPS Using Hardware Serial (e.g. GPS Flora or FeatherWing)
//
// This code shows how to listen to the GPS module via polling. Best used with
// Feathers or Flora where you have hardware Serial and no interrupt
//
// Tested and works great with the Adafruit GPS FeatherWing
// ------> https://www.adafruit.com/products/3133
// or Flora GPS
// ------> https://www.adafruit.com/products/1059
// but also works with the shield, breakout
// ------> https://www.adafruit.com/products/1272
// ------> https://www.adafruit.com/products/746
// 
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada
     
#include <Adafruit_GPS.h>

// what's the name of the hardware serial port?
// #define GPSSerial Serial1

// Connect to the GPS on the hardware port
// Adafruit_GPS GPS(&GPSSerial);
     
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false


#define GPSSerial Serial1 // Serial port used for GPS transmission. Careful if using ESP 8266, Serial is used for bootloading
Adafruit_GPS GPS(&GPSSerial);

//uint32_t timer = millis();

void GPSMainRead() // to be called before every other parsing function
{
      // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    // Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
}
// added by Drew on 1/28/18, adapted from http://arduinodev.woofex.net/2013/02/06/adafruit_gps_forma/
double convertDegMinToDecDeg (float degMin) {
    double min = 0.0;
    double decDeg = 0.0;
    
    //get the minutes, fmod() requires double
    min = fmod((double)degMin, 100.0);
    
    //rebuild coordinates in decimal degrees
    degMin = (int) ( degMin / 100 );
    decDeg = degMin + ( min / 60 );
    
    return decDeg;
}

void displayGPSPositionReadings()
{
Serial.println("GPS:");
//  Serial.print("\tFix:\t\t  "); Serial.print((int)GPS.fix);
//  Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    Serial.print("\tLatitude:\t  "); Serial.print(GPS.latitude); Serial.print(GPS.lat); Serial.println(" (decimal minutes)");
  Serial.print("\tLongitude:\t  "); Serial.print(GPS.longitude); Serial.print(GPS.lon); Serial.println(" (decimal minutes)");
  Serial.print("\tLatitude:\t  "); Serial.print(convertDegMinToDecDeg(GPS.latitude)); Serial.print(GPS.lat); Serial.println(" (decimal degrees)");
Serial.print("\tLongitude:\t  "); Serial.print(convertDegMinToDecDeg(GPS.longitude)); Serial.print(GPS.lon); Serial.println(" (decimal degrees)");
  Serial.print("\tSpeed:\t\t  "); Serial.print(GPS.speed); Serial.println(" (knots)");
  Serial.print("\tAngle:\t\t  "); Serial.println(GPS.angle);
    Serial.print("\tAltitude:\t  "); Serial.println(GPS.altitude);
//  Serial.print("\tSatellites:\t  "); Serial.println((int)GPS.satellites);
}

void displayGPSRTCReadings() // Function for reading Real time Clock and Date data
{
  Serial.print("    Time: ");
  Serial.print(GPS.hour, DEC); Serial.print(':');
  Serial.print(GPS.minute, DEC); Serial.print(':');
  Serial.print(GPS.seconds, DEC); Serial.print('.');
  Serial.print(GPS.milliseconds);
  Serial.print("  Date: ");
  Serial.print(GPS.day, DEC); Serial.print('/');
  Serial.print(GPS.month, DEC); Serial.print("/20");
  Serial.println(GPS.year, DEC);
}
