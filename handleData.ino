char longBuff[15];
char latBuff[15];
char tempBuff[15];
char humidityBuff[15];
char accXBuff[15];
char accYBuff[15];
char accZBuff[15];
char altitudeBuff[6];
char speedBuff[15];
char angleBuff[15];

void handleData() {
  led.breath(20);
  int outputLength = 0;
  timer = millis(); // reset the timer
  packetNumber++;
  if (debug) {
    Serial.print("\n-----------------"); Serial.print(" PACKET: "); Serial.print(packetNumber); Serial.println(" -----------------");
    displayGPSRTCReadings();
    displayGPSPositionReadings();
    displaySoilSensorReadings();
    displayAccSensorReading();
    displayUltrasonicSensorReadings();
  }
  sensors_event_t aevent, mevent;
  // get a new sensor event
  accelmag.getEvent(&aevent, &mevent);

  char radiopacket[200];
  radiopacket[0] = 0;
  char latPolar[0];
  char lonPolar[0];

  char comma[] = ",";
  char* timeData = timePacket(GPS.hour, GPS.minute, GPS.seconds);
  char* dateData = datePacket(GPS.month, GPS.day, GPS.year);
  char* tempCelcius = dtostrf(sht1x.readTemperatureC(), 4, 10, tempBuff);
  char* humidityData = dtostrf(sht1x.readHumidity(), 4, 10, humidityBuff);
  char* acc_x = dtostrf(aevent.acceleration.x, 4, 10, accXBuff);
  char* acc_y = dtostrf(aevent.acceleration.y, 4, 10, accYBuff);
  char* acc_z = dtostrf(aevent.acceleration.z, 4, 10, accZBuff);
  char* SPEED = dtostrf(GPS.speed, 4, 10, speedBuff);
  char* angle = dtostrf(GPS.angle, 4, 10, angleBuff);
  char* distance = ltoa(sensorValue, buffer, 10);
  char* latitudeCoord = dtostrf(convertDegMinToDecDeg(GPS.latitude), 4, 10, latBuff);
  char* longitudeCoord = dtostrf(convertDegMinToDecDeg(GPS.longitude), 4, 10, longBuff);
  latPolar[0] = GPS.lat;
  latPolar[1] = '\0';
  lonPolar[0] = GPS.lon;
  lonPolar[1] = '\0';
  char* altitude = dtostrf(GPS.altitude, 4, 2, altitudeBuff);

  strcat(radiopacket, SENSOR_ID);
  outputLength += strlen(SENSOR_ID);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, timeData);
  outputLength += strlen(timeData);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, dateData);
  outputLength += strlen(dateData);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, latitudeCoord);
  outputLength += strlen(latitudeCoord);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, latPolar);
  outputLength += strlen(latPolar);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, longitudeCoord);
  outputLength += strlen(longitudeCoord);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, lonPolar);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, altitude);
  outputLength += strlen(altitude);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, SPEED);
  outputLength += strlen(SPEED);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, angle);
  outputLength += strlen(angle);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, tempCelcius);
  outputLength += strlen(tempCelcius);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, humidityData);
  outputLength += strlen(humidityData);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, acc_x);
  outputLength += strlen(acc_x);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, acc_y);
  outputLength += strlen(acc_y);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, acc_z);
  outputLength += strlen(acc_z);
  strcat(radiopacket, comma); outputLength++;
  strcat(radiopacket, distance);
  outputLength += strlen(distance);

  if (debug) Serial.print("\nOutput size: "); Serial.println(outputLength);
  //  if (debug) Serial.print("radiopacket: "); Serial.println(radiopacket);
  sendRadiopacket(radiopacket);

  free(timeData);
  free(dateData);
}

