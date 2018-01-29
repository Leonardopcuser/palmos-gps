void initializeSensor() {

  // Radio Setup
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);
  if (WAIT_FOR_SERIAL) while (!Serial);
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  /*Initializing the RF Radio */
  if (!rf95.init()) {
    if (debug) Serial.println("LoRa radio init failed");
    while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    if (debug) Serial.println("setFrequency failed");
    while (1);
  }

  /* Initialise the 9 DOF sensor */
  if (!accelmag.begin(ACCEL_RANGE_4G))
  {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    if (debug) Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1);
  }

  // Initialize GPS Module
  GPS.begin(9600);

  if (debug) {
    Serial.println("Sensor & GPS Transmission"); Serial.println("");
    Serial.print("Frequency of Transmission: "); Serial.print(FREQUENCY); Serial.println("/h");
    Serial.print("Time Delay: "); Serial.print(time_delay); Serial.println(" ms");
  }

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // RMC and GGA format, including altitude for Parsing
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate

  rf95.setTxPower(23, false); //rf95
}

