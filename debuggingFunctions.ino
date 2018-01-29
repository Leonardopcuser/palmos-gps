void printSensorRelayConnection() {
  if (rf95.available() && !foundMaster && debug) {
    Serial.println("Connected with Master Node");
    foundMaster = true;
  } else {
    foundMaster = false;
  }
}
