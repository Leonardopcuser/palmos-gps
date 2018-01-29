void sendRadiopacket(char* radiopacket, int outputLength) {

  if (debug) Serial.print("\nOutput: "); Serial.println(radiopacket);
  if (debug) Serial.print("\nSize: "); Serial.println(outputLength);
  if (debug) Serial.println("Sending...\t");
  rf95.send((uint8_t *)radiopacket, 200);
  rf95.waitPacketSent();
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (debug) Serial.print("Waiting for reply...");
  if (rf95.waitAvailableTimeout(RF_TIMEOUT))
  {
    if (rf95.recv(buf, &len) && debug) {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
    } else {
      if (debug) Serial.println("Receive failed");
    }
  }
  else {
    if (debug) Serial.println("Master node not connected!");
  }
}
