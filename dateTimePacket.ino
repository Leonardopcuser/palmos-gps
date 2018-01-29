// Takes in time data from GPS and converts it to KSK standardized string.
char* datePacket(int month, int day, int year) {
  // KSK standard: dash "/" delineates hours from minutes from seconds

  char monthStr[2];
  char dayStr[2];
  char yearStr[4];
  char dash[] = "/";

  char *dateStr = (char*) malloc(10);

  itoa(month, monthStr, 10);
  itoa(day, dayStr, 10);
  itoa(year, yearStr, 10);

  strcpy(dateStr, monthStr);
  strcat(dateStr, dash);
  strcat(dateStr, dayStr);
  strcat(dateStr, dash);
  strcat(dateStr, yearStr);

  return dateStr;

}

// Takes in time data from GPS and converts it to KSK standardized string.
char* timePacket(int hours, int minutes, int seconds) {
  // KSK standard: dash "/" delineates hours from minutes from seconds

  char hoursStr[2];
  char minutesStr[2];
  char secondsStr[2];
  char dash[] = "/";

  char *timeStr = (char*) malloc(10);

  itoa(hours, hoursStr, 10);
  itoa(minutes, minutesStr, 10);
  itoa(seconds, secondsStr, 10);

  strcpy(timeStr, hoursStr);
  strcat(timeStr, dash);
  strcat(timeStr, minutesStr);
  strcat(timeStr, dash);
  strcat(timeStr, secondsStr);

  return timeStr;

}
