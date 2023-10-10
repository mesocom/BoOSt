/*
 * 
 *     UAH BoOSt - Arduino
 *
 *    Written by Nick Perlaky
 * 
 */


#include <SparkFun_u-blox_SARA-R5_Arduino_Library.h>
#include "AssetTrackerPins.h"
#include <RTClib.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_ADS1X15.h>
#include <IPAddress.h>
#include <SPI.h>
#include <SdFat.h>
#include "Adafruit_PM25AQI.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"


/************CONFIG**************/
#define SERIAL_DEBUG 1
#define STATION_ID "BOOST06"
#define MAX_TEMP_SENSORS 1
DeviceAddress dsAddresses[MAX_TEMP_SENSORS] = {
{0x28, 0x2C, 0x97, 0xE, 0xF, 0x0, 0x0, 0x79}
};
/************CONFIG**************/


// Hardware constants
#define LED_PIN LED_BUILTIN
#define POWER_MOSFET_PIN PWM0
#define DS_PIN D0
#define WSPD_PIN D1
#define ADS_PV 0
#define ADS_BATT 1
#define ADS_VOLT 2
#define ADS_WDIR 3

// Data logging constants
#define LOG_INTERVAL_SECONDS 10
#define POINT_LENGTH_BYTES 96 // 95 bytes + ; + \0

// Status constants
#define STATUS_BLINK 0
#define STATUS_SUCCESS 1
#define STATUS_ERROR 2

// Clock
RTC_DS3231 rtc;
DateTime currentTime;

// SARA R5
#define saraSerial Serial1
SARA_R5 assetTracker(SARA_PWR);
RTC_DATA_ATTR boolean radioWasInitiallyTurnedOff = false;


// OneWire
OneWire oneWire(DS_PIN);
DallasTemperature dsSensors(&oneWire);


// ADS
Adafruit_ADS1115 ads0;


// Davis
volatile unsigned long wspd_counter = 0;
volatile unsigned long ContactBounceTime;
void IRAM_ATTR Davis_ISR() {
  if ((millis() - ContactBounceTime) > 15 ) { // debounce the switch contact.
    wspd_counter++;
    ContactBounceTime = millis();
  }
}


// AQI, SHT31, BMP388
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
Adafruit_BME680 bme;


// SD
SdFat sd;
SdFile logFile;
SdFile uploadFile;
boolean sdCardFunctioning = false;
boolean sdInUse = false;


void setup() {

  /* Start serial */
  if (SERIAL_DEBUG) {Serial.begin(115200);delay(100);Serial.println("Booted...");}


  /* Hardware setup */
  initializeAssetTrackerPins();
  assetTracker.invertPowerPin(true);
  pinMode(LED_PIN, OUTPUT);
  pinMode(POWER_MOSFET_PIN, OUTPUT);
  pinMode(DS_PIN, INPUT);
  pinMode(WSPD_PIN, INPUT_PULLUP);
  

  // Turn on sensors
  digitalWrite(POWER_MOSFET_PIN, HIGH);
  delay(3000);
  if (SERIAL_DEBUG) Serial.println(F("Sensor power on..."));


  /* RTC Initialization */
  while (!rtc.begin()) {
    if (SERIAL_DEBUG) Serial.println(F("RTC Error..."));
    systemStatus(STATUS_ERROR);
  }
  

  /* SD Card Check */
  disableMicroSDPower();
  enableMicroSDPower();
  SD_and_IMU_SPI.begin();
  while (sd.begin(MICROSD_CS, SD_SCK_MHZ(24)) == false) {
    if (SERIAL_DEBUG) Serial.println(F("SD Error..."));
    systemStatus(STATUS_ERROR);
  }


  /* DS18B20 Initialization */
  dsSensors.begin();
  dsSensors.setResolution(12);


  /* ADS1115 Initialization */
  while ( !ads0.begin(0x48) ) {
    if (SERIAL_DEBUG) Serial.println(F("Failed to initialize ADS1115 0"));
    systemStatus(STATUS_ERROR);
  }
  ads0.setGain(GAIN_TWOTHIRDS);

  /* AQI Initialization */
  while (!aqi.begin_I2C()) {
    if (SERIAL_DEBUG) Serial.println(F("Failed to initialize PM 2.5"));
    systemStatus(STATUS_ERROR);
  }

  /* BMP688 Initialization */
  while (!bme.begin()) {
    if (SERIAL_DEBUG) Serial.println(F("Failed to initialize BME688"));
    systemStatus(STATUS_ERROR);
  }
  
  /* Shut off cell radio */
  if (!radioWasInitiallyTurnedOff) {
    assetTracker.hardPowerOff();
    radioWasInitiallyTurnedOff = true;
  }


  if (SERIAL_DEBUG) Serial.println(F("Setup Complete. Starting log procedure..."));

}


/*--------------------------------------------------*/
/*----------------------- Loop ---------------------*/
/*--------------------------------------------------*/


void loop() {

  // Wait for log time
  while (1) {
    currentTime = rtc.now();
    if (currentTime.unixtime() % LOG_INTERVAL_SECONDS == 0) break;
    delay(100);
  }

  if (SERIAL_DEBUG) Serial.println(F("Logging..."));

  // Collect data point
  char d[POINT_LENGTH_BYTES];
  logDataPoint(currentTime, d);

  if (SERIAL_DEBUG) Serial.println(F("Logging complete"));

}



/*--------------------------------------------------*/
/*-------------------- Functions -------------------*/
/*--------------------------------------------------*/



/* Log data point */
void logDataPoint(DateTime currentTime, char *d) {

  systemStatus(STATUS_BLINK);

  uint32_t currentTimeStamp = currentTime.unixtime();


  // Measure 3s wind speed
  if (SERIAL_DEBUG) Serial.println(F("Measuring wind speed"));
  uint32_t startTime = millis();
  int interruptNumber = digitalPinToInterrupt(WSPD_PIN);
  attachInterrupt(interruptNumber, Davis_ISR, FALLING);
  while (millis() - startTime < 3000) {delay(10);}
  detachInterrupt(interruptNumber);
  if (SERIAL_DEBUG) Serial.println(F("Measured Wind Speed"));
  
  // Character array position
  int p = 0;


  // DS18B20 Temperature
  dsSensors.requestTemperatures();
  for (int i = 0; i < MAX_TEMP_SENSORS; i++) {
    float tempC = dsSensors.getTempC(dsAddresses[i]);
    if (tempC != DEVICE_DISCONNECTED_C) {
      dtostrf(tempC, 6, 5, d + (7*i));
    } else {
      sprintf(d + (7*i), "%s", "-99.99");
    }
      d[6 + (7*i)] = ',';
      p = 7*(i+1);
  }
  if (SERIAL_DEBUG) Serial.println(F("Measured skin temperature"));

  // Moistures / voltages
  float voltages[4];
  for (int i = 0; i < 4; i++) {
    voltages[i] = 1000*ads0.computeVolts(ads0.readADC_SingleEnded(i));
  }
  for (int i = 0; i < 4; i++) {
    dtostrf(
      voltages[i],
      7, 5, d + p);
    p += 7;
    d[p] = ','; p+=1;
  }
  if (SERIAL_DEBUG) Serial.println(F("Measured voltages"));


  // Wind Speed v = p * 2.25/t * .44704 for m/s --> v = p * 0.33528
  dtostrf(
      float(wspd_counter) * 0.33528,
      4, 5, d + p);
  p+=5;
  d[p] = ','; p+=1;
  wspd_counter = 0;
  if (SERIAL_DEBUG) Serial.println(F("Measured wspd"));


  // Wind Direction
  dtostrf(
      (voltages[ADS_WDIR] / voltages[ADS_VOLT]) * 360.0,
      5, 5, d + p);
  p+=5;
  d[p] = ','; p+=1;
  if (SERIAL_DEBUG) Serial.println(F("Measured wdir"));


  // PM2.5
  PM25_AQI_Data pmData;
  int pm[3];
  if (aqi.read(&pmData)) {
    pm[0] = pmData.pm10_env;
    pm[1] = pmData.pm25_env;
    pm[2] = pmData.pm100_env;
  } else {
    pm[0] = 0;
    pm[1] = 0;
    pm[2] = 0;
  }
  sprintf(d+p, "%03d", pm[0]); p+=3; d[p] = ','; p+=1;
  sprintf(d+p, "%03d", pm[1]); p+=3; d[p] = ','; p+=1;
  sprintf(d+p, "%03d", pm[2]); p+=3; d[p] = ','; p+=1;
  if (SERIAL_DEBUG) Serial.println(F("Measured PM 2.5"));


  // BME688
  float t, p_pa, rh;
  
  if (bme.temperature > -40.0) t = bme.readTemperature();
  else t = -99.99;
  dtostrf(
      t,
      5, 5, d + p);
  d[p+6] = ',';
  p += 7;

  if (bme.humidity > 0.0) rh = bme.readHumidity();
  else rh = -99.99;
  dtostrf(
      rh,
      4, 5, d + p);
  d[p+5] = ',';
  p += 6;

  if (bme.readPressure() > 0.0) p_pa = bme.readPressure();
  else p_pa = -99.99;
  if (p_pa < 100000.0) {
    d[p] = '0';
    sprintf(d + p + 1, "%lu", int(p_pa));
  } else {
    sprintf(d + p, "%lu", int(p_pa));
  }
  d[p+6] = ',';
  p += 7;

  if (SERIAL_DEBUG) Serial.println(F("Measured BME688"));


  // Insert time
  sprintf(d + p, "%lu", currentTimeStamp); p += 10;
  d[p] = '\n'; p+=1;
  d[p] = 0;
  if (SERIAL_DEBUG) {Serial.print(F("Log time: "));Serial.println(currentTimeStamp);}


  // Log to SD card - new file each day to prevent loss - and upload buffer
  Serial.print(F("Saving to SD..."));

  char path[26];
  sprintf(path, "%s_%04d-%02d-%02d-%02d.txt", STATION_ID, currentTime.year(), currentTime.month(), currentTime.day(), currentTime.hour());
  if (SERIAL_DEBUG) Serial.println(F("Starting save process..."));
  appendLogFile(path, d);

  // Print
  if (SERIAL_DEBUG) {Serial.print("Logged: ");Serial.println(d);}

  systemStatus(STATUS_SUCCESS);
  
}



/* System status indicator */
void systemStatus(int sts) {

  if (SERIAL_DEBUG) {Serial.print("Status: ");Serial.println(sts);}

  switch (sts) {
    case 0:
      digitalWrite(LED_PIN, HIGH);delay(50);digitalWrite(LED_PIN, LOW);
      break;
    case 1:
      digitalWrite(LED_PIN, HIGH);delay(50);digitalWrite(LED_PIN, LOW);delay(100);
      digitalWrite(LED_PIN, HIGH);delay(50);digitalWrite(LED_PIN, LOW);delay(100);
      digitalWrite(LED_PIN, HIGH);delay(50);digitalWrite(LED_PIN, LOW);
      break;
    case 2:
      digitalWrite(LED_PIN, HIGH);delay(500);digitalWrite(LED_PIN, LOW);delay(500);
    default:
      break;
  }
  
}



void appendLogFile(char *path, char *d) {

  if (SERIAL_DEBUG) Serial.println(F("Saving ..."));
  
  // Open file
  if (!logFile.open(path, FILE_WRITE)) {
    if (SERIAL_DEBUG) Serial.println(F("Failed to open file"));
    systemStatus(STATUS_ERROR);
    return;
  } else if (SERIAL_DEBUG) Serial.println(F("Opened log file"));

  // Append to file
  if (!logFile.print(d)) {
    if (SERIAL_DEBUG) Serial.println(F("Failed write data to SD card"));
    systemStatus(STATUS_ERROR);
  } else if (SERIAL_DEBUG) Serial.println(F("Wrote data"));

  // Check for failure
  if (!logFile.close()) {
    if (SERIAL_DEBUG) Serial.println(F("Failed to close file"));
    systemStatus(STATUS_ERROR);
  } else if (SERIAL_DEBUG) Serial.println(F("Closed log file"));

  if (SERIAL_DEBUG) {Serial.print(F("Wrote data: "));Serial.println(d);}
  
}
