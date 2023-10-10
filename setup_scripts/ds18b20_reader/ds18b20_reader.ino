/*
 * 
 *     UAH BoOSt — Arduino
 *        DS18B20 Reader
 *
 *   Written by Nick Perlaky 2023
 * 
 * Reads DS18B20 IDs and returns them in a
 * format suitable for the BoOSt script.
 * 
 */

#include <SparkFun_u-blox_SARA-R5_Arduino_Library.h>
#include "AssetTrackerPins.h"

#include <OneWire.h>

OneWire oneWire(PWM0);


SARA_R5 assetTracker(SARA_PWR);

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("DS18B20 Temperature Sensor Reader");
  Serial.println("Reads 1 sensor per second to avoid rapid spamming.");
  Serial.println("Prints in C array format for use in the STEMMNet script.");

  initializeAssetTrackerPins();
  pinMode(PWM0, OUTPUT);
  
  digitalWrite(PWM0, HIGH);

}

void loop() {
  
  OneWire ow(D0);

  uint8_t address[8];
  uint8_t count = 0;

  if (ow.search(address)) {
    
    do {
      count++;
    } while (ow.search(address));

    if (address[0] != 40) {return;}

    Serial.print("{");
    for (int i = 0; i < 7; i++) {Serial.print("0x");Serial.print(address[i], HEX);Serial.print(", ");}
    Serial.print("0x");Serial.print(address[7], HEX);
    Serial.println("},");
    
  }

  delay(1000);

}
