/*
This file contains various helper functions
*/
#include <SPI.h>
#include <Wire.h>

#include <Arduino.h>
#include <HardwareSerial.h>
#include <U8g2lib.h>
#include <SparkFunHTU21D.h>
#include <RtcDS3231.h>

#include <Time.h>

#include <TinyGPS++.h>

#include <SdFat.h>
#include <sdios.h>

#include <Preferences.h>

#ifdef ENABLE_OTA
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#endif

#ifdef USE_RFID
#include <MFRC522_I2C.h>		//RFID I²C
#endif

#include "gps.h"
#include "gui.h"
#include "helper_functions.h"
#include "ota_wifi.h"
#include "sd_card.h"


// Logs total distance to Storage every 10 additional meters
void calculate_total_dist();

void calculate_avg();

// Logs maximum values to internal Storage
void calculate_max();

// Called once at startup to load in previous values
void load_max_avg_values();