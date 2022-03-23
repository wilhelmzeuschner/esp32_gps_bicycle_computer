/*
This file contains all GPS related Code
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

/*
Time sync code
*/
void sync_rtc_with_gps();

/*
Determine whether or not GPS has a fix
*/
bool check_gps_fix();

/*
This function captures data and saves the path as vectors.
It is called whenever the GPS coordinates are updated.
*/
void gps_mapper();
void draw_gps_path();
void measure_distance_gps();
void update_gps_data();
void update_gps();
void display_gps_info();