/*
This file contains various helper functions
*/
#ifndef __HELPER_FUNCT
#define __HELPER_FUNCT
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
#include "ota_wifi.h"
#include "sd_card.h"
#include "bicycle_computer_config.h"

extern portMUX_TYPE mux;
//PWM properties
extern const int freq;
extern const int ledChannel;
extern const int resolution;

//Variables
extern byte gui_selection;
extern float humid, temp;
extern int ldr_reading;
extern int pwm_value;
extern String sensor_comb;
extern String time_comb;
extern String date_comb;
extern bool time_sync_flag;
extern char daysOfTheWeek[7][12];
extern unsigned int seconds_running;
extern String time_running;

extern unsigned long loop_timing;
extern unsigned long loop_timing_2;
extern unsigned long loop_timing_3;

extern volatile bool button_data; 
extern volatile unsigned long button_timing;

extern unsigned long rfid_unlock[5];
extern struct gps_data_struct;

/*
Struct that contains statistics and other data to be displayed
*/
extern struct stat_display_data_struct;

extern struct gps_mapper_struct;

/*
SD Variables
*/
extern unsigned long last_log_sd ;
extern String filename;
extern char fileName[33];

extern bool SD_present;
extern unsigned long sd_log_count;


extern gps_data_struct gps_data;
extern gps_mapper_struct mapper;

extern stat_display_data_struct stats;

extern U8G2_ST7565_ERC12864_ALT_F_4W_HW_SPI u8g2;  // contrast improved version for ERC12864
extern HTU21D myHumidity;
extern RtcDS3231<TwoWire> Rtc;
extern TinyGPSPlus gps;
extern SdFat sd;
extern SdFile file;
extern Preferences preferences;
#ifdef USE_RFID
extern MFRC522 mfrc522;
#endif



// Logs total distance to Storage every 10 additional meters
void calculate_total_dist();

void calculate_avg();

// Logs maximum values to internal Storage
void calculate_max();

// Called once at startup to load in previous values
void load_max_avg_values();

void log_reset_times();
#endif