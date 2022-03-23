/*
This file contains all GPS related Code
*/
#ifndef __GPS
#define __GPS
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


#include "gui.h"
#include "helper_functions.h"
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
#endif

