#ifndef __COMPUTER_CONFIG
#define __COMPUTER_CONFIG

#include <Arduino.h>

// Optional features
//#define 	ENABLE_OTA
//#define 	USE_RFID
//#define	ENABLE_TRIP_VISUALIZER
//#define 	ENABLE_STATS_DISPLAY



#define DEBUG			//Print out debug messages
#define USE_REAL_ARRAY		//GPS Path mapper uses corrent array
#define PC_SERIAL	115200	//Serial Baud rate for connection between PC (USB to UART) and ESP32
#define MIN_NO_SAT		3	//Min number of satellites necessary for stats
#define UTC_ADJ			2	//Adjustment for your particular time zone


#define SD_SPEED		10		//SD Clock Speed (in MHz)
#define LCD_CONTRAST	75
#define GPS_BAUD		9600
#define REF_VOLTAGE		2.48	//TL431 Voltage (for calibration)
#define REF_ADJ			1.08	//Adjustment multiplier


#ifdef ENABLE_OTA
const char* ssid = "-";
const char* password = "-";
#endif

//Pins
const byte backlight_pin = 15;
const byte ldr_pin = 32;
const byte button_pin = 25;
const byte ref_pin = 26;
const byte battery_measure_pin = 33;
const byte rtc_interrupt = 27;
const byte LCD_DC = 4;
const byte LCD_CS = 5;
const byte SD_CHIP_SELECT = 14;
const byte DISABLE_CHIP_SELECT = LCD_CS;
#ifdef USE_RFID
const byte rfid_reset = 12;
#endif

#define GPS_SPEED_DISPLAY_THRESH	2	 //If speed is below GPS_SPEED_DISPLAY_THRESH, display 0 and don't log this speed / calculate the average based on it

#define LDR_DARK_VAL				700		//Higher means: turns on when it's even darker
#define LDR_HYSTERESIS				200
#define DARK_PWM_VAL				10

#endif