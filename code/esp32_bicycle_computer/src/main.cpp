/*
   Bicycle Computer
   Wilhelm Zeuschner
   16.03.2019
   Based on an ESP32

   u8g2 Font info: https://github.com/olikraus/u8g2/wiki/fntlistall
   GPS: http://arduiniana.org/libraries/tinygpsplus/
 */


#define DEBUG			//Print out debug messages
#define USE_REAL_ARRAY		//GPS Path mapper uses corrent array
#define PC_SERIAL	115200	//Serial Baud rate for connection between PC (USB to UART) and ESP32
//#define ENABLE_OTA			//Optional, for OTA Programming
#define MIN_NO_SAT		3	//Min number of satellites necessary for stats
#define UTC_ADJ			2	//Adjustment for your particular time zone
//#define USE_RFID

#define SD_SPEED		10		//SD Clock Speed (in MHz)
#define LCD_CONTRAST	75
#define GPS_BAUD		9600
#define REF_VOLTAGE		2.48	//TL431 Voltage (for calibration)
#define REF_ADJ			1.08	//Adjustment multiplier

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

#ifdef ENABLE_OTA
const char* ssid = "-";
const char* password = "-";
#endif


portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

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

//PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

//Variables
byte gui_selection = 0;
float humid = 0, temp = 0;
int ldr_reading = 0;
int pwm_value = 2550;
String sensor_comb;
String time_comb;
String date_comb;
bool time_sync_flag = 0;
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
unsigned int seconds_running = 0;
String time_running;

unsigned long loop_timing = 0;
unsigned long loop_timing_2 = 0;
unsigned long loop_timing_3 = 0;

volatile bool button_data = 0; 
volatile unsigned long button_timing = 0;

unsigned long rfid_unlock[5] = { 1891469948 , 3204651, 2771751097, 2247910361, 4067332910 };

struct gps_data_struct {
	double speed;
	double altitude;
	double course;
	unsigned int satellites;

	//Accumulated distance
	bool startup;		//False / 0 when program starts
	double travel_distance_km;
	double last_lat;
	double last_lng;
};

/*
Struct that contains statistics and other data to be displayed
*/
struct stat_display_data_struct {
	double max_speed;
	double max_alt;
	int max_sat;

	double avg_speed;
	double avg_temp;
	double avg_humid;

	double total_dist;
};

struct gps_mapper_struct
{
	double last_lat;					//Lat of last point
	double last_lng;					//Lng of last point
	double current_heading;				//Heading resulting from current Lat and Lng compared to old one
	double current_length;				//"travelled distance" (length of the vector)

	/*
	These two arrays contain individual vectors that together make up one larger one.
	If they are at least 10m long, they get combined into one and saved to the final array.
	Then the arrays are emptied and the process begins from the beginning.
	*/
	double ten_meter_x_array[100];		//Array to hold X coordinates until 10m are travelled
	double ten_meter_y_array[100];		//Array to hold Y coordinates until 10m are travelled
	unsigned int ten_length;			//Length of the vectors inside the ten_meter arrays
	unsigned int ten_counter;
	double temp_length;


	/*
	These arrays contain vectors that should all be longer than 10m.
	They make up the final segments of the visulization.
	*/
	double path_x_array[4000];
	double path_y_array[4000];
	unsigned int path_counter;
};

/*
SD Variables
*/
unsigned long last_log_sd = 0;
String filename;
char fileName[33];
#warning the size of fileName is restricted here!

bool SD_present = 0;
unsigned long sd_log_count = 0;


gps_data_struct gps_data;
gps_mapper_struct mapper;

stat_display_data_struct stats;

U8G2_ST7565_ERC12864_ALT_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ LCD_CS, /* dc=*/ LCD_DC, /* reset=*/ -1);  // contrast improved version for ERC12864
HTU21D myHumidity;
RtcDS3231<TwoWire> Rtc(Wire);
TinyGPSPlus gps;
SdFat sd;
SdFile file;
Preferences preferences;
#ifdef USE_RFID
MFRC522 mfrc522(0x28, rfid_reset);
#endif

void setup() {
	pinMode(ldr_pin, INPUT);
	pinMode(button_pin, INPUT_PULLUP);
	pinMode(ref_pin, INPUT);
	pinMode(battery_measure_pin, INPUT);
	pinMode(rtc_interrupt, INPUT);
#ifdef USE_RFID
	pinMode(rfid_reset, OUTPUT);
#endif

	attachInterrupt(digitalPinToInterrupt(button_pin), button_isr, FALLING);

	//Turn of BLE
	btStop();

	ledcSetup(ledChannel, freq, resolution);
	ledcAttachPin(backlight_pin, ledChannel);
	digitalWrite(backlight_pin, LOW);

	Serial2.begin(GPS_BAUD);
	Serial.begin(PC_SERIAL);

	Serial.println("\n\nBike Computer started.");
	Serial.printf("Compile date and time: %s, %s.\n\n", __DATE__, __TIME__);

	Rtc.Begin();
	myHumidity.begin();
	u8g2.begin();
	u8g2.setContrast(LCD_CONTRAST);

#ifdef USE_RFID
	mfrc522.PCD_Init();
	//Lockscreen
	rfid_lockscreen();
#endif

#ifdef ENABLE_OTA									//OTA is enabled
	if (digitalRead(button_pin) == 0) {				//Check if user button is held down at startup
		init_ota();									//Initialize OTA
		while (millis() - loop_timing < 30000)		//Cancel when longer than 20s
		{
			ArduinoOTA.handle();
		}
	}
	WiFi.mode(WIFI_OFF);
#endif

	//Start SD Logging
	SD_present = sd.begin(SD_CHIP_SELECT, SD_SCK_MHZ(SD_SPEED));
	if (!SD_present) {
		Serial.println("cardBegin failed!");
		//sd.initErrorHalt();
	}
	if (SD_present) {
		uint32_t cardSize = sd.card()->cardSize();
		Serial.print("Card Size: ");
		Serial.print(cardSize * float(0.000512), 0);
		Serial.println("MB");

		if (init_sd_logger()) {
			Serial.println("SD logging started successfully!");
		}
		else {
			Serial.println("Error while starting SD logging!");
		}
	}

	//Initialization
	load_max_avg_values();
	read_sensors();
	rtc_time();
	gui_selector();

	//Set initial values VERY IMPORTANT
	mapper.path_x_array[0] = 0;		mapper.path_y_array[0] = 0;
	mapper.path_x_array[1] = 1;		mapper.path_y_array[1] = 1;
	mapper.path_counter = 1;

	log_reset_times();
}


void loop() {
	//Run timed functions
	if (millis() - loop_timing_3 >= 1000) {		//Run this every 1000ms
		on_time_helper(false);
		loop_timing_3 = millis();
		//Call gui_selector(); just to update the display - not ideal here!
		gui_selector();

		if (!time_sync_flag) {		//Sync RTC to GPS time
			sync_rtc_with_gps();
		}
	}

	if (millis() - loop_timing >= 500) {		//Run this every 500ms
		read_sensors();
		rtc_time();

		if (SD_present) {
			digitalWrite(DISABLE_CHIP_SELECT, HIGH);
			digitalWrite(SD_CHIP_SELECT, LOW);
			SD_present = sd_log_data();
			digitalWrite(SD_CHIP_SELECT, HIGH);
			digitalWrite(DISABLE_CHIP_SELECT, LOW);
		}

		//Call gui_selector(); just to update the display
		gui_selector();

		loop_timing = millis();		
	}
	if (millis() - loop_timing_2 >= 10) {		//Run this every 10ms
		//Dim Screen Backlight
		ldr_dimmer();
		loop_timing_2 = millis();
	}

	//Update GPS Data
	update_gps();

	//Execute code if GPS has a fix
	if (check_gps_fix()) {
		calculate_max();
		calculate_total_dist();
	}

	//Check for button press
	if (button_data) {
		button_data = 0;

		//Advance GUI
		if (gui_selection < 2) {
			gui_selection++;
		}
		else {
			gui_selection = 0;
		}
		gui_selector();
	}
}

float read_battery_voltage() {
	int ref_val = analogRead(ref_pin);
	float battery_voltage = (4095.0 / ref_val) * (REF_VOLTAGE / REF_ADJ);
	//Serial.printf("Analog value: %i\nBattery Voltage: %f\n", ref_val, battery_voltage);
	return battery_voltage;
}

void read_sensors() {
	humid = myHumidity.readHumidity();
	temp = myHumidity.readTemperature();


	/*Serial.print(" Temperature:");
	Serial.print(temp, 1);
	Serial.print("C");
	Serial.print(" Humidity:");
	Serial.print(humid, 1);
	Serial.println("%");*/

	byte decimal_places = 2;
	if (temp >= 10 || temp <= -10) {
		decimal_places = 1;
	}
	if (temp >= 100 || temp <= -100) {
		decimal_places = 0;
	}

	sensor_comb = String(temp, decimal_places) + "C " + String(humid, 0) + "%";
}

void rtc_time() {
	//RTC Measurement

	RtcDateTime now = Rtc.GetDateTime();

	String day_padding = zero_padder(String(now.Day()));  //Add Padding to Days
	/*Serial.print(day_padding);
	Serial.print('.');*/
	String month_padding = zero_padder(String(now.Month()));  //Add Padding to Months
	/*Serial.print(month_padding);
	Serial.print('.');
	Serial.print(now.year(), DEC);
	Serial.print(" (");
	Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
	Serial.print(") ");*/

	String hour_padding = zero_padder(String(now.Hour()));  //Add Padding to Hours
	String min_padding = zero_padder(String(now.Minute())); //Add Padding to Minutes	
	String sec_padding = zero_padder(String(now.Second())); //Add Padding to Seconds	

	/*Serial.print(hour_padding);
	Serial.print(':');
	Serial.print(min_padding);
	Serial.print(':');
	Serial.print(sec_padding);
	Serial.println();*/

	time_comb = String(hour_padding) + ":" + String(min_padding) + ":" + String(sec_padding);
	date_comb = String(day_padding) + "." + String(month_padding) + "." + String(now.Year());
}

//This function calculates how long the device has been running
//and creates an appropriate String that can be displayed
void on_time_helper(bool create_output) {
	if (!create_output) seconds_running++;		//Just log time
	else
	{
		time_running = zero_padder(String(seconds_running / 3600)) + ":" + zero_padder(String(seconds_running / 60)) + ":" + zero_padder(String(seconds_running - (seconds_running / 60) * 60));
	}
	/*Serial.println(time_running);
	Serial.printf("on-time: %u\n", seconds_running);*/
}

//Zero Padding function
String zero_padder(String z_p) {
	String zero_padded = z_p;   //Variable to hold result
	float current = z_p.toFloat();  //Integer for comparison
	if (current < 10) {
		zero_padded = "0" + z_p;
	}
	return zero_padded;
}

//Dim the backlight according to the LDR reading and apply a low pass filter
void ldr_dimmer() {
	ldr_reading = analogRead(ldr_pin);
	
	if (ldr_reading < 500) {		//Very dark
		if (pwm_value > 100)  pwm_value = pwm_value * 0.99;
	}
	else if (ldr_reading < 1500) {	//Dark
		if (pwm_value > 800)  pwm_value = pwm_value * 0.99;
		else if (pwm_value < 800) pwm_value = pwm_value * 1.02;
	}
	else if (ldr_reading < 2200) {	//Slightly dark
		if (pwm_value > 1800)  pwm_value = pwm_value * 0.99;
		else if (pwm_value < 1800) pwm_value = pwm_value * 1.02;
	}
	
	ledcWrite(ledChannel, pwm_value / 10);

	//It's so bright that no backlight is necessary
	if (ldr_reading >= 2200) {
		ledcWrite(ledChannel, 0);
		pwm_value = 100;
	}
}

/*IRAM_ATTR*/
void IRAM_ATTR button_isr() {
	if (millis() - button_timing >= 300) {
		button_timing = millis();
		button_data = 1;
	}
}

#ifdef USE_RFID
void rfid_lockscreen() {
	bool rfid_lock = 1;
	while (rfid_lock) {		
		//Run timed functions in locked mode
		if (millis() - loop_timing >= 1000) {		//Run this every 1000ms
			read_sensors();
			rtc_time();
			u8g2.clearBuffer();
			u8g2.setFont(u8g2_font_crox4hb_tf);
			u8g2.setCursor(5, 25);
			u8g2.print("Bicycle");
			u8g2.setCursor(5, 41);
			u8g2.print("Computer");
			u8g2.setCursor(5, 59);
			u8g2.print("is locked");
			//Display Time and Date
			u8g2.setFont(u8g2_font_t0_12_tr);
			u8g2.setCursor(81, 8);
			u8g2.print(time_comb);
			u8g2.drawHLine(81, 9, 47);

			u8g2.setFont(u8g2_font_5x7_mr);
			u8g2.setCursor(79, 17);
			u8g2.print(date_comb);
			u8g2.drawHLine(80, 18, 48);
			//Display Sensor Readings
			u8g2.setCursor(81, 26);
			u8g2.print(sensor_comb);
			u8g2.drawHLine(81, 27, 47);
			loop_timing = millis();
		}
		if (millis() - loop_timing_2 >= 10) {		//Run this every 10ms
			//Dim Screen Backlight
			ldr_dimmer();
			loop_timing_2 = millis();
		}
		u8g2.sendBuffer();

		//Check for new RFID card
		if (mfrc522.PICC_IsNewCardPresent()) {
			unsigned long uid = getID();
			if (uid != -1) {
				#ifdef DEBUG
				Serial.print("Card detected, UID: "); Serial.println(uid);
				#endif // DEBUG
				for (int i = 0; i < sizeof(rfid_unlock); i++) {
					if (uid == rfid_unlock[i]) {
						#ifdef DEBUG
						Serial.println("Access granted!");
						#endif // DEBUG
						digitalWrite(rfid_reset, LOW);
						rfid_lock = 0;
					}
				}
			}
		}
	}
}

//Read RFID ID
unsigned long getID() {
	if (!mfrc522.PICC_ReadCardSerial()) { //Since a PICC placed get Serial and continue
		return -1;
	}
	unsigned long hex_num;
	hex_num = mfrc522.uid.uidByte[0] << 24;
	hex_num += mfrc522.uid.uidByte[1] << 16;
	hex_num += mfrc522.uid.uidByte[2] << 8;
	hex_num += mfrc522.uid.uidByte[3];
	mfrc522.PICC_HaltA(); // Stop reading
	return hex_num;
}
#endif
