/*
   Bicycle Computer
   Wilhelm Zeuschner
   16.03.2019
   Based on an ESP32

   u8g2 Font info: https://github.com/olikraus/u8g2/wiki/fntlistall
   GPS: http://arduiniana.org/libraries/tinygpsplus/
 */

#include <SPI.h>
#include <Wire.h>

#include <Arduino.h>
#include <HardwareSerial.h>
#include <U8g2lib.h>
#include <HTU2xD_SHT2x_Si70xx.h>
#include <RtcDS3231.h>

#include <Time.h>

#include <TinyGPS++.h>

#include <SdFat.h>
#include <sdios.h>

#ifdef ENABLE_PREFERENCES
#include <Preferences.h>
#endif

#include "bicycle_computer_config.h"

#ifdef ENABLE_OTA
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#endif

#ifdef USE_RFID
#include <MFRC522_I2C.h> //RFID I²C
#endif

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

// Variables
byte gui_selection = 0;
float humid = 0, temp = 0;
int ldr_reading = 0;
int pwm_value = 2550;
String sensor_comb;
String time_comb;
String date_comb;
bool time_sync_flag = 0;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
unsigned int seconds_running = 0;
String time_running;

unsigned long loop_timing = 0;
unsigned long loop_timing_2 = 0;
unsigned long loop_timing_3 = 0;

volatile bool button_data = 0;
volatile unsigned long button_timing = 0;

#ifdef USE_RFID
unsigned long rfid_unlock[5] = {1891469948, 3204651, 2771751097, 2247910361, 4067332910};
#endif

struct gps_data_struct
{
	double speed;
	double altitude;
	double course;
	unsigned int satellites;

	// Accumulated distance
	bool startup; // False / 0 when program starts
	double travel_distance_km;
	double last_lat;
	double last_lng;
};

/*
Struct that contains statistics and other data to be displayed
*/
struct stat_display_data_struct
{
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
	double last_lat;		// Lat of last point
	double last_lng;		// Lng of last point
	double current_heading; // Heading resulting from current Lat and Lng compared to old one
	double current_length;	//"travelled distance" (length of the vector)

	/*
	These two arrays contain individual vectors that together make up one larger one.
	If they are at least 10m long, they get combined into one and saved to the final array.
	Then the arrays are emptied and the process begins from the beginning.
	*/
	double ten_meter_x_array[100]; // Array to hold X coordinates until 10m are travelled
	double ten_meter_y_array[100]; // Array to hold Y coordinates until 10m are travelled
	unsigned int ten_length;	   // Length of the vectors inside the ten_meter arrays
	unsigned int ten_counter;
	double temp_length;

	/*
	These arrays contain vectors that should all be longer than 10m.
	They make up the final segments of the visualization.
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

U8G2_ST7565_ERC12864_ALT_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/LCD_CS, /* dc=*/LCD_DC, /* reset=*/-1); // contrast improved version for ERC12864
// HTU21D myHumidity; //deprecated
HTU2xD_SHT2x_SI70xx ht2x(HTU2xD_SENSOR, HUMD_12BIT_TEMP_14BIT); // sensor type, resolution
RtcDS3231<TwoWire> Rtc(Wire);
TinyGPSPlus gps;
SdFat sd;
SdFile file;
#ifdef ENABLE_PREFERENCES
Preferences preferences;
#endif

#ifdef USE_RFID
MFRC522 mfrc522(0x28, rfid_reset);
#endif

// Function declarations
void rfid_lockscreen();
void IRAM_ATTR button_isr();
void ldr_dimmer();
String zero_padder(String z_p);
void on_time_helper(bool create_output);
void rtc_time();
void read_sensors();
float read_battery_voltage();

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
#ifdef ENABLE_TRIP_VISUALIZER
void gps_mapper();
void draw_gps_path();
#endif
void measure_distance_gps();
void update_gps_data();
void update_gps();
void display_gps_info();

// This function calls the apppropriate GUI drawing function
void gui_selector();
#ifdef ENABLE_STATS_DISPLAY
void draw_stats();
#endif
void update_display();

#ifdef ENABLE_PREFERENCES
// Logs total distance to Storage every 10 additional meters
void calculate_total_dist();

void calculate_avg();

// Logs maximum values to internal Storage
void calculate_max();
// Called once at startup to load in previous values
void load_max_avg_values();
void log_reset_times();
#endif

#ifdef ENABLE_OTA
void init_ota();
#endif // ENABLE_OTA

// Saves data to SD
bool sd_log_data();

bool init_sd_logger();
void set_filename();
bool SD_set_timestamps();
String time_comb_helper(bool include_day, bool include_date, bool include_time, bool time_separator);

void setup()
{
	pinMode(ldr_pin, INPUT);
	pinMode(button_pin, INPUT_PULLUP);
	pinMode(ref_pin, INPUT);
	pinMode(battery_measure_pin, INPUT);
	pinMode(rtc_interrupt, INPUT);
#ifdef USE_RFID
	pinMode(rfid_reset, OUTPUT);
#endif

	attachInterrupt(digitalPinToInterrupt(button_pin), button_isr, FALLING);

	// Turn of BLE
	btStop();

	ledcSetup(ledChannel, freq, resolution);
	ledcAttachPin(backlight_pin, ledChannel);
	digitalWrite(backlight_pin, LOW);

	Serial2.begin(GPS_BAUD);
	Serial.begin(PC_SERIAL);

	Serial.println("\n\nBike Computer started.");
	Serial.printf("Compile date and time: %s, %s.\n\n", __DATE__, __TIME__);

	Rtc.Begin();
	ht2x.begin();
	u8g2.begin();
	u8g2.setContrast(LCD_CONTRAST);

#ifdef USE_RFID
	mfrc522.PCD_Init();
	// Lockscreen
	rfid_lockscreen();
#endif

#ifdef ENABLE_OTA // OTA is enabled
	if (digitalRead(button_pin) == 0)
	{										   // Check if user button is held down at startup
		init_ota();							   // Initialize OTA
		while (millis() - loop_timing < 30000) // Cancel when longer than 20s
		{
			ArduinoOTA.handle();
		}
	}
	WiFi.mode(WIFI_OFF);
#endif

	// Start SD Logging
	SD_present = sd.begin(SD_CHIP_SELECT, SD_SCK_MHZ(SD_SPEED));
	if (!SD_present)
	{
		Serial.println("cardBegin failed!");
		// sd.initErrorHalt();
	}
	if (SD_present)
	{
		uint32_t cardSize = sd.card()->cardSize();
		Serial.print("Card Size: ");
		Serial.print(cardSize * float(0.000512), 0);
		Serial.println("MB");

		if (init_sd_logger())
		{
			Serial.println("SD logging started successfully!");
		}
		else
		{
			Serial.println("Error while starting SD logging!");
		}
	}

// Initialization
#ifdef ENABLE_PREFERENCES
	load_max_avg_values();
#endif

	read_sensors();
	rtc_time();
	gui_selector();

	// Set initial values VERY IMPORTANT
	mapper.path_x_array[0] = 0;
	mapper.path_y_array[0] = 0;
	mapper.path_x_array[1] = 1;
	mapper.path_y_array[1] = 1;
	mapper.path_counter = 1;

#ifdef ENABLE_PREFERENCES
	log_reset_times();
#endif
	Serial.println("Setup done");
}

void loop()
{
	// Run timed functions
	if (millis() - loop_timing_3 >= 1000)
	{ // Run this every 1000ms
		on_time_helper(false);
		loop_timing_3 = millis();
		// Call gui_selector(); just to update the display - not ideal here!
		gui_selector();

		if (!time_sync_flag)
		{ // Sync RTC to GPS time
			sync_rtc_with_gps();
		}
	}

	if (millis() - loop_timing >= 500)
	{ // Run this every 500ms
		read_sensors();
		rtc_time();

		if (SD_present)
		{
			digitalWrite(DISABLE_CHIP_SELECT, HIGH);
			digitalWrite(SD_CHIP_SELECT, LOW);
			SD_present = sd_log_data();
			digitalWrite(SD_CHIP_SELECT, HIGH);
			digitalWrite(DISABLE_CHIP_SELECT, LOW);
		}

		// Call gui_selector(); just to update the display
		gui_selector();

		loop_timing = millis();
	}

	if (millis() - loop_timing_2 >= 10)
	{ // Run this every 10ms
		// Dim Screen Backlight
		ldr_dimmer();
		loop_timing_2 = millis();
	}

	// Update GPS Data
	update_gps();

#ifdef ENABLE_PREFERENCES
	// Execute code if GPS has a fix
	if (check_gps_fix())
	{

		calculate_max();
		calculate_total_dist();
	}
#endif

	// Check for button press
	if (button_data)
	{
		button_data = 0;

		// Advance GUI
		if (gui_selection < 2)
		{
			gui_selection++;
		}
		else
		{
			gui_selection = 0;
		}

// Account for the fact that some screens might be unselected in the config
#ifndef ENABLE_TRIP_VISUALIZER
		if (gui_selection == 1)
			gui_selection++;
#endif
#ifndef ENABLE_STATS_DISPLAY
		if (gui_selection == 2)
			gui_selection = 0;
#endif

		gui_selector();
	}
}

float read_battery_voltage()
{
	int ref_val = analogRead(ref_pin);
	float battery_voltage = (4095.0 / ref_val) * (REF_VOLTAGE / REF_ADJ);
	// Serial.printf("Analog value: %i\nBattery Voltage: %f\n", ref_val, battery_voltage);
	return battery_voltage;
}

void read_sensors()
{
	humid = ht2x.readHumidity();
	temp = ht2x.readTemperature();

	/*Serial.print(" Temperature:");
	Serial.print(temp, 1);
	Serial.print("C");
	Serial.print(" Humidity:");
	Serial.print(humid, 1);
	Serial.println("%");*/

	int decimal_places = 2;
	if (temp >= 10 || temp <= -10)
	{
		decimal_places = 1;
	}
	if (temp >= 100 || temp <= -100)
	{
		decimal_places = 0;
	}

	sensor_comb = String(temp, decimal_places) + "C " + String(humid, 0) + "%";
}

void rtc_time()
{
	// RTC Measurement

	RtcDateTime now = Rtc.GetDateTime();

	String day_padding = zero_padder(String(now.Day())); // Add Padding to Days
	/*Serial.print(day_padding);
	Serial.print('.');*/
	String month_padding = zero_padder(String(now.Month())); // Add Padding to Months
	/*Serial.print(month_padding);
	Serial.print('.');
	Serial.print(now.year(), DEC);
	Serial.print(" (");
	Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
	Serial.print(") ");*/

	String hour_padding = zero_padder(String(now.Hour()));	// Add Padding to Hours
	String min_padding = zero_padder(String(now.Minute())); // Add Padding to Minutes
	String sec_padding = zero_padder(String(now.Second())); // Add Padding to Seconds

	/*Serial.print(hour_padding);
	Serial.print(':');
	Serial.print(min_padding);
	Serial.print(':');
	Serial.print(sec_padding);
	Serial.println();*/

	time_comb = String(hour_padding) + ":" + String(min_padding) + ":" + String(sec_padding);
	date_comb = String(day_padding) + "." + String(month_padding) + "." + String(now.Year());
}

// This function calculates how long the device has been running
// and creates an appropriate String that can be displayed
void on_time_helper(bool create_output)
{
	if (!create_output)
		seconds_running++; // Just log time
	else
	{
		time_running = zero_padder(String(seconds_running / 3600)) + ":" + zero_padder(String(seconds_running / 60)) + ":" + zero_padder(String(seconds_running - (seconds_running / 60) * 60));
	}
	/*Serial.println(time_running);
	Serial.printf("on-time: %u\n", seconds_running);*/
}

// Zero Padding function
String zero_padder(String z_p)
{
	String zero_padded = z_p;	   // Variable to hold result
	float current = z_p.toFloat(); // Integer for comparison
	if (current < 10)
	{
		zero_padded = "0" + z_p;
	}
	return zero_padded;
}

// Dim the backlight according to the LDR reading and apply a low pass filter
void ldr_dimmer()
{
	ldr_reading = analogRead(ldr_pin);

	if (ldr_reading < 500)
	{ // Very dark
		if (pwm_value > 100)
			pwm_value = pwm_value * 0.99;
	}
	else if (ldr_reading < 1500)
	{ // Dark
		if (pwm_value > 800)
			pwm_value = pwm_value * 0.99;
		else if (pwm_value < 800)
			pwm_value = pwm_value * 1.02;
	}
	else if (ldr_reading < 2200)
	{ // Slightly dark
		if (pwm_value > 1800)
			pwm_value = pwm_value * 0.99;
		else if (pwm_value < 1800)
			pwm_value = pwm_value * 1.02;
	}

	ledcWrite(ledChannel, pwm_value / 10);

	// It's so bright that no backlight is necessary
	if (ldr_reading >= 2200)
	{
		ledcWrite(ledChannel, 0);
		pwm_value = 100;
	}
}

/*IRAM_ATTR*/
void IRAM_ATTR button_isr()
{
	if (millis() - button_timing >= 300)
	{
		button_timing = millis();
		button_data = 1;
	}
}

#ifdef USE_RFID
void rfid_lockscreen()
{
	bool rfid_lock = 1;
	while (rfid_lock)
	{
		// Run timed functions in locked mode
		if (millis() - loop_timing >= 1000)
		{ // Run this every 1000ms
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
			// Display Time and Date
			u8g2.setFont(u8g2_font_t0_12_tr);
			u8g2.setCursor(81, 8);
			u8g2.print(time_comb);
			u8g2.drawHLine(81, 9, 47);

			u8g2.setFont(u8g2_font_5x7_mr);
			u8g2.setCursor(79, 17);
			u8g2.print(date_comb);
			u8g2.drawHLine(80, 18, 48);
			// Display Sensor Readings
			u8g2.setCursor(81, 26);
			u8g2.print(sensor_comb);
			u8g2.drawHLine(81, 27, 47);
			loop_timing = millis();
		}
		if (millis() - loop_timing_2 >= 10)
		{ // Run this every 10ms
			// Dim Screen Backlight
			ldr_dimmer();
			loop_timing_2 = millis();
		}
		u8g2.sendBuffer();

		// Check for new RFID card
		if (mfrc522.PICC_IsNewCardPresent())
		{
			unsigned long uid = getID();
			if (uid != -1)
			{
#ifdef DEBUG
				Serial.print("Card detected, UID: ");
				Serial.println(uid);
#endif // DEBUG
				for (int i = 0; i < sizeof(rfid_unlock); i++)
				{
					if (uid == rfid_unlock[i])
					{
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

// Read RFID ID
unsigned long getID()
{
	if (!mfrc522.PICC_ReadCardSerial())
	{ // Since a PICC placed get Serial and continue
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

/////////////////////////////////////////////////////////////////////////
// GPS
/*
Time sync code
*/
void sync_rtc_with_gps()
{
	if (gps.time.isValid() && gps.date.isValid() && gps.time.isUpdated() && gps.date.isUpdated() && gps.satellites.value() > 3 && gps.time.hour() != 24)
	{
		Rtc.SetDateTime(RtcDateTime(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour() + UTC_ADJ, gps.time.minute(), gps.time.second()));
		Serial.println("Time synced");
		time_sync_flag = 1;
	}
}

/*
Determine whether or not GPS has a fix
*/
bool check_gps_fix()
{
	bool fix = 0;

	if (gps.satellites.value() >= MIN_NO_SAT)
	{			 // If number is not 1 (which means no fix)
		fix = 1; // a fix must be established, min no. of satellites must be reached
		Serial.println("has fix");
	}

	return fix;
}

/*
This function captures data and saves the path as vectors.
It is called whenever the GPS coordinates are updated.
*/
#ifdef ENABLE_TRIP_VISUALIZER
void gps_mapper()
{
	int size_of_meter_array = sizeof(mapper.ten_meter_x_array) / sizeof(double);
	int size_of_path_array = sizeof(mapper.path_x_array) / sizeof(double);

	// Calculate new values for current dataset
	mapper.current_length = gps.distanceBetween(mapper.last_lat, mapper.last_lng, gps.location.lat(), gps.location.lng());
	mapper.current_heading = gps.courseTo(mapper.last_lat, mapper.last_lng, gps.location.lat(), gps.location.lng());

	// Convert angle
	mapper.current_heading = mapper.current_heading * PI / 180;

	// Process data if position actually changed
	// The x-component is multiplied with -1 in order to flip it over the y-axis
	if (mapper.current_length != 0.0)
	{
		double x_component = mapper.current_length * cos(mapper.current_heading) * -1;
		double y_component = mapper.current_length * sin(mapper.current_heading);

		// Save new data into 10m array
		// Calculate x and y component of the current vector
		mapper.ten_meter_x_array[mapper.ten_counter] = x_component;
		mapper.ten_meter_y_array[mapper.ten_counter] = y_component;

		// Increment Counter for meter array (or reset it if the end of the array is reached)
		if (mapper.ten_counter < (size_of_meter_array - 1))
		{
			mapper.ten_counter += 1;
		}
		else
		{
			mapper.ten_counter = 0;
		}

		// Calculate total lenght of the vectors inside the meter_arrays
		double x_sum = 0, y_sum = 0;
		double length = 0;
		for (int i = 0; i < size_of_meter_array; i++)
		{ // Assume than x and y are the same size (they have to!)
			x_sum = x_sum + mapper.ten_meter_x_array[i];
			y_sum = y_sum + mapper.ten_meter_y_array[i];
		}
		length = sqrt((x_sum * x_sum) + (y_sum * y_sum)); // Pythagorean theorem
		mapper.temp_length = length;

		// Save the new larger vector into the final array it it is long enough
		if (length >= 10)
		{
			mapper.path_x_array[mapper.path_counter] = x_sum;
			mapper.path_y_array[mapper.path_counter] = y_sum;
#ifdef DEBUG
			/*Serial.println("!!Saved to path array!!");
			Serial.print("mapper.path_counter: ");		Serial.println(mapper.path_counter);*/
#endif

			// Increment variable / reset it in case of an overflow
			mapper.path_counter += 1;
			if (mapper.path_counter == (sizeof(mapper.path_x_array) / 8) - 1)
			{
				mapper.path_counter = 0;
			}

			// Clear 10m array
			for (int i = 0; i < size_of_meter_array; i++)
			{
				mapper.ten_meter_x_array[i] = 0;
				mapper.ten_meter_y_array[i] = 0;
			}
			// Reset counter
			mapper.ten_counter = 0;

			// Print array
#ifdef DEBUG
			// for (int i = 0; i < size_of_path_array; i++) {		//Assume than x and y are the same size (they have to!)
			//	Serial.print(mapper.path_x_array[i]);
			//	Serial.print("|");
			//	Serial.print(mapper.path_y_array[i]);
			//	Serial.print("; ");
			// }
			// Serial.println("######\n\n");
#endif // DEBUG
		}

		// Debug output
		/*
		Serial.print(": ");		Serial.println();
		*/
		/*
		Serial.print("mapper.current_length: ");		Serial.println(mapper.current_length);
		Serial.print("mapper.current_heading: ");		Serial.println(mapper.current_heading);
		Serial.print("x_component: ");					Serial.println(x_component);
		Serial.print("y_component: ");					Serial.println(y_component);
		Serial.print("x_sum: ");						Serial.println(x_sum);
		Serial.print("y_sum: ");						Serial.println(y_sum);
		Serial.print("length: ");						Serial.println(length);
		Serial.print("mapper.ten_counter: ");			Serial.println(mapper.ten_counter);

		//Print arrays
		Serial.println("mapper.ten_meter_x_array: ");
		for (int i = 0; i < size_of_meter_array; i++) {		//Assume than x and y are the same size (they have to!)
			Serial.print(mapper.ten_meter_x_array[i]);
			Serial.print("|");
			Serial.print(mapper.ten_meter_y_array[i]);
			Serial.print("; ");
		}


		Serial.println("\n-----");
		*/
	}

	// Update reference
	mapper.last_lat = gps.location.lat();
	mapper.last_lng = gps.location.lng();
}

void draw_gps_path()
{
	u8g2.clearBuffer();

#ifndef USE_REAL_ARRAY
	// Change this if ten_meter_array is no longer being displayed
	int size_of_display_array = sizeof(mapper.ten_meter_x_array) / 8;

	// Variables to determine how large the boundaries have to be
	double x_min = 100000, x_max = -100000;
	double y_min = 100000, y_max = -100000;
	double x_span = -1, y_span = -1;

	double temp_x_sum = 0, temp_y_sum = 0;

	// Determine min and max values of vectors
	for (int i = 0; i <= size_of_display_array; i++)
	{
		temp_x_sum += mapper.ten_meter_x_array[i];
		temp_y_sum += mapper.ten_meter_y_array[i];

		if (temp_x_sum > x_max)
			x_max = temp_x_sum;
		if (temp_y_sum > y_max)
			y_max = temp_y_sum;
		if (temp_x_sum < x_min)
			x_min = temp_x_sum;
		if (temp_y_sum < y_min)
			y_min = temp_y_sum;
	}
	// Determine Span
	x_span = x_max - x_min;
	y_span = y_max - y_min;

	// Draw vectors
	int x_coord_start = 0, y_coord_start = 0;
	int x_coord_end = 0, y_coord_end = 0;

	// Avoid IntegerDivideByZero). Exception was unhandled. errors:
	if (x_min == x_max)
		x_min -= 1;
	if (y_min == y_max)
		y_min -= 1;
	// Multiply / Amplify
	x_min *= 100;
	x_max *= 100;
	y_min *= 100;
	y_max *= 100;

	for (int i = 0; i < size_of_display_array; i++)
	{
		x_coord_start = map(mapper.ten_meter_x_array[i] * 100, x_min, x_max, 64, 128);
		y_coord_start = map(mapper.ten_meter_y_array[i] * 100, y_min, y_max, 0, 64);
		x_coord_end = map(mapper.ten_meter_x_array[i + 1] * 100, x_min, x_max, 64, 128);
		y_coord_end = map(mapper.ten_meter_y_array[i + 1] * 100, y_min, y_max, 0, 64);

		// Only draw a line if the next point is not (0|0)
		if (mapper.ten_meter_x_array[i + 1] != 0 && mapper.ten_meter_y_array[i + 1] != 0)
			u8g2.drawLine(x_coord_start, y_coord_start, x_coord_end, y_coord_end);

		Serial.print("x_coord_start: ");
		Serial.println(x_coord_start);
		Serial.print("x_coord_end: ");
		Serial.println(x_coord_end);
		Serial.print("y_coord_start: ");
		Serial.println(y_coord_start);
		Serial.print("y_coord_end: ");
		Serial.println(y_coord_end);
	}

	// Debug output
	/*
	Serial.print(": ");		Serial.println();
	*/
	Serial.print("x_max: ");
	Serial.println(x_max);
	Serial.print("x_min: ");
	Serial.println(x_min);
	Serial.print("y_max: ");
	Serial.println(y_max);
	Serial.print("y_min: ");
	Serial.println(y_min);
	Serial.print("x_span: ");
	Serial.println(x_span);
	Serial.print("y_span: ");
	Serial.println(y_span);
#else
	/*
	Test Data
	*/
	// mapper.path_x_array[0] = 0;		mapper.path_y_array[0] = 0;
	/*mapper.path_x_array[1] = 10;	mapper.path_y_array[1] = 4;
	mapper.path_x_array[2] = 0;		mapper.path_y_array[2] = 4;
	mapper.path_x_array[3] = -8;	mapper.path_y_array[3] = 0;
	mapper.path_x_array[4] = 0;		mapper.path_y_array[4] = -20;
	mapper.path_x_array[5] = 1;		mapper.path_y_array[5] = 1;*/

	// Change this if array is changed
	int size_of_display_array = sizeof(mapper.path_x_array) / sizeof(double);

	// Variables to determine how large the boundaries have to be
	double x_min = 100000, x_max = -100000;
	double y_min = 100000, y_max = -100000;
	double x_span = -1, y_span = -1;

	// Determine min and max values of vectors
	double temp_x_sum = 0, temp_y_sum = 0;
	for (int i = 0; i <= size_of_display_array; i++)
	{
		temp_x_sum += mapper.path_x_array[i];
		temp_y_sum += mapper.path_y_array[i];

		if (temp_x_sum > x_max)
			x_max = temp_x_sum;
		if (temp_y_sum > y_max)
			y_max = temp_y_sum;
		if (temp_x_sum < x_min)
			x_min = temp_x_sum;
		if (temp_y_sum < y_min)
			y_min = temp_y_sum;

		// Cancel for-loop when elements are zero (no more data will follow)
		if (i > 1 && mapper.path_x_array[i] == 0 && mapper.path_y_array[i] == 0)
			break;
	}
	// Determine Span
	x_span = x_max + abs(x_min);
	y_span = y_max + abs(y_min);

	// Only draw the path if the data is present (path length is not 0)
	bool path_valid = 0;
	if (x_span != 0 || y_span != 0)
	{
		path_valid = true;
		float amp_factor = 100.0; // Static 100x (if it's 100 everywhere it won't make a difference)

		// Multiply / Amplify
		x_min = x_min * amp_factor;
		x_max = x_max * amp_factor;
		y_min = y_min * amp_factor;
		y_max = y_max * amp_factor;

		// Avoid "IntegerDivideByZero Exception was unhandled." errors:
		if (x_min == x_max)
			x_min -= 1;
		if (y_min == y_max)
			y_min -= 1;
		if (x_min == 0)
			x_min -= 1;
		if (y_min == 0)
			y_min -= 1;
		if (x_max == 0)
			x_max -= 1;
		if (y_max == 0)
			y_max -= 1;

		// Make sure that the scaling is correct
		/*if (x_min < y_min) y_min = y_min * (y_min / x_min);
		else x_min = x_min * (x_min / y_min);
		if (x_max > y_max) y_max = y_max * (y_max / x_max);
		else x_max = x_max * (x_max / y_max);*/
		if (x_span != 0 && y_span != 0)
		{
			if (x_span > y_span)
			{
				y_min = y_min * (x_span / y_span);
				y_max = y_max * (x_span / y_span);
			}
			else if (y_span > x_span)
			{
				x_min = x_min * (y_span / x_span);
				x_max = x_max * (y_span / x_span);
			}
		}

		// Variables for how large the realtive x and y movement is
		int x_travel = 0, y_travel = 0;

		// This is the coordinate where the first vector starts
		int start_x = map(mapper.path_x_array[0] * amp_factor, x_min, x_max, 63, 127);
		int start_y = map(mapper.path_y_array[0] * amp_factor, y_min, y_max, 63, 0);

		// Draw a circle to indicate where the line starts
		u8g2.drawDisc(start_x, start_y, 2, U8G2_DRAW_ALL);

		// Variables to calculate relative changes in coordinates
		int x_coord_start = 0, y_coord_start = 0;
		int x_coord_end = 0, y_coord_end = 0;

		// Coordinates for the drawLine() function
		int current_x_begin = start_x;
		int current_y_begin = start_y;
		int current_x_end = start_x;
		int current_y_end = start_y;

		// Map coordinates to the size of the display area
		// Add the lendcoordinate of the previous vector so that the new one begins at the end of the old one
		for (unsigned int i = 0; i < size_of_display_array; i++)
		{

			// Calculate relative length of specific new vector
			x_coord_start = map(mapper.path_x_array[i] * amp_factor, x_min, x_max, 63, 127);
			y_coord_start = map(mapper.path_y_array[i] * amp_factor, y_min, y_max, 63, 0);
			x_coord_end = map(mapper.path_x_array[i + 1] * amp_factor, x_min, x_max, 63, 127);
			y_coord_end = map(mapper.path_y_array[i + 1] * amp_factor, y_min, y_max, 63, 0);
			x_travel = x_travel + (x_coord_end - x_coord_start);
			y_travel = y_travel + (y_coord_end - y_coord_start);

			// Calculate endpoint of vector
			current_x_end = current_x_end + x_travel;
			current_y_end = current_y_end + y_travel;

			// Draw line
			u8g2.drawLine(current_x_begin, current_y_begin, current_x_end, current_y_end);

			// Set starting point for the next vector (end of current one)
			current_x_begin = current_x_end;
			current_y_begin = current_y_end;

			// Cancel for-loop when elements are zero (no more data will follow)
			if (i > 1 && mapper.path_x_array[i] == 0 && mapper.path_y_array[i] == 0)
				break;

			// Debug output
			/*Serial.println();
			Serial.print("i: ");				Serial.println(i);
			Serial.print("x_coord_start: ");	Serial.println(x_coord_start);
			Serial.print("x_coord_end: ");		Serial.println(x_coord_end);
			Serial.print("y_coord_start: ");	Serial.println(y_coord_start);
			Serial.print("y_coord_end: ");		Serial.println(y_coord_end);
			Serial.print("x_travel: ");			Serial.println(x_travel);
			Serial.print("y_travel: ");			Serial.println(y_travel);
			Serial.println();*/
		}
	}
	else
	{
		u8g2.setFont(u8g2_font_helvB12_tf);
		u8g2.setCursor(u8g2.getDisplayWidth() - 43, u8g2.getDisplayHeight() / 2);
		u8g2.print("NO");
		u8g2.setCursor(u8g2.getDisplayWidth() - 55, u8g2.getDisplayHeight() / 2 + 13);
		u8g2.print("DATA");
	}
#endif // !USE_REAL_ARRAY

	// General Information
	u8g2.setCursor(0, 8);
	u8g2.setFont(u8g2_font_t0_12_tr);
	u8g2.print("GPS Path:");
	u8g2.drawHLine(2, 10, 56);
	u8g2.setCursor(0, 20);
	u8g2.print("Sat.: " + String(gps.satellites.value()));
	u8g2.setCursor(0, 29);
	u8g2.print("Cntr.: " + String(mapper.path_counter));
	u8g2.setCursor(0, 38);
	if (mapper.temp_length < 10)
		u8g2.print("Len.: " + String(mapper.temp_length));
	else
		u8g2.print("Len.: " + String(mapper.temp_length, 1));
	u8g2.setCursor(0, 51);

	on_time_helper(true);
	u8g2.print("On-time:");
	u8g2.setCursor(0, 60);
	u8g2.print(time_running);

	u8g2.drawVLine((u8g2.getDisplayWidth() / 2) - 3, 0, u8g2.getDisplayHeight());

	u8g2.sendBuffer();
}
#endif

void measure_distance_gps()
{
	// Check if location has changed
	if (gps.location.isUpdated())
	{
		if (!gps_data.startup)
		{ // Called once at startup
			gps_data.startup = 1;
			gps_data.last_lat = gps.location.lat();
			gps_data.last_lng = gps.location.lng();

			// Also update data for Mapper
			mapper.last_lat = gps.location.lat();
			mapper.last_lng = gps.location.lng();
		}
		else
		{
			// gps_data.travel_distance;
			gps_data.travel_distance_km = gps_data.travel_distance_km + (gps.distanceBetween(gps.location.lat(), gps.location.lng(), gps_data.last_lat, gps_data.last_lng) / 1000.0);

			gps_data.last_lat = gps.location.lat();
			gps_data.last_lng = gps.location.lng();

			/*Serial.print("gps_data.travel_distance: ");
			Serial.println(gps_data.travel_distance_km);
			Serial.println(gps.location.lat(), 6);
			Serial.println(gps.location.lng(), 6);*/

#ifdef ENABLE_TRIP_VISUALIZER
			// There was an update to the data, so call the mapper function
			gps_mapper();
#endif
		}
	}
}

void update_gps_data()
{
	if (gps.speed.age() < 1000)
	{
		gps_data.speed = gps.speed.kmph();
	}
	else
	{
		gps_data.speed = 0;
	}

	if (gps.altitude.age() < 1000)
	{
		gps_data.altitude = gps.altitude.meters();
	}
	else
	{
		gps_data.altitude = 0;
	}

	if (gps.course.age() < 1000)
	{
		gps_data.course = gps.course.deg();
	}
	else
	{
		gps_data.course = 0;
	}

	if (gps.satellites.age() < 1000)
	{
		gps_data.satellites = gps.satellites.value();
	}
	else
	{
		gps_data.satellites = 0;
	}
}

void update_gps()
{
	while (Serial2.available() > 0)
	{
		if (gps.encode(Serial2.read()))
		{
			// display_gps_info();
			if (millis() > 5000 && gps.charsProcessed() < 10)
			{
				Serial.println(F("No GPS detected: check wiring."));
				while (true)
					;
			}
		}
	}

	// Update GPS Data struct
	update_gps_data();
#ifdef ENABLE_TRIP_VISUALIZER
	// Calculate Distance
	measure_distance_gps();
#endif
}

void display_gps_info()
{
	Serial.print(F("Location: "));
	if (gps.location.isValid())
	{
		Serial.print(gps.location.lat(), 6);
		Serial.print(F(","));
		Serial.print(gps.location.lng(), 6);
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.print(F("  Date/Time: "));
	if (gps.date.isValid())
	{
		Serial.print(gps.date.month());
		Serial.print(F("/"));
		Serial.print(gps.date.day());
		Serial.print(F("/"));
		Serial.print(gps.date.year());
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.print(F(" "));
	if (gps.time.isValid())
	{
		if (gps.time.hour() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.hour());
		Serial.print(F(":"));
		if (gps.time.minute() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.minute());
		Serial.print(F(":"));
		if (gps.time.second() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.second());
		Serial.print(F("."));
		if (gps.time.centisecond() < 10)
			Serial.print(F("0"));
		Serial.print(gps.time.centisecond());
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.println();
}
// END GPS
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
// GUI
/*
All Display related code goes here
*/

// This function calls the apppropriate GUI drawing function
void gui_selector()
{
	if (gui_selection == 0)
	{
		update_display();
	}
#ifdef ENABLE_TRIP_VISUALIZER
	else if (gui_selection == 1)
	{
		draw_gps_path();
	}
#endif
#ifdef ENABLE_STATS_DISPLAY
	else if (gui_selection == 2)
	{
		draw_stats();
	}
#endif
}

#ifdef ENABLE_STATS_DISPLAY
void draw_stats()
{
	u8g2.clearBuffer();

	// Headline
	u8g2.setFont(u8g2_font_9x18B_tf);
	u8g2.setCursor(0, 10);
	u8g2.print("Statistics:");

	// Data
	u8g2.setFont(u8g2_font_profont12_tf);
	u8g2.setCursor(0, 22);
	u8g2.print("Total Dist.:");
	u8g2.setCursor(75, 22);
	u8g2.printf("%.1lfkm", stats.total_dist);
	u8g2.setCursor(21, 32);
	u8g2.print("Maximum values:");
	u8g2.drawLine(18, 33, 110, 33);
	u8g2.setFont(u8g2_font_profont11_tf);
	u8g2.setCursor(0, 42);
	u8g2.printf("%.1lfkmh", stats.max_speed);
	u8g2.setCursor(46, 42);
	u8g2.printf("Alt:%.0lfm", stats.max_alt);
	u8g2.setCursor(93, 42);
	u8g2.printf("Sat:%i", stats.max_sat);

	u8g2.setFont(u8g2_font_profont12_tf);
	u8g2.setCursor(21, 54);
	u8g2.print("Average values:");
	u8g2.drawLine(18, 55, 110, 55);
	u8g2.setFont(u8g2_font_profont11_tf);
	u8g2.setCursor(0, 64);
	u8g2.printf("%.1lfkmh", 24.5);
	u8g2.setCursor(46, 64);
	u8g2.printf("T:%.0lfC", 35.6);
	u8g2.setCursor(85, 64);
	u8g2.printf("RH:%.0lf%%", 12.2);

	u8g2.sendBuffer();
}
#endif

void update_display()
{
	u8g2.clearBuffer();
	// Display Satellites
	u8g2.setFont(u8g2_font_t0_12_tr);
	u8g2.setCursor(0, 8);
	u8g2.print("Sat.:" + String(gps_data.satellites));
	// Display Battery Voltage
	u8g2.setCursor(42, 8);
	u8g2.print("B:" + String(read_battery_voltage(), 1) + "V");

	// Display SD Info
	u8g2.setFont(u8g2_font_profont10_tf);
	u8g2.setCursor(0, 15);
	if (SD_present)
		u8g2.print("microSD: OK");
	else
		u8g2.print("microSD: X");

	// Display Course
	u8g2.setFont(u8g2_font_t0_12_tr);
	u8g2.setCursor(0, 24);
	u8g2.print("Course: " + String(gps.cardinal(gps_data.course)));

	// Display Time
	u8g2.setFont(u8g2_font_t0_12_tr);
	u8g2.setCursor(81, 8);
	u8g2.print(time_comb);
	u8g2.drawHLine(81, 9, 47);
	// Display Sensor Readings
	u8g2.setFont(u8g2_font_5x7_mr);
	u8g2.setCursor(81, 17);
	u8g2.print(sensor_comb);
	u8g2.drawHLine(76, 18, 52);

	// Display Speed
	u8g2.setCursor(-2, u8g2.getDisplayHeight() - 11);
	u8g2.setFont(u8g2_font_logisoso28_tf);
	u8g2.print(String(gps_data.speed));

	int speed_width = u8g2.getStrWidth(String(gps_data.speed).c_str());
	u8g2.setCursor(speed_width + 2, u8g2.getDisplayHeight() - 11);
	u8g2.setFont(u8g2_font_t0_11b_tf);
	u8g2.print("km/h");

	// Display Altitude
	u8g2.setFont(u8g2_font_5x7_mf);
	u8g2.setCursor(speed_width + 2, u8g2.getDisplayHeight() - 22);
	u8g2.print("Alt.:" + String(gps_data.altitude));
	int altitude_width = u8g2.getStrWidth(String("Alt:" + String(gps.altitude.meters())).c_str());
	u8g2.setCursor(speed_width + altitude_width + 3, u8g2.getDisplayHeight() - 22);
	u8g2.print("m");

	// Display LAT and LNG
	if (gps_data.speed < 10)
	{
		u8g2.setCursor(74, 26);
		u8g2.print("Lat:" + String(gps.location.lat(), 4));
		u8g2.setCursor(74, 34);
		u8g2.print("Lng:" + String(gps.location.lng(), 4));
	}
	else
	{
		u8g2.setCursor(speed_width + 2, 26);
		u8g2.print("Lat:" + String(gps.location.lat(), 3));
		u8g2.setCursor(speed_width + 2, 34);
		u8g2.print("Lng:" + String(gps.location.lng(), 3));
	}

	// Display travelled Distance
	u8g2.setFont(u8g2_font_9x18B_tf);
	u8g2.setCursor(0, u8g2.getDisplayHeight());
	u8g2.print("Dist.:" + String(gps_data.travel_distance_km, 2) + "km");

	////////////////////
	u8g2.sendBuffer();
}
// END GUI
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
// HELPER
/*
This file contains various helper functions
*/
#ifdef ENABLE_PREFERENCES
// Logs total distance to Storage every 10 additional meters
void calculate_total_dist()
{
	static double current_total_dist = 0;

	if (gps_data.travel_distance_km > (current_total_dist + 10))
	{
		current_total_dist = gps_data.travel_distance_km;
		stats.total_dist = stats.total_dist + gps_data.travel_distance_km;
		preferences.begin("pref_stats", false);
		preferences.putDouble("total_dist", stats.total_dist);
		Serial.println("logging dist");
		preferences.end();
	}
}

void calculate_avg()
{
}

// Logs maximum values to internal Storage
void calculate_max()
{
	// Local variables
	static double max_speed = stats.max_speed;
	static int max_alt = stats.max_alt, max_sat = stats.max_sat;

	preferences.begin("pref_stats", false);

	// Speed
	if (gps.speed.kmph() > stats.max_speed)
	{
		stats.max_speed = gps.speed.kmph();
		if (max_speed < stats.max_speed)
		{
			max_speed = stats.max_speed;
			preferences.putDouble("max_speed", max_speed);
			Serial.println("logging speed");
		}
	}

	// Altitude
	if (gps.altitude.meters() > stats.max_alt)
	{
		stats.max_alt = gps.altitude.meters();
		if (max_alt < (int)stats.max_alt)
		{
			max_alt = (int)stats.max_alt;
			preferences.putInt("max_alt", max_alt);
			Serial.println("logging alt");
		}
	}

	// Satellites
	if (gps.satellites.value() > stats.max_sat)
	{
		stats.max_sat = gps.satellites.value();
		if (max_sat < stats.max_sat)
		{
			max_sat = stats.max_sat;
			preferences.putInt("max_sat", max_sat);
			Serial.println("logging sat");
		}
	}

	preferences.end();
}

// Called once at startup to load in previous values
void load_max_avg_values()
{
	preferences.begin("pref_stats", false);

	stats.max_alt = preferences.getInt("max_alt", 1);
	stats.max_sat = preferences.getInt("max_sat", 1);
	stats.max_speed = preferences.getDouble("max_speed", 1.1);
	stats.total_dist = preferences.getDouble("total_dist", 0);
	Serial.printf("total dist: %lf\n", stats.total_dist);

	preferences.end();
}

void log_reset_times()
{
	preferences.begin("pref_stats", false);
	unsigned int reset_times = preferences.getUInt("reset_times", 0);
	reset_times++;
	Serial.printf("Number of restart times: %d\n", reset_times);
	preferences.putUInt("reset_times", reset_times);
	preferences.end();
}
#endif

// END HELPER
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
// OTA
/*
OTA
*/

#ifdef ENABLE_OTA
void init_ota()
{
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);

	// Display
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_9x18B_tf);
	u8g2.setCursor(0, 18);
	u8g2.print("OTA Update");
	u8g2.sendBuffer();

	while (WiFi.waitForConnectResult() != WL_CONNECTED)
	{
		Serial.println("Connection Failed! Rebooting...");
		delay(5000);
		ESP.restart();
	}

	// Port defaults to 3232
	// ArduinoOTA.setPort(3232);

	// Hostname defaults to esp3232-[MAC]
	ArduinoOTA.setHostname("BicycleComputer");

	// No authentication by default
	// ArduinoOTA.setPassword("admin");

	// Password can be set with it's md5 value as well
	// MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
	ArduinoOTA.setPasswordHash("db4fd8b15e65f37ac0c5808c80694cd1");

	ArduinoOTA
		.onStart([]()
				 {
		String type;
		if (ArduinoOTA.getCommand() == U_FLASH)
			type = "sketch";
		else // U_SPIFFS
			type = "filesystem";

		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		Serial.println("Start updating " + type); })
		.onEnd([]()
			   { Serial.println("\nEnd"); })
		.onProgress([](unsigned int progress, unsigned int total)
					{
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
		u8g2.drawFrame(12, 50, 104, 10);
		u8g2.drawBox(14, 52, (progress / (total / 100)), 6);
		u8g2.sendBuffer(); })
		.onError([](ota_error_t error)
				 {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

	ArduinoOTA.begin();

	Serial.println("OTA Ready");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	u8g2.setCursor(0, 40);
	u8g2.print(WiFi.localIP());

	u8g2.sendBuffer();
}

#endif // ENABLE_OTA

// END OTA
/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
// SD
// Saves data to SD
bool sd_log_data()
{
	bool status = 1;
	sd_log_count += 1;
	if (SD_present)
	{
		if (gps.location.age() < 3000 /* && gps.location.isUpdated()*/)
		{
			file.printf("%10.7lf", gps.location.lat());
			file.print(",");
			file.printf("%10.7lf", gps.location.lng());
			file.print(",");
			file.printf("%i:%i:%i,", gps.time.hour(), gps.time.minute(), gps.time.second());
			file.printf("%6.3f,", gps.speed.kmph());
			file.printf("%i,", gps.satellites.value());
			file.printf("%5.2f,", ht2x.readTemperature());
			file.printf("%5.2f,", ht2x.readHumidity());
			file.printf("%i,", analogRead(ldr_pin));
			file.printf("%5.2f,", gps_data.travel_distance_km);
			file.print("\n");
			file.flush();
#ifdef DEBUG
			// Serial.println("Logging successful!");
#endif // DEBUG
		}
	}
	else
	{
		status = 0;
	}
	return status;
}

bool init_sd_logger()
{
	bool status = 1;

	// Get appropriate filename with correct name
	set_filename();

	// Check if file exists
	if (sd.exists(fileName))
	{
#ifdef DEBUG
		Serial.println("Error file already exists!");
#endif // DEBUG
		status = 0;
	}
	// Open file
	if (!file.open(fileName, O_RDWR | O_CREAT))
	{
#ifdef DEBUG
		Serial.println("Error while opening file!");
#endif // DEBUG
		status = 0;
	}
	// Set timestamps
	status = SD_set_timestamps();

	// Print CSV Data
	file.println("Latitude,Longitude,Time,Speed,Satellites,Temperature,Humidity,Light,Distance,");
	file.flush();
	return status;
}

void set_filename()
{
	// Set Filename
	// bool include_day, bool include_date, bool include_time
	filename = String("GPS_Data_") + String(time_comb_helper(false, true, true, false)) + String(".csv");
	unsigned int filename_length = filename.length() + 1;
	// fileName[filename_length];
	filename.toCharArray(fileName, filename_length);
#ifdef DEBUG
	Serial.print("filename_length: ");
	Serial.println(filename_length);
	Serial.print("fileName: ");
	Serial.println(fileName);
#endif // DEBUG
}

bool SD_set_timestamps()
{
	bool status = 1;
	RtcDateTime now_temp = Rtc.GetDateTime();
	if (!file.timestamp(T_CREATE, now_temp.Year(), now_temp.Month(), now_temp.Day(), now_temp.Hour(), now_temp.Minute(), now_temp.Second()))
	{
#ifdef DEBUG
		Serial.println("set create time failed");
#endif // DEBUG
		status = 0;
	}
	if (!file.timestamp(T_WRITE, now_temp.Year(), now_temp.Month(), now_temp.Day(), now_temp.Hour(), now_temp.Minute(), now_temp.Second()))
	{
#ifdef DEBUG
		Serial.println("set write time failed");
#endif // DEBUG
		status = 0;
	}
	if (!file.timestamp(T_ACCESS, now_temp.Year(), now_temp.Month(), now_temp.Day(), now_temp.Hour(), now_temp.Minute(), now_temp.Second()))
	{
#ifdef DEBUG
		Serial.println("set access time failed");
#endif // DEBUG
		status = 0;
	}
	return status;
}

String time_comb_helper(bool include_day, bool include_date, bool include_time, bool time_separator)
{
	RtcDateTime now = Rtc.GetDateTime();
	String t_c;
	String hour_padding = zero_padder(String(now.Hour()));	 // Add Padding to Hours
	String min_padding = zero_padder(String(now.Minute()));	 // Add Padding to Minutes
	String sec_padding = zero_padder(String(now.Second()));	 // Add Padding to Seconds
	String day_padding = zero_padder(String(now.Day()));	 // Add Padding to Days
	String month_padding = zero_padder(String(now.Month())); // Add Padding to Months

	if (include_day)
	{ // Add current day to returned String
		t_c = String(daysOfTheWeek[now.DayOfWeek()]);
		if (include_date || include_time)
		{
			t_c = t_c + String("_");
		}
	}
	if (include_date)
	{
		t_c = t_c + String(day_padding) + "." + String(month_padding) + "." + String(now.Year());
		if (include_time)
		{
			t_c = t_c + String("_");
		}
	}
	if (include_time)
	{
		// Can't always use ":" (file name for example)
		String time_separator_character = "_";
		if (time_separator)
		{
			time_separator_character = ":";
		}
		t_c = t_c + String(hour_padding) + time_separator_character + String(min_padding) + time_separator_character + String(sec_padding);
	}
	return t_c;
}

// END SD
/////////////////////////////////////////////////////////////////////////
