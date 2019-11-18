//Saves data to SD
bool sd_log_data() {
	bool status = 1;
	sd_log_count += 1;
	if (SD_present) {
		if (gps.location.age() < 3000/* && gps.location.isUpdated()*/) {
			file.printf("%10.7lf", gps.location.lat());
			file.print(",");
			file.printf("%10.7lf", gps.location.lng());
			file.print(",");
			file.printf("%i:%i:%i,", gps.time.hour(), gps.time.minute(), gps.time.second());
			file.printf("%6.3f,", gps.speed.kmph());
			file.printf("%i,", gps.satellites.value());
			file.printf("%5.2f,", myHumidity.readTemperature());
			file.printf("%5.2f,", myHumidity.readHumidity());
			file.printf("%i,", analogRead(ldr_pin));
			file.printf("%5.2f,", gps_data.travel_distance_km);
			file.print("\n");
			file.flush();
#ifdef DEBUG
			//Serial.println("Logging successful!");
#endif // DEBUG			
		}
	}	
	else {
		status = 0;
	}
	return status;
}

bool init_sd_logger() {
	bool status = 1;

	//Get appropriate filename with correct name
	set_filename();

	//Check if file exists
	if (sd.exists(fileName)) {
#ifdef DEBUG
		Serial.println("Error file already exists!");
#endif // DEBUG		
		status = 0;
	}
	//Open file
	if (!file.open(fileName, O_RDWR | O_CREAT)) {
#ifdef DEBUG
		Serial.println("Error while opening file!");
#endif // DEBUG		
		status = 0;
	}
	//Set timestamps
	status = SD_set_timestamps();

	//Print CSV Data
	file.println("Latitude,Longitude,Time,Speed,Satellites,Temperature,Humidity,Light,Distance,");
	file.flush();
	return status;
}

void set_filename() {
	//Set Filename
	//bool include_day, bool include_date, bool include_time
	filename = String("GPS_Data_") + String(time_comb_helper(false, true, true, false)) + String(".csv");
	unsigned int filename_length = filename.length() + 1;
	//fileName[filename_length];
	filename.toCharArray(fileName, filename_length);
#ifdef DEBUG
	Serial.print("filename_length: ");
	Serial.println(filename_length);
	Serial.print("fileName: ");
	Serial.println(fileName);
#endif // DEBUG	
}

bool SD_set_timestamps() {
	bool status = 1;
	RtcDateTime now_temp = Rtc.GetDateTime();
	if (!file.timestamp(T_CREATE, now_temp.Year(), now_temp.Month(), now_temp.Day(), now_temp.Hour(), now_temp.Minute(), now_temp.Second())) {
#ifdef DEBUG
		Serial.println("set create time failed");
#endif // DEBUG
		status = 0;
	}
	if (!file.timestamp(T_WRITE, now_temp.Year(), now_temp.Month(), now_temp.Day(), now_temp.Hour(), now_temp.Minute(), now_temp.Second())) {
#ifdef DEBUG
		Serial.println("set write time failed");
#endif // DEBUG		
		status = 0;
	}
	if (!file.timestamp(T_ACCESS, now_temp.Year(), now_temp.Month(), now_temp.Day(), now_temp.Hour(), now_temp.Minute(), now_temp.Second())) {
#ifdef DEBUG
		Serial.println("set access time failed");
#endif // DEBUG
		status = 0;
	}
	return status;
}


String time_comb_helper(bool include_day, bool include_date, bool include_time, bool time_separator) {
	RtcDateTime now = Rtc.GetDateTime();
	String t_c;
	String hour_padding = zero_padder(String(now.Hour()));		//Add Padding to Hours
	String min_padding = zero_padder(String(now.Minute()));		//Add Padding to Minutes
	String sec_padding = zero_padder(String(now.Second()));		//Add Padding to Seconds
	String day_padding = zero_padder(String(now.Day()));		//Add Padding to Days
	String month_padding = zero_padder(String(now.Month()));	//Add Padding to Months

	if (include_day) {  //Add current day to returned String
		t_c = String(daysOfTheWeek[now.DayOfWeek()]);
		if (include_date || include_time) {
			t_c = t_c + String("_");
		}
	}
	if (include_date) {
		t_c = t_c + String(day_padding) + "." + String(month_padding) + "." + String(now.Year());
		if (include_time) {
			t_c = t_c + String("_");
		}
	}
	if (include_time) {
		//Can't always use ":" (file name for example)
		String time_separator_character = "_";
		if (time_separator) {
			time_separator_character = ":";
		}
		t_c = t_c + String(hour_padding) + time_separator_character + String(min_padding) + time_separator_character + String(sec_padding);

	}
	return t_c;
}
