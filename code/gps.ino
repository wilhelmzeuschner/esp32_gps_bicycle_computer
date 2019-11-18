/*
This file contains all GPS related Code
*/

/*
Time sync code
*/
void sync_rtc_with_gps() {	
	if (gps.time.isValid() && gps.date.isValid() && gps.time.isUpdated() && gps.date.isUpdated() && gps.satellites.value() > 3 && gps.time.hour() != 24) {
		Rtc.SetDateTime(RtcDateTime(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour() + UTC_ADJ, gps.time.minute(), gps.time.second()));
		Serial.println("Time synced");
		time_sync_flag = 1;
	}
}

/*
Determine whether or not GPS has a fix
*/
bool check_gps_fix() {
	bool fix = 0;

	if (gps.satellites.value() >= MIN_NO_SAT) {		//If number is not 1 (which means no fix)
		fix = 1;									//a fix must be established, min no. of satellites must be reached
		Serial.println("has fix");
	}
	
	return fix;
}

/*
This function captures data and saves the path as vectors.
It is called whenever the GPS coordinates are updated.
*/
void gps_mapper() {
	int size_of_meter_array = sizeof(mapper.ten_meter_x_array) / sizeof(double);
	int size_of_path_array = sizeof(mapper.path_x_array) / sizeof(double);

	//Calculate new values for current dataset
	mapper.current_length = gps.distanceBetween(mapper.last_lat, mapper.last_lng, gps.location.lat(), gps.location.lng());
	mapper.current_heading = gps.courseTo(mapper.last_lat, mapper.last_lng, gps.location.lat(), gps.location.lng());

	//Convert angle
	mapper.current_heading = mapper.current_heading * PI / 180;

	//Process data if position actually changed
	//The x-component is multiplied with -1 in order to flip it over the y-axis
	if (mapper.current_length != 0.0) {
		double x_component = mapper.current_length * cos(mapper.current_heading) * -1;
		double y_component = mapper.current_length * sin(mapper.current_heading);

		//Save new data into 10m array
		//Calculate x and y component of the current vector
		mapper.ten_meter_x_array[mapper.ten_counter] = x_component;
		mapper.ten_meter_y_array[mapper.ten_counter] = y_component;

		//Increment Counter for meter array (or reset it if the end of the array is reaced
		if (mapper.ten_counter < (size_of_meter_array - 1)) {
			mapper.ten_counter += 1;
		}
		else {
			mapper.ten_counter = 0;
		}

		//Calculate total lenght of the vectors inside the meter_arrays
		double x_sum = 0, y_sum = 0;
		double length = 0;
		for (int i = 0; i < size_of_meter_array; i++) {		//Assume than x and y are the same size (they have to!)
			x_sum = x_sum + mapper.ten_meter_x_array[i];
			y_sum = y_sum + mapper.ten_meter_y_array[i];
		}
		length = sqrt((x_sum * x_sum) + (y_sum * y_sum));		//Pythagorean theorem
		mapper.temp_length = length;

		//Save the new larger vector into the final array it it is long enough
		if (length >= 10) {
			mapper.path_x_array[mapper.path_counter] = x_sum;
			mapper.path_y_array[mapper.path_counter] = y_sum;
#ifdef DEBUG
			/*Serial.println("!!Saved to path array!!");
			Serial.print("mapper.path_counter: ");		Serial.println(mapper.path_counter);*/
#endif

			//Increment variable / reset it in case of an overflow
			mapper.path_counter += 1;
			if (mapper.path_counter == (sizeof(mapper.path_x_array) / 8) -1) {
				mapper.path_counter = 0;
			}

			//Clear 10m array
			for (int i = 0; i < size_of_meter_array; i++) {
				mapper.ten_meter_x_array[i] = 0;
				mapper.ten_meter_y_array[i] = 0;
			}
			//Reset counter
			mapper.ten_counter = 0;

			//Print array
#ifdef DEBUG
			//for (int i = 0; i < size_of_path_array; i++) {		//Assume than x and y are the same size (they have to!)
			//	Serial.print(mapper.path_x_array[i]);
			//	Serial.print("|");
			//	Serial.print(mapper.path_y_array[i]);
			//	Serial.print("; ");
			//}
			//Serial.println("######\n\n");
#endif // DEBUG

			
		}

		//Debug output
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
	
	//Update reference
	mapper.last_lat = gps.location.lat();
	mapper.last_lng = gps.location.lng();
}

void draw_gps_path() {
	u8g2.clearBuffer();

#ifndef USE_REAL_ARRAY
	//Change this if ten_meter_array is no longer being displayed
	int size_of_display_array = sizeof(mapper.ten_meter_x_array) / 8;

	//Variables to determine how large the boundaries have to be
	double x_min = 100000, x_max = -100000;
	double y_min = 100000, y_max = -100000;
	double x_span = -1, y_span = -1;

	double temp_x_sum = 0, temp_y_sum = 0;

	//Determine min and max values of vectors
	for (int i = 0; i <= size_of_display_array; i++) {
		temp_x_sum += mapper.ten_meter_x_array[i];
		temp_y_sum += mapper.ten_meter_y_array[i];

		if (temp_x_sum > x_max) x_max = temp_x_sum;
		if (temp_y_sum > y_max) y_max = temp_y_sum;
		if (temp_x_sum < x_min) x_min = temp_x_sum;
		if (temp_y_sum < y_min) y_min = temp_y_sum;

	}
	//Determine Span
	x_span = x_max - x_min;
	y_span = y_max - y_min;

	//Draw vectors
	int x_coord_start = 0, y_coord_start = 0;
	int x_coord_end = 0, y_coord_end = 0;

	//Avoid IntegerDivideByZero). Exception was unhandled. errors:	
	if (x_min == x_max) x_min -= 1;
	if (y_min == y_max) y_min -= 1;
	//Multiply / Amplify
	x_min *= 100;
	x_max *= 100;
	y_min *= 100;
	y_max *= 100;

	for (int i = 0; i < size_of_display_array; i++) {
		x_coord_start = map(mapper.ten_meter_x_array[i] * 100, x_min, x_max, 64, 128);
		y_coord_start = map(mapper.ten_meter_y_array[i] * 100, y_min, y_max, 0, 64);
		x_coord_end = map(mapper.ten_meter_x_array[i + 1] * 100, x_min, x_max, 64, 128);
		y_coord_end = map(mapper.ten_meter_y_array[i + 1] * 100, y_min, y_max, 0, 64);

		//Only draw a line if the next point is not (0|0)
		if (mapper.ten_meter_x_array[i + 1] != 0 && mapper.ten_meter_y_array[i + 1] != 0) u8g2.drawLine(x_coord_start, y_coord_start, x_coord_end, y_coord_end);


		Serial.print("x_coord_start: ");	Serial.println(x_coord_start);
		Serial.print("x_coord_end: ");		Serial.println(x_coord_end);
		Serial.print("y_coord_start: ");	Serial.println(y_coord_start);
		Serial.print("y_coord_end: ");		Serial.println(y_coord_end);
	}


	//Debug output
	/*
	Serial.print(": ");		Serial.println();
	*/
	Serial.print("x_max: ");		Serial.println(x_max);
	Serial.print("x_min: ");		Serial.println(x_min);
	Serial.print("y_max: ");		Serial.println(y_max);
	Serial.print("y_min: ");		Serial.println(y_min);
	Serial.print("x_span: ");		Serial.println(x_span);
	Serial.print("y_span: ");		Serial.println(y_span);
#else
	/*
	Test Data
	*/
	//mapper.path_x_array[0] = 0;		mapper.path_y_array[0] = 0;
	/*mapper.path_x_array[1] = 10;	mapper.path_y_array[1] = 4;
	mapper.path_x_array[2] = 0;		mapper.path_y_array[2] = 4;
	mapper.path_x_array[3] = -8;	mapper.path_y_array[3] = 0;
	mapper.path_x_array[4] = 0;		mapper.path_y_array[4] = -20;
	mapper.path_x_array[5] = 1;		mapper.path_y_array[5] = 1;*/

	//Change this if array is changed
	int size_of_display_array = sizeof(mapper.path_x_array) / sizeof(double);

	//Variables to determine how large the boundaries have to be
	double x_min = 100000, x_max = -100000;
	double y_min = 100000, y_max = -100000;
	double x_span = -1, y_span = -1;

	//Determine min and max values of vectors
	double temp_x_sum = 0, temp_y_sum = 0;
	for (int i = 0; i <= size_of_display_array; i++) {
		temp_x_sum += mapper.path_x_array[i];
		temp_y_sum += mapper.path_y_array[i];

		if (temp_x_sum > x_max) x_max = temp_x_sum;
		if (temp_y_sum > y_max) y_max = temp_y_sum;
		if (temp_x_sum < x_min) x_min = temp_x_sum;
		if (temp_y_sum < y_min) y_min = temp_y_sum;

		//Cancel for-loop when elements are zero (no more data will follow)
		if (i > 1 && mapper.path_x_array[i] == 0 && mapper.path_y_array[i] == 0) break;
	}
	//Determine Span
	x_span = x_max + abs(x_min);
	y_span = y_max + abs(y_min);

	//Only draw the path if the data is present (path lenght is not 0)
	bool path_valid = 0;
	if (x_span != 0 || y_span != 0) {
		path_valid = true;
		float amp_factor = 100.0;		//Static 100x (if it's 100 everywhere it won't make a difference)
		
		//Multiply / Amplify	
		x_min = x_min * amp_factor;
		x_max = x_max * amp_factor;
		y_min = y_min * amp_factor;
		y_max = y_max * amp_factor;

		//Avoid "IntegerDivideByZero Exception was unhandled." errors:	
		if (x_min == x_max) x_min -= 1;
		if (y_min == y_max) y_min -= 1;
		if (x_min == 0) x_min -= 1;
		if (y_min == 0) y_min -= 1;
		if (x_max == 0) x_max -= 1;
		if (y_max == 0) y_max -= 1;

		//Make sure that the scaling is correct
		/*if (x_min < y_min) y_min = y_min * (y_min / x_min);
		else x_min = x_min * (x_min / y_min);
		if (x_max > y_max) y_max = y_max * (y_max / x_max);
		else x_max = x_max * (x_max / y_max);*/
		if (x_span != 0 && y_span != 0) {
			if (x_span > y_span) {
				y_min = y_min * (x_span / y_span);
				y_max = y_max * (x_span / y_span);
			}
			else if (y_span > x_span) {
				x_min = x_min * (y_span / x_span);
				x_max = x_max * (y_span / x_span);
			}
		}		

		//Variables for how large the realtive x and y movement is
		int x_travel = 0, y_travel = 0;

		//This is the coordinate where the first vector starts
		int start_x = map(mapper.path_x_array[0] * amp_factor, x_min, x_max, 63, 127);
		int start_y = map(mapper.path_y_array[0] * amp_factor, y_min, y_max, 63, 0);

		//Draw a circle to indicate where the line starts
		u8g2.drawDisc(start_x, start_y, 2, U8G2_DRAW_ALL);

		//Variables to calculate relative changes in coordinates
		int x_coord_start = 0, y_coord_start = 0;
		int x_coord_end = 0, y_coord_end = 0;

		//Coordniates for the drawLine() function
		int current_x_begin = start_x;
		int current_y_begin = start_y;
		int current_x_end = start_x;
		int current_y_end = start_y;

		//Map coordinates to the size of the display area
		//Add the lendcoordinate of the previous vector so that the new one begins at the end of the old one
		for (unsigned int i = 0; i < size_of_display_array; i++) {

			//Calculate relative lenght of specific new vector
			x_coord_start = map(mapper.path_x_array[i] * amp_factor, x_min, x_max, 63, 127);
			y_coord_start = map(mapper.path_y_array[i] * amp_factor, y_min, y_max, 63, 0);
			x_coord_end = map(mapper.path_x_array[i + 1] * amp_factor, x_min, x_max, 63, 127);
			y_coord_end = map(mapper.path_y_array[i + 1] * amp_factor, y_min, y_max, 63, 0);
			x_travel = x_travel + (x_coord_end - x_coord_start);
			y_travel = y_travel + (y_coord_end - y_coord_start);

			//Calculate endpoint of vector
			current_x_end = current_x_end + x_travel;
			current_y_end = current_y_end + y_travel;

			//Draw line
			u8g2.drawLine(current_x_begin, current_y_begin, current_x_end, current_y_end);

			//Set starting point for the next vector (end of current one)
			current_x_begin = current_x_end;
			current_y_begin = current_y_end;

			//Cancel for-loop when elements are zero (no more data will follow)
			if (i > 1 && mapper.path_x_array[i] == 0 && mapper.path_y_array[i] == 0) break;

			//Debug output
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
	else {
		u8g2.setFont(u8g2_font_helvB12_tf);
		u8g2.setCursor(u8g2.getDisplayWidth() - 43, u8g2.getDisplayHeight() / 2);		
		u8g2.print("NO");
		u8g2.setCursor(u8g2.getDisplayWidth() - 55, u8g2.getDisplayHeight() / 2 + 13);
		u8g2.print("DATA");
	}
#endif // !USE_REAL_ARRAY

	//General Information
	u8g2.setCursor(0, 8);
	u8g2.setFont(u8g2_font_t0_12_tr);
	u8g2.print("GPS Path:");
	u8g2.drawHLine(2, 10, 56);
	u8g2.setCursor(0, 20);
	u8g2.print("Sat.: " + String(gps.satellites.value()));
	u8g2.setCursor(0, 29);
	u8g2.print("Cntr.: " + String(mapper.path_counter));
	u8g2.setCursor(0, 38);
	if (mapper.temp_length < 10) u8g2.print("Len.: " + String(mapper.temp_length));
	else u8g2.print("Len.: " + String(mapper.temp_length, 1));
	u8g2.setCursor(0, 51);

	on_time_helper(true);
	u8g2.print("On-time:");
	u8g2.setCursor(0, 60);
	u8g2.print(time_running);

	u8g2.drawVLine((u8g2.getDisplayWidth() / 2) - 3, 0, u8g2.getDisplayHeight());

	u8g2.sendBuffer();
}

void measure_distance_gps() {	
	//Check if location has changed
	if (gps.location.isUpdated()) {
		if (!gps_data.startup) {		//Called once at startup
			gps_data.startup = 1;
			gps_data.last_lat = gps.location.lat();
			gps_data.last_lng = gps.location.lng();

			//Also update data for Mapper
			mapper.last_lat = gps.location.lat();
			mapper.last_lng = gps.location.lng();
		}
		else {
			//gps_data.travel_distance;		
			gps_data.travel_distance_km = gps_data.travel_distance_km + (gps.distanceBetween(gps.location.lat(), gps.location.lng(), gps_data.last_lat, gps_data.last_lng) / 1000.0);

			gps_data.last_lat = gps.location.lat();
			gps_data.last_lng = gps.location.lng();

			/*Serial.print("gps_data.travel_distance: ");
			Serial.println(gps_data.travel_distance_km);
			Serial.println(gps.location.lat(), 6);
			Serial.println(gps.location.lng(), 6);*/

			//There was an update to the data, so call the mapper function
			gps_mapper();
		}		
	}
}

void update_gps_data() {
	if (gps.speed.age() < 1000) {
		gps_data.speed = gps.speed.kmph();
	}
	else {
		gps_data.speed = 0;
	}

	if (gps.altitude.age() < 1000) {
		gps_data.altitude = gps.altitude.meters();
	}
	else {
		gps_data.altitude = 0;
	}

	if (gps.course.age() < 1000) {
		gps_data.course = gps.course.deg();
	}
	else {
		gps_data.course = 0;
	}

	if (gps.satellites.age() < 1000) {
		gps_data.satellites = gps.satellites.value();
	}
	else {
		gps_data.satellites = 0;
	}

}

void update_gps() {
	while (Serial2.available() > 0) {
		if (gps.encode(Serial2.read())) {
			//display_gps_info();
			if (millis() > 5000 && gps.charsProcessed() < 10)
			{
				Serial.println(F("No GPS detected: check wiring."));
				while (true);
			}
		}			
	}
		
	//Update GPS Data struct
	update_gps_data();
	//Calculate Distance
	measure_distance_gps();
}

void display_gps_info() {
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
		if (gps.time.hour() < 10) Serial.print(F("0"));
		Serial.print(gps.time.hour());
		Serial.print(F(":"));
		if (gps.time.minute() < 10) Serial.print(F("0"));
		Serial.print(gps.time.minute());
		Serial.print(F(":"));
		if (gps.time.second() < 10) Serial.print(F("0"));
		Serial.print(gps.time.second());
		Serial.print(F("."));
		if (gps.time.centisecond() < 10) Serial.print(F("0"));
		Serial.print(gps.time.centisecond());
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.println();
}