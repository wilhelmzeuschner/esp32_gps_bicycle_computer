/*
All Display related code goes here
*/

//This function calls the apppropriate GUI drawing function
void gui_selector() {
	if (gui_selection == 0) {
		update_display();
	}
	else if (gui_selection == 1) {
		draw_gps_path();
	}
	else if (gui_selection == 2) {
		draw_stats();
	}
}

void draw_stats() {
	u8g2.clearBuffer();

	//Headline
	u8g2.setFont(u8g2_font_9x18B_tf);
	u8g2.setCursor(0, 10);
	u8g2.print("Statistics:");
	
	//Data
	u8g2.setFont(u8g2_font_profont12_tf);
	u8g2.setCursor(0, 22);
	u8g2.print("Total Dist.:");
	u8g2.setCursor(75, 22);
	u8g2.printf("%.1lfkm", stats.total_dist);
	u8g2.setCursor(21, 32);
	u8g2.print("Maximum values:");	
	u8g2.drawLine(18, 33, 110, 33);
	u8g2.setFont(u8g2_font_profont11_tf);
	u8g2.setCursor(0, 42);		u8g2.printf("%.1lfkmh", stats.max_speed);
	u8g2.setCursor(46, 42);		u8g2.printf("Alt:%.0lfm", stats.max_alt);
	u8g2.setCursor(93, 42);		u8g2.printf("Sat:%i", stats.max_sat);

	u8g2.setFont(u8g2_font_profont12_tf);
	u8g2.setCursor(21, 54);
	u8g2.print("Average values:");
	u8g2.drawLine(18, 55, 110, 55);
	u8g2.setFont(u8g2_font_profont11_tf);
	u8g2.setCursor(0, 64);		u8g2.printf("%.1lfkmh", 24.5);
	u8g2.setCursor(46, 64);		u8g2.printf("T:%.0lfC", 35.6);
	u8g2.setCursor(85, 64);		u8g2.printf("RH:%.0lf%%", 12.2);


	u8g2.sendBuffer();
}

void update_display() {
	u8g2.clearBuffer();
	//Display Satellites
	u8g2.setFont(u8g2_font_t0_12_tr);
	u8g2.setCursor(0, 8);
	u8g2.print("Sat.:" + String(gps_data.satellites));
	//Display Battery Voltage
	u8g2.setCursor(42, 8);
	u8g2.print("B:" + String(read_battery_voltage(), 1) + "V");
	
	//Display SD Info
	u8g2.setFont(u8g2_font_profont10_tf);
	u8g2.setCursor(0, 15);
	if (SD_present) u8g2.print("microSD: OK");
	else u8g2.print("microSD: X");

	//Display Course
	u8g2.setFont(u8g2_font_t0_12_tr);
	u8g2.setCursor(0, 24);
	u8g2.print("Course: " + String(gps.cardinal(gps_data.course)));	

	//Display Time
	u8g2.setFont(u8g2_font_t0_12_tr);
	u8g2.setCursor(81, 8);
	u8g2.print(time_comb);
	u8g2.drawHLine(81, 9, 47);
	//Display Sensor Readings
	u8g2.setFont(u8g2_font_5x7_mr);
	u8g2.setCursor(81, 17);
	u8g2.print(sensor_comb);
	u8g2.drawHLine(76, 18, 52);
	
	//Display Speed
	u8g2.setCursor(-2, u8g2.getDisplayHeight() - 11);
	u8g2.setFont(u8g2_font_logisoso28_tf);
	u8g2.print(String(gps_data.speed));

	int speed_width = u8g2.getStrWidth(String(gps_data.speed).c_str());
	u8g2.setCursor(speed_width + 2, u8g2.getDisplayHeight() - 11);
	u8g2.setFont(u8g2_font_t0_11b_tf);
	u8g2.print("km/h");

	//Display Altitude
	u8g2.setFont(u8g2_font_5x7_mf);
	u8g2.setCursor(speed_width + 2, u8g2.getDisplayHeight() - 22);
	u8g2.print("Alt.:" + String(gps_data.altitude));
	int altitude_width = u8g2.getStrWidth(String("Alt:" + String(gps.altitude.meters())).c_str());
	u8g2.setCursor(speed_width + altitude_width + 3, u8g2.getDisplayHeight() - 22);
	u8g2.print("m");

	//Display LAT and LNG
	if (gps_data.speed < 10) {
		u8g2.setCursor(74, 26);
		u8g2.print("Lat:" + String(gps.location.lat(), 4));
		u8g2.setCursor(74, 34);
		u8g2.print("Lng:" + String(gps.location.lng(), 4));
	}
	else {
		u8g2.setCursor(speed_width + 2, 26);
		u8g2.print("Lat:" + String(gps.location.lat(), 3));
		u8g2.setCursor(speed_width + 2, 34);
		u8g2.print("Lng:" + String(gps.location.lng(), 3));
	}
	

	//Display travelled Distance
	u8g2.setFont(u8g2_font_9x18B_tf);
	u8g2.setCursor(0, u8g2.getDisplayHeight());
	u8g2.print("Dist.:" + String(gps_data.travel_distance_km, 2) + "km");


	////////////////////
	u8g2.sendBuffer();
}