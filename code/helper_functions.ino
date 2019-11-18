/*
This file contains various helper functions
*/

//Logs total distance to Storage every 10 additional meters
void calculate_total_dist() {
	static double current_total_dist = 0;

	if (gps_data.travel_distance_km > (current_total_dist + 10)) {
		current_total_dist = gps_data.travel_distance_km;
		stats.total_dist = stats.total_dist + gps_data.travel_distance_km;
		preferences.begin("pref_stats", false);
		preferences.putDouble("total_dist", stats.total_dist);
		Serial.println("logging dist");
		preferences.end();
	}	
}

void calculate_avg() {

}

//Logs maximum values to internal Storage
void calculate_max() {
	//Local variables
	static double max_speed = stats.max_speed;
	static int max_alt = stats.max_alt, max_sat = stats.max_sat;

	preferences.begin("pref_stats", false);

	//Speed
	if (gps.speed.kmph() > stats.max_speed) {
		stats.max_speed = gps.speed.kmph();
		if (max_speed < stats.max_speed) {
			max_speed = stats.max_speed;
			preferences.putDouble("max_speed", max_speed);
			Serial.println("logging speed");
		}
	}


	//Altitude
	if (gps.altitude.meters() > stats.max_alt) {
		stats.max_alt = gps.altitude.meters();
		if (max_alt < (int)stats.max_alt) {
			max_alt = (int)stats.max_alt;
			preferences.putInt("max_alt", max_alt);
			Serial.println("logging alt");
		}
	}


	//Satellites
	if (gps.satellites.value() > stats.max_sat) {
		stats.max_sat = gps.satellites.value();
		if (max_sat < stats.max_sat) {
			max_sat = stats.max_sat;
			preferences.putInt("max_sat", max_sat);
			Serial.println("logging sat");
		}
	}

	preferences.end();
}

//Called once at startup to load in previous values
void load_max_avg_values() {
	preferences.begin("pref_stats", false);

	stats.max_alt = preferences.getInt("max_alt", 1);
	stats.max_sat = preferences.getInt("max_sat", 1);
	stats.max_speed = preferences.getDouble("max_speed", 1.1);
	stats.total_dist = preferences.getDouble("total_dist", 0);
	Serial.printf("total dist: %lf\n", stats.total_dist);

	preferences.end();
}

void log_reset_times() {
	preferences.begin("pref_stats", false);
	unsigned int reset_times = preferences.getUInt("reset_times", 0);
	reset_times++;
	Serial.printf("Number of restart times: %d\n", reset_times);
	preferences.putUInt("reset_times", reset_times);
	preferences.end();
}