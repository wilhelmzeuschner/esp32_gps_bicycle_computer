/*
OTA
*/

#ifdef ENABLE_OTA
void init_ota() {
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);

	//Display
	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_9x18B_tf);
	u8g2.setCursor(0, 18);
	u8g2.print("OTA Update");
	u8g2.sendBuffer();

	while (WiFi.waitForConnectResult() != WL_CONNECTED) {
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
		.onStart([]() {
		String type;
		if (ArduinoOTA.getCommand() == U_FLASH)
			type = "sketch";
		else // U_SPIFFS
			type = "filesystem";

		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		Serial.println("Start updating " + type);
	})
		.onEnd([]() {
		Serial.println("\nEnd");
	})
		.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
		u8g2.drawFrame(12, 50, 104, 10);
		u8g2.drawBox(14, 52, (progress / (total / 100)), 6);
		u8g2.sendBuffer();
	})
		.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});

	ArduinoOTA.begin();

	Serial.println("OTA Ready");
	Serial.print("IP address: ");
	Serial.println(WiFi.localIP());

	u8g2.setCursor(0, 40);
	u8g2.print(WiFi.localIP());

	u8g2.sendBuffer();
}


#endif // ENABLE_OTA
