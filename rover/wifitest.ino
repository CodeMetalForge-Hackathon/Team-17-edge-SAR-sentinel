
#include <M5StickCPlus.h>
#include <WiFi.h>

const char* ssid = "RoverControl";
const char* password = "Rover2026Test!";

void setup() {
  M5.begin();

  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(GREEN);

  WiFi.mode(WIFI_AP);

  bool success = WiFi.softAP(
    ssid,
    password,
    1,     // Wi-Fi channel
    0,     // SSID visible
    2      // Maximum clients
  );

  M5.Lcd.setCursor(5, 10);

  if (success) {
    M5.Lcd.println("WiFi Ready");

    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(1);

    M5.Lcd.setCursor(5, 45);
    M5.Lcd.println(ssid);

    M5.Lcd.setCursor(5, 65);
    M5.Lcd.println(WiFi.softAPIP());
  } else {
    M5.Lcd.setTextColor(RED);
    M5.Lcd.println("WiFi Failed");
  }
}

void loop() {
  delay(1000);
}
