
#include <M5StickCPlus.h>
#include <M5_RoverC.h>
#include <WiFi.h>
#include <WiFiUdp.h>

M5_RoverC rover;
WiFiUDP udp;

const char* ssid = "RoverControl";
const char* password = "Rover2026Test!";

const int UDP_PORT = 4210;
const int SPEED = 35;
const unsigned long TIMEOUT_MS = 400;

unsigned long lastCommand = 0;
bool moving = false;

void stopRover() {
  rover.setSpeed(0, 0, 0);
  moving = false;
}

void executeCommand(char cmd) {
  switch (cmd) {
    case 'F':
      rover.setSpeed(0, SPEED, 0);
      moving = true;
      break;

    case 'B':
      rover.setSpeed(0, -SPEED, 0);
      moving = true;
      break;

    case 'L':
      rover.setSpeed(-SPEED, 0, 0);
      moving = true;
      break;

    case 'R':
      rover.setSpeed(SPEED, 0, 0);
      moving = true;
      break;

    case 'Q':
      rover.setSpeed(0, 0, SPEED);
      moving = true;
      break;

    case 'E':
      rover.setSpeed(0, 0, -SPEED);
      moving = true;
      break;

    case 'S':
      stopRover();
      break;

    default:
      stopRover();
      break;
  }
}

void setup() {
  M5.begin();
  rover.begin();

  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);

  stopRover();

  WiFi.mode(WIFI_AP);
  bool apReady = WiFi.softAP(
    ssid, password, 1, 0, 2
  );

  M5.Lcd.setCursor(5, 10);

  if (!apReady) {
    M5.Lcd.setTextColor(RED);
    M5.Lcd.println("WiFi Failed");
    return;
  }

  udp.begin(UDP_PORT);

  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.println("Rover Ready");

  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(5, 45);
  M5.Lcd.println(WiFi.softAPIP());

  M5.Lcd.setCursor(5, 65);
  M5.Lcd.println("UDP Port: 4210");
}

void loop() {
  M5.update();

  // Physical stop button
  if (M5.BtnA.isPressed()) {
    stopRover();
    delay(10);
    return;
  }

  int packetSize = udp.parsePacket();

  if (packetSize > 0) {
    char cmd = udp.read();

    // Discard remaining packet bytes
    while (udp.available()) {
      udp.read();
    }

    // Only accept commands from the Pi
    if (udp.remoteIP() ==
        IPAddress(192, 168, 4, 2)) {
      lastCommand = millis();
      executeCommand(cmd);
    }
  }

  // Safety watchdog
  if (moving &&
      millis() - lastCommand > TIMEOUT_MS) {
    stopRover();
  }

  delay(5);
}
