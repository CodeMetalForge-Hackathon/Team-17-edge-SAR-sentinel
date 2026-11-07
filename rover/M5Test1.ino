
#include <M5StickCPlus.h>
#include <M5_RoverC.h>

M5_RoverC rover;

const int SPEED = 35;

int mode = 0;
bool running = false;
unsigned long startTime = 0;

const char* names[] = {
  "STOP",
  "FORWARD",
  "BACKWARD",
  "LEFT",
  "RIGHT",
  "ROTATE LEFT",
  "ROTATE RIGHT"
};

void stopRover() {
  rover.setSpeed(0, 0, 0);
  running = false;
}

void moveRover(int m) {

  switch (m) {

    // Corrected Forward
    case 1:
      rover.setSpeed(0, SPEED, 0);
      break;

    // Corrected Backward
    case 2:
      rover.setSpeed(0, -SPEED, 0);
      break;

    // Corrected Left
    // Strafe LEFT
    case 3:
      rover.setSpeed(-SPEED, 0, 0);
      break;

    // Strafe RIGHT
    case 4:
      rover.setSpeed(SPEED, 0, 0);
      break;

    // Rotate LEFT (counterclockwise)
    case 5:
      rover.setSpeed(0, 0, SPEED);
      break;

    // Rotate RIGHT (clockwise)
    case 6:
      rover.setSpeed(0, 0, -SPEED);
      break;
    default:
      stopRover();
      return;
  }

  running = true;
  startTime = millis();
}

void showStatus() {
  M5.Lcd.fillScreen(BLACK);

  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(GREEN);
  M5.Lcd.setCursor(10, 10);
  M5.Lcd.println("RoverC-Pro");

  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(10, 45);
  M5.Lcd.println(names[mode]);

  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(10, 100);
  M5.Lcd.println("A: Select  B: Run");
}

void setup() {
  M5.begin();
  rover.begin();

  M5.Lcd.setRotation(1);

  stopRover();
  showStatus();
}

void loop() {
  M5.update();

  // Button A: select movement
  if (M5.BtnA.wasPressed()) {
    stopRover();

    mode = (mode + 1) % 7;
    showStatus();
  }

  // Button B: execute movement
  if (M5.BtnB.wasPressed()) {
    if (running) {
      stopRover();
    } else {
      moveRover(mode);
    }

    showStatus();
  }

  // Stop automatically after 500 ms
  if (running && millis() - startTime >= 500) {
    stopRover();
    showStatus();
  }

  delay(10);
}
