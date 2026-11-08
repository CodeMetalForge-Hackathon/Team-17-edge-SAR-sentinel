// Forge SAR survivor node for the chance-of-life model (model/survivor.py on the Dhrubo branch).
// Arduino UNO Q MCU sketch: Grove sensors -> one JSON line every 100 ms (10 Hz) on NODE_SERIAL.
// The existing UNO Q relay (docs/UNO_Q_STANDALONE.md) carries these lines over Wi-Fi to the Pi's
// ~/uno-q-readings/n1.jsonl, and pi/model_feed.py turns them into the model's CSV.
//
// Wiring (Grove Base Shield, VOLTAGE SWITCH ON 3V3 -- A0/A1 on the UNO Q are not 5 V tolerant):
//   D2   Mini PIR Motion Sensor        -> pir
//   D3   Vibration Sensor (SW-420)     -> vib
//   D4   Ultrasonic Ranger             -> us_cm
//   D5   (unused: no flame sensor; flame is always sent as 0)
//   D6   Buzzer                        (beacon output, optional)
//   D7   LED                           (beacon output, optional)
//   D8   Temp & Humidity (DHT11)       -> temp
//   A0   Loudness Sensor               -> loud    (mean over each 100 ms)
//   A2   Sound Sensor                  -> sound   (PEAK over each 100 ms, for knock detection)
//   I2C  3-Axis Digital Accel +-1.5g   -> ax, ay, az in g (sample with the biggest jolt in each 100 ms)
// No CO2 sensor and no flame sensor in this build: no co2 field is sent, flame is always 0.
// Set USE_FLAME 1 and wire a flame sensor to D5 if you get one.
//
// Pi -> node commands (one char): 'B' beacon 2 s, 'A' ack blink.
// Polarity: watch the output, trigger each sensor, and flip *_ACTIVE if it reads backwards.

#include <Wire.h>

#define NODE_ID "n1"
#define USE_DHT 1
#define USE_FLAME 0   // no flame sensor fitted; an unwired D5 would float and fake flames

#ifndef NODE_SERIAL
#define NODE_SERIAL Serial   // router-backed Monitor on current UNO Q cores (what the relay reads)
#endif

#define PIN_PIR 2
#define PIN_VIB 3
#define PIN_US 4
#define PIN_FLAME 5
#define PIN_BUZZ 6
#define PIN_LED 7
#define PIN_DHT 8
#define PIN_LOUD A0
#define PIN_SOUND A2

#define VIB_ACTIVE LOW
#define FLAME_ACTIVE LOW     // Grove flame sensor pulls SIG low when it sees a flame

#define ACC_ADDR 0x4C        // MMA7660FC
#define ACC_COUNTS_PER_G 21.33f

#if USE_DHT
#include <DHT.h>
DHT dht(PIN_DHT, DHT11);
#endif

const unsigned long PERIOD_MS = 100;
const unsigned long US_PERIOD_MS = 100;   // one ultrasonic ping per output window
unsigned long lastOut = 0, lastDht = 0, lastAcc = 0, lastUs = 0, beaconUntil = 0;

long loudSum = 0; int loudN = 0;
int soundMax = 0;
bool vibSeen = false, flameSeen = false;
bool accOk = false;
float accBase = 1.0f;                     // slow running |a|, so the jolt is measured against it
float pkX = 0, pkY = 0, pkZ = 0, pkDev = -1;
float lastX = 0, lastY = 0, lastZ = 1;
float usCm = 400;
float tempC = NAN;

// ---------------------------------------------------------------- MMA7660 (no library needed)
bool accWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ACC_ADDR);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool accBegin() {
  return accWrite(0x07, 0x00)      // standby
      && accWrite(0x08, 0x00)      // 120 samples/s
      && accWrite(0x07, 0x01);     // active
}

bool accRead(float &x, float &y, float &z) {
  Wire.beginTransmission(ACC_ADDR);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ACC_ADDR, 3) != 3) return false;
  int8_t v[3];
  for (int i = 0; i < 3; i++) {
    uint8_t r = Wire.read();
    if (r & 0x40) return false;            // alert bit: register was updating, skip this sample
    int8_t s = r & 0x3F;
    if (s & 0x20) s -= 64;                 // 6-bit two's complement
    v[i] = s;
  }
  x = v[0] / ACC_COUNTS_PER_G; y = v[1] / ACC_COUNTS_PER_G; z = v[2] / ACC_COUNTS_PER_G;
  return true;
}

// ---------------------------------------------------------------- Grove Ultrasonic Ranger (one signal pin)
float pingCm() {
  pinMode(PIN_US, OUTPUT);
  digitalWrite(PIN_US, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_US, HIGH); delayMicroseconds(5);
  digitalWrite(PIN_US, LOW);
  pinMode(PIN_US, INPUT);
  unsigned long us = pulseIn(PIN_US, HIGH, 25000UL);   // 25 ms timeout ~ 4 m
  if (us == 0) return 400;                             // nothing in range
  return us / 58.0f;
}

void setup() {
  NODE_SERIAL.begin(115200);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_VIB, INPUT);
  pinMode(PIN_FLAME, INPUT);
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  Wire.begin();
  accOk = accBegin();
#if USE_DHT
  dht.begin();
#endif
}

void handleCommands() {
  while (NODE_SERIAL.available()) {
    char c = NODE_SERIAL.read();
    if (c == 'B') beaconUntil = millis() + 2000;
    if (c == 'A') { digitalWrite(PIN_LED, HIGH); delay(80); digitalWrite(PIN_LED, LOW); }
  }
}

void loop() {
  handleCommands();
  unsigned long now = millis();

  // fast sampling between outputs
  loudSum += analogRead(PIN_LOUD); loudN++;
  int snd = analogRead(PIN_SOUND);
  if (snd > soundMax) soundMax = snd;
  if (digitalRead(PIN_VIB) == VIB_ACTIVE) vibSeen = true;
#if USE_FLAME
  if (digitalRead(PIN_FLAME) == FLAME_ACTIVE) flameSeen = true;
#endif

  if (now - lastAcc >= 8) {                 // ~120 Hz, matches the sensor's rate
    lastAcc = now;
    float x, y, z;
    if (!accOk) accOk = accBegin();
    if (accOk && accRead(x, y, z)) {
      float mag = sqrtf(x * x + y * y + z * z);
      float dev = fabsf(mag - accBase);
      accBase += 0.01f * (mag - accBase);
      lastX = x; lastY = y; lastZ = z;
      if (dev > pkDev) { pkDev = dev; pkX = x; pkY = y; pkZ = z; }
    }
  }

  if (now - lastUs >= US_PERIOD_MS) { lastUs = now; usCm = pingCm(); }

#if USE_DHT
  if (now - lastDht > 2000) {               // DHT11 is slow
    float t = dht.readTemperature();
    if (!isnan(t)) tempC = t;
    lastDht = now;
  }
#endif

  bool beacon = now < beaconUntil;
  digitalWrite(PIN_BUZZ, beacon && ((now / 250) % 2 == 0));
  digitalWrite(PIN_LED, beacon ? (now / 125) % 2 : LOW);

  if (now - lastOut >= PERIOD_MS) {
    lastOut = now;
    if (pkDev < 0) { pkX = lastX; pkY = lastY; pkZ = lastZ; }
    NODE_SERIAL.print("{\"node\":\""); NODE_SERIAL.print(NODE_ID);
    NODE_SERIAL.print("\",\"ms\":");     NODE_SERIAL.print(lastOut);
    NODE_SERIAL.print(",\"pir\":");      NODE_SERIAL.print(digitalRead(PIN_PIR) == HIGH ? 1 : 0);
    NODE_SERIAL.print(",\"loud\":");     NODE_SERIAL.print(loudN ? loudSum / loudN : 0);
    NODE_SERIAL.print(",\"sound\":");    NODE_SERIAL.print(soundMax);
    NODE_SERIAL.print(",\"ax\":");       NODE_SERIAL.print(pkX, 3);
    NODE_SERIAL.print(",\"ay\":");       NODE_SERIAL.print(pkY, 3);
    NODE_SERIAL.print(",\"az\":");       NODE_SERIAL.print(pkZ, 3);
    NODE_SERIAL.print(",\"acc_ok\":");   NODE_SERIAL.print(accOk ? 1 : 0);
    NODE_SERIAL.print(",\"vib\":");      NODE_SERIAL.print(vibSeen ? 1 : 0);
    NODE_SERIAL.print(",\"us_cm\":");    NODE_SERIAL.print(usCm, 1);
    NODE_SERIAL.print(",\"flame\":");    NODE_SERIAL.print(flameSeen ? 1 : 0);
    if (!isnan(tempC)) { NODE_SERIAL.print(",\"temp\":"); NODE_SERIAL.print(tempC, 1); }
    NODE_SERIAL.println("}");
    loudSum = 0; loudN = 0; soundMax = 0;
    vibSeen = false; flameSeen = false; pkDev = -1;
  }
}
