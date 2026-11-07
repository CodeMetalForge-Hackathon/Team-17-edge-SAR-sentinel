// Forge SAR sensor node: Grove sensors -> JSON lines over USB serial (10 Hz).
// Each Arduino = one node. Set NODE_ID per board.
// Pi -> node commands (one char + newline): 'B' = beacon (buzzer + LED 2 s), 'A' = ack blink.
//
// Wiring (Grove-to-male-jumper cables, or a Grove Base Shield if you have one):
//   D2  Mini PIR motion      (HIGH = motion)
//   D3  Vibration SW-420      (check polarity on your board, see VIB_ACTIVE)
//   D4  Flame sensor          (LOW = flame on most Grove flame boards, see FLAME_ACTIVE)
//   D5  Water sensor          (LOW = wet on Grove water sensor, see WATER_ACTIVE)
//   D6  Buzzer                (output)
//   D7  Red/Green LED         (output)
//   A0  Loudness sensor       (analog)
//   A1  Light sensor v1.2     (analog)
//   D8  DHT11 temp/humidity   (optional, set USE_DHT 1 and install Seeed "Grove Temperature And Humidity Sensor" lib)
//
// Polarity: open Serial Monitor, trigger each sensor, and flip the *_ACTIVE define if it reads backwards.

#define NODE_ID "n1"
#define USE_DHT 0

#define PIN_PIR 2
#define PIN_VIB 3
#define PIN_FLAME 4
#define PIN_WATER 5
#define PIN_BUZZ 6
#define PIN_LED 7
#define PIN_LOUD A0
#define PIN_LIGHT A1
#define PIN_DHT 8

#define VIB_ACTIVE HIGH
#define FLAME_ACTIVE LOW
#define WATER_ACTIVE LOW

#if USE_DHT
#include "DHT.h"
DHT dht(PIN_DHT, DHT11);
#endif

const unsigned long PERIOD_MS = 100;  // 10 Hz output
unsigned long lastOut = 0;
unsigned long beaconUntil = 0;

// Accumulate between outputs so short events (a tap, a shake) are not missed.
// vib is 0/1 per 100 ms period ("was the vibration sensor active at all"), NOT a count of loop()
// passes: loop() runs hundreds of times per period, so a raw count turns one tap into vib=40+.
// The Pi sums these into vib_hits = 0..10 per 1 s window (tap ~1-2, shake 5+).
int loudMax = 0;
long loudSum = 0;
int loudN = 0;
bool vibSeen = false;

float tempC = NAN, hum = NAN;
unsigned long lastDht = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_VIB, INPUT);
  pinMode(PIN_FLAME, INPUT);
  pinMode(PIN_WATER, INPUT);
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
#if USE_DHT
  dht.begin();
#endif
}

void handleCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'B') beaconUntil = millis() + 2000;
    if (c == 'A') { digitalWrite(PIN_LED, HIGH); delay(80); digitalWrite(PIN_LED, LOW); }
  }
}

void loop() {
  handleCommands();

  // Fast sampling between outputs
  int loud = analogRead(PIN_LOUD);
  if (loud > loudMax) loudMax = loud;
  loudSum += loud; loudN++;
  if (digitalRead(PIN_VIB) == VIB_ACTIVE) vibSeen = true;

  // Beacon output
  bool beacon = millis() < beaconUntil;
  digitalWrite(PIN_BUZZ, beacon && ((millis() / 250) % 2 == 0));
  if (beacon) digitalWrite(PIN_LED, (millis() / 125) % 2);

#if USE_DHT
  if (millis() - lastDht > 2000) {  // DHT11 is slow, read every 2 s
    tempC = dht.readTemperature();
    hum = dht.readHumidity();
    lastDht = millis();
  }
#endif

  if (millis() - lastOut >= PERIOD_MS) {
    lastOut = millis();
    Serial.print("{\"node\":\""); Serial.print(NODE_ID);
    Serial.print("\",\"ms\":"); Serial.print(lastOut);
    Serial.print(",\"pir\":"); Serial.print(digitalRead(PIN_PIR) == HIGH ? 1 : 0);
    Serial.print(",\"vib\":"); Serial.print(vibSeen ? 1 : 0);
    Serial.print(",\"flame\":"); Serial.print(digitalRead(PIN_FLAME) == FLAME_ACTIVE ? 1 : 0);
    Serial.print(",\"water\":"); Serial.print(digitalRead(PIN_WATER) == WATER_ACTIVE ? 1 : 0);
    Serial.print(",\"loud_max\":"); Serial.print(loudMax);
    Serial.print(",\"loud_mean\":"); Serial.print(loudN ? loudSum / loudN : 0);
    Serial.print(",\"light\":"); Serial.print(analogRead(PIN_LIGHT));
    if (!isnan(tempC)) { Serial.print(",\"temp\":"); Serial.print(tempC, 1); }
    if (!isnan(hum)) { Serial.print(",\"hum\":"); Serial.print(hum, 1); }
    Serial.println("}");
    loudMax = 0; loudSum = 0; loudN = 0; vibSeen = false;
  }
}
