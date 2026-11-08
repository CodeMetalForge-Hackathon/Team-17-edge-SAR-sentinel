// Forge SAR sensor node: Grove sensors -> JSON lines over serial (10 Hz).
// One Arduino = one node. Set NODE_ID per board.
// Pi -> node commands (one char): 'B' = beacon (buzzer + LED for 2 s), 'A' = ack blink,
//                                 'Z' = re-zero the heat baseline (use if the room temperature changed).
//
// Wiring (Grove Base Shield, voltage switch on 3V3):
//   D2  Mini PIR motion        (HIGH = motion)
//   D3  Vibration SW-420       (polarity set by VIB_ACTIVE)
//   D5  Water sensor           (LOW = wet; optional, input is pulled up so an unwired pin reads "dry")
//   D6  Buzzer                 (output)
//   D7  LED                    (output)
//   D8  DHT11 temp/humidity    (needs the DHT library; set USE_DHT 0 if you don't have it)
//   A0  Sound sensor           (analog, reported as loud_max / loud_mean)
//   A1  Light sensor           (analog, optional)
//
// There is no flame sensor in this build. "flame" in the JSON is a HEAT flag: 1 when the DHT11
// temperature is HEAT_RISE_C or more above the temperature measured shortly after boot.
// The rest of the pipeline (brain / policy) treats flame = 1 as FIRE, so nothing else changes.
//
// Polarity: open the Serial Monitor, trigger each sensor, and flip the *_ACTIVE define if it reads backwards.

#define NODE_ID "n1"
#define USE_DHT 1

// Which serial port carries the JSON. Defaults to USB Serial. If another file or build flag already
// defines NODE_SERIAL (for example Serial1 on the UNO Q), that one wins.
#ifndef NODE_SERIAL
#define NODE_SERIAL Serial
#endif

#define PIN_PIR 2
#define PIN_VIB 3
#define PIN_WATER 5
#define PIN_BUZZ 6
#define PIN_LED 7
#define PIN_DHT 8
#define PIN_LOUD A0
#define PIN_LIGHT A1

#define VIB_ACTIVE LOW
#define WATER_ACTIVE LOW

#define HEAT_RISE_C 5.0            // degrees above the startup temperature that counts as fire (3.0 is easier to trigger in a demo)
#define HEAT_BASELINE_AFTER_MS 5000  // take the baseline this long after boot, once the DHT has a valid reading

#if USE_DHT
#include <DHT.h>
DHT dht(PIN_DHT, DHT11);
#endif

const unsigned long PERIOD_MS = 100;  // 10 Hz output
unsigned long lastOut = 0;
unsigned long beaconUntil = 0;

// Accumulate between outputs so short events (a tap, a shake) are not missed.
int loudMax = 0;
long loudSum = 0;
int loudN = 0;
bool vibSeen = false;  // any vibration during this 100 ms window -> reported as vib = 1

float tempC = NAN, hum = NAN;
float baseTemp = NAN;
unsigned long lastDht = 0;

void setup() {
  NODE_SERIAL.begin(115200);
  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_VIB, INPUT);
  pinMode(PIN_WATER, INPUT_PULLUP);  // unwired = HIGH = dry, so no phantom FLOOD events
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
#if USE_DHT
  dht.begin();
#endif
}

void handleCommands() {
  while (NODE_SERIAL.available()) {
    char c = NODE_SERIAL.read();
    if (c == 'B') beaconUntil = millis() + 2000;
    if (c == 'A') { digitalWrite(PIN_LED, HIGH); delay(80); digitalWrite(PIN_LED, LOW); }
    if (c == 'Z') baseTemp = tempC;  // re-zero heat baseline to the current temperature
  }
}

void loop() {
  handleCommands();

  // Fast sampling between outputs
  int loud = analogRead(PIN_LOUD);
  if (loud > loudMax) loudMax = loud;
  loudSum += loud;
  loudN++;
  if (digitalRead(PIN_VIB) == VIB_ACTIVE) vibSeen = true;

  // Beacon output
  bool beacon = millis() < beaconUntil;
  digitalWrite(PIN_BUZZ, beacon && ((millis() / 250) % 2 == 0));
  if (beacon) digitalWrite(PIN_LED, (millis() / 125) % 2);
  else digitalWrite(PIN_LED, LOW);

#if USE_DHT
  if (millis() - lastDht > 2000) {  // DHT11 is slow, read every 2 s
    tempC = dht.readTemperature();
    hum = dht.readHumidity();
    lastDht = millis();
  }
#endif

  // Heat baseline, taken once shortly after boot
  if (isnan(baseTemp) && !isnan(tempC) && millis() > HEAT_BASELINE_AFTER_MS) baseTemp = tempC;
  bool hot = !isnan(baseTemp) && !isnan(tempC) && tempC >= baseTemp + HEAT_RISE_C;

  if (millis() - lastOut >= PERIOD_MS) {
    lastOut = millis();
    NODE_SERIAL.print("{\"node\":\"");
    NODE_SERIAL.print(NODE_ID);
    NODE_SERIAL.print("\",\"ms\":");
    NODE_SERIAL.print(lastOut);
    NODE_SERIAL.print(",\"pir\":");
    NODE_SERIAL.print(digitalRead(PIN_PIR) == HIGH ? 1 : 0);
    NODE_SERIAL.print(",\"vib\":");
    NODE_SERIAL.print(vibSeen ? 1 : 0);
    NODE_SERIAL.print(",\"flame\":");
    NODE_SERIAL.print(hot ? 1 : 0);
    NODE_SERIAL.print(",\"water\":");
    NODE_SERIAL.print(digitalRead(PIN_WATER) == WATER_ACTIVE ? 1 : 0);
    NODE_SERIAL.print(",\"loud_max\":");
    NODE_SERIAL.print(loudMax);
    NODE_SERIAL.print(",\"loud_mean\":");
    NODE_SERIAL.print(loudN ? loudSum / loudN : 0);
    NODE_SERIAL.print(",\"light\":");
    NODE_SERIAL.print(analogRead(PIN_LIGHT));
    if (!isnan(tempC)) { NODE_SERIAL.print(",\"temp\":"); NODE_SERIAL.print(tempC, 1); }
    if (!isnan(hum)) { NODE_SERIAL.print(",\"hum\":"); NODE_SERIAL.print(hum, 1); }
    NODE_SERIAL.println("}");
    loudMax = 0;
    loudSum = 0;
    loudN = 0;
    vibSeen = false;
  }
}
