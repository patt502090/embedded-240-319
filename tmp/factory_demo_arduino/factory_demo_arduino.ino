// factory_demo_arduino.ino
// Demo: water-level (ADC), keypad setpoint, status via Serial, PWM pump, watchdog
// Boards: Arduino UNO / ATmega328P @ 16 MHz
#include <Keypad.h>
#include <avr/wdt.h>

// -------- Pins --------
const uint8_t PIN_WATER_ADC = A0;   // water level sensor (0-1023)
const uint8_t PIN_PUMP_PWM  = 5;    // OC0B (Timer0 PWM-capable on UNO), drives pump/valve
const uint8_t PIN_LED       = 13;   // heartbeat

// -------- Keypad (4x3) --------
// Rows: D2..D5, Cols: D6..D8  (adjust to your wiring)
const byte ROWS = 4, COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {2,3,4,5};
byte colPins[COLS] = {6,7,8};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// -------- State --------
int setpoint = 600;               // target water level (0-1023)
int adcVal   = 0;                 // current water level
String entry = "";                // keypad entry buffer
unsigned long lastPrint = 0;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_PUMP_PWM, OUTPUT);
  analogWrite(PIN_PUMP_PWM, 0);   // pump off

  Serial.begin(9600);             // 8N1
  wdt_enable(WDTO_2S);            // enable watchdog ~2s

  // Small banner
  Serial.println(F("Factory demo (ADC+Keypad+PWM+WDT)"));
  Serial.println(F("Enter setpoint (0..1023) then press #"));
}

void loop() {
  // ----- Read keypad (non-blocking) -----
  char k = keypad.getKey();
  if (k) {
    if (k >= '0' && k <= '9') {
      entry += k;
      Serial.print(k);
    } else if (k == '#') {
      if (entry.length() > 0) {
        long v = entry.toInt();
        if (v < 0) v = 0;
        if (v > 1023) v = 1023;
        setpoint = (int)v;
      }
      Serial.print(F(" -> setpoint="));
      Serial.println(setpoint);
      entry = "";
    } else if (k == '*') {
      entry = "";
      Serial.println(F("\nCleared"));
    }
  }

  // ----- Sample water level (ADC) -----
  adcVal = analogRead(PIN_WATER_ADC);

  // ----- Simple control: drive pump based on error -----
  int error = setpoint - adcVal;          // positive -> need more water
  int duty  = 0;
  if (error > 0) {
    // map error (0..512) -> duty (0..255)
    long e = error; if (e > 512) e = 512;
    duty = (int)(e * 255L / 512L);
  } else {
    duty = 0; // above setpoint -> stop pump
  }
  analogWrite(PIN_PUMP_PWM, duty);

  // ----- Heartbeat + periodic status -----
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    Serial.print(F("ADC=")); Serial.print(adcVal);
    Serial.print(F("  SP=")); Serial.print(setpoint);
    Serial.print(F("  PWM=")); Serial.println(duty);
  }

  // ----- Pet the watchdog -----
  wdt_reset();
}
