#include <LiquidCrystal.h>
#include <Arduino.h>
#include <avr/wdt.h>

// กำหนดขา ADC สำหรับคีย์แพด
const int KEY_ADC_PIN = A5;
LiquidCrystal lcd(8, 9, 10, 11, 12, 13); // RS,E,D4,D5,D6,D7

unsigned int target_value = 0;

// ถอดรหัสช่วงแรงดันเป็นปุ่ม
char decodeKey(int adc) {
  if (adc > 33 && adc < 60)    return '1';
  else if (adc >= 62 && adc < 100)  return '2';
  else if (adc >= 100 && adc < 135) return '3';
  else if (adc >= 140 && adc < 170) return '4';
  else if (adc >= 175 && adc < 220) return '5';
  else if (adc >= 225 && adc < 289) return '6';
  else if (adc >= 294 && adc < 326) return '7';
  else if (adc >= 330 && adc < 405) return '8';
  else if (adc >= 410 && adc < 473) return '9';
  else if (adc >= 480 && adc < 581) return '0';
  else if (adc >= 843 && adc < 884) return '*';
  else if (adc >= 901 && adc < 936) return '#';
  return 0;
}

// ส่งเป้าหมายไปบอร์ด 1
void sendTarget() {
  char buf[20];
  sprintf(buf, "TARGET:%03u\n", target_value);
  Serial.print(buf);
}

void setup() {
  Serial.begin(9600); // 9600 bps
  lcd.begin(16,2);
  lcd.print("Set Target:");
  wdt_enable(WDTO_2S);
}

void loop() {
  wdt_reset();
  static char lastKey = 0;
  int adcVal = analogRead(KEY_ADC_PIN);
  static int lastAdc = 0;
  static byte stableCount = 0;

  // ตรวจว่าค่า ADC ไม่เปลี่ยนมาก (<6) จึงถือว่า stable
  if (abs(adcVal - lastAdc) < 6 && adcVal > 33) {
    stableCount++;
    if (stableCount > 3) { // debounce ~3*20ms = 60ms
      char k = decodeKey(adcVal);
      if (k && k != lastKey) {
        lastKey = k;
        if (k >= '0' && k <= '9') {
          unsigned int newVal = target_value*10 + (k - '0');
          if (newVal <= 999) target_value = newVal;
        } else if (k == '*') {
          target_value = 0;
        } else if (k == '#') {
          sendTarget();
        }
      }
    }
  } else {
    stableCount = 0;
    lastKey = 0;
  }
  lastAdc = adcVal;

  lcd.setCursor(0,1);
  lcd.print("Val:");
  lcd.print(target_value);
  lcd.print("      ");
  delay(20);
}
