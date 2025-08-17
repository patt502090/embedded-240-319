#include <LiquidCrystal.h>
#include <Arduino.h>
#include <avr/wdt.h>

LiquidCrystal lcd(8,9,10,11,12,13); // RS,E,D4,D5,D6,D7

unsigned int pills = 0;
unsigned int progress = 0;

void setup() {
  Serial.begin(9600); // รับข้อมูลจากบอร์ด 1 (PILLS, PROGRESS)
  lcd.begin(16, 2);
  lcd.print("Monitor Ready");
  wdt_enable(WDTO_2S);
}

void loop() {
  wdt_reset();
  // อ่านสตริงจาก Serial
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    if (line.startsWith("PILLS:")) {
      pills = line.substring(6).toInt();
    } else if (line.startsWith("PROGRESS:")) {
      progress = line.substring(9).toInt();
    }
  }
  // แสดงผล
  lcd.setCursor(0,0);
  lcd.print("Pills:");
  lcd.print(pills);
  lcd.print("    ");
  lcd.setCursor(0,1);
  lcd.print("Prog:");
  lcd.print(progress);
  lcd.print("%   ");

  // ถึง 100% → ทำงานล้าง (ตัวอย่างด้วย LED)
  if (progress >= 100) {
    // เปิด LED/โซลินอยด์ 3-5s แล้วส่ง RESET กลับไปบอร์ด 1
    // (สามารถใช้ digitalWrite() กับพินจริง)
    Serial.print("RESET\n");
    pills = 0;
    progress = 0;
  }
  delay(100);
}
