// ----------------- ฝั่งรับ (Arduino) -----------------
#include <LiquidCrystal.h>                  // ไลบรารี LCD

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);     // RS=12, E=11, D4=5, D5=4, D6=3, D7=2
String line;                                // เก็บข้อความที่อ่านได้จาก UART

void setup() {
  Serial.begin(9600);                       // เปิดพอร์ตอนุกรม 9600 bps
  lcd.begin(16, 2);                         // เริ่มจอ 16คอลัมน์ 2แถว
  lcd.clear();                              // ล้างจอ
  lcd.print("Receiver Ready");              // ข้อความต้อนรับ
  delay(800);                               // หน่วงให้เห็น
  lcd.clear();                              // ล้างเพื่อเตรียมแสดงจริง
}

void loop() {
  if (Serial.available()) {                                // มีข้อมูลเข้ามาไหม?
    line = Serial.readStringUntil('\n');                   // อ่านถึง '\n' หนึ่งบรรทัด
    line.trim();                                           // ตัดช่องว่าง/CR/LF รอบ ๆ ออก

    // คาดหวังรูปแบบ "Temp: <number> C"
    if (line.startsWith("Temp:")) {                        // ถ้ามี prefix ตรงตามที่ส่ง
      int colon = line.indexOf(':');                       // หาตำแหน่ง ':'
      String rest = line.substring(colon + 1);             // ตัดเอามาหลัง ':'
      rest.trim();                                         // ตัดช่องว่างหน้า/หลัง

      int sp = rest.indexOf(' ');                          // หา space ก่อนหน่วย 'C'
      String num = (sp >= 0) ? rest.substring(0, sp)       // ถ้ามีเว้นวรรค → "27"
                             : rest;                       // ถ้าไม่มี → ทั้งก้อน

      // แสดงผลลง LCD
      lcd.setCursor(0, 0);                                 // บรรทัดบน
      lcd.print("Temperature   ");                         // พาดหัว + ช่องว่างลบตัวเก่า
      lcd.setCursor(0, 1);                                 // บรรทัดล่าง
      lcd.print("Now: ");                                   // คำว่า Now:
      lcd.print(num);                                      // ตัวเลข
      lcd.print(" C      ");                               // หน่วย + เว้นลบตัวเก่า
    } else {
      // ถ้าข้อความไม่ได้ขึ้นต้นด้วย "Temp:" ก็แสดง raw เฉย ๆ
      lcd.setCursor(0, 0);
      lcd.print("RX:             ");                       // ล้างหางด้วยช่องว่าง
      lcd.setCursor(0, 1);
      // แสดงได้สูงสุด 16 ตัวอักษร (ตัดให้พอดี)
      String s = line.substring(0, 16);
      lcd.print(s);
      // เติมช่องว่างลบตัวเก่าในคอลัมน์ที่เหลือ
      for (int i = s.length(); i < 16; i++) lcd.print(' ');
    }
  }
}
