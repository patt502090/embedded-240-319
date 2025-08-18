#include <Keypad.h>

// ===== Keypad 4x4 =====
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  { '1','2','3','A' },
  { '4','5','6','B' },
  { '7','8','9','C' },
  { '*','0','#','D' }
};
// เปลี่ยนขาตามการต่อของคุณ
byte rowPins[ROWS] = {11, 10, 9, 8};
byte colPins[COLS] = {7, 6, 5, 4};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

String buffer = "";          // เก็บค่าที่กดสะสม
const int LED_ACK = 13;      // ไฟแจ้งเตือน ACK (เลือกใช้ได้)

void setup() {
  pinMode(LED_ACK, OUTPUT);
  digitalWrite(LED_ACK, LOW);

  // 9600 bps, 8 data bits, Even parity, 1 stop bit (ให้ตรงกับผู้รับ)
  Serial.begin(9600, SERIAL_8E1);
  // Serial.println("Sender Ready");  // debug (ถ้าอยากเปิด)
}

void loop() {
  // 1) อ่านปุ่มจาก Keypad
  char key = keypad.getKey();
  if (key) {
    if (key == '#') {
      // 2) กด # = ส่งทั้ง buffer ออกไปเป็น 1 บรรทัด (มี \n ปิดท้าย)
      Serial.println(buffer);
      buffer = ""; // เคลียร์เตรียมรอบใหม่
    } else if (key == '*') {
      // 3) กด * = ล้าง buffer
      buffer = "";
    } else {
      // 4) ปุ่มอื่นๆ = ต่อท้ายลง buffer
      buffer += key;
    }
  }

  // 5) เช็คว่ามี ACK กลับมาหรือไม่ (ไม่บล็อก)
  static String line = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line.trim();
      if (line.equalsIgnoreCase("ACK")) {
        // ได้ ACK → กระพริบ LED แจ้งเตือนสั้นๆ
        digitalWrite(LED_ACK, HIGH);
        delay(120);
        digitalWrite(LED_ACK, LOW);
      }
      line = "";
    } else {
      line += c;
      if (line.length() > 64) line.remove(0); // กันล้น
    }
  }
}
