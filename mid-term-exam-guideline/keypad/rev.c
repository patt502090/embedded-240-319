#include <avr/io.h>                 // เข้าถึงรีจิสเตอร์ AVR

// รับข้อความผ่าน Serial 9600, 8E1
// เมื่อรับครบ 1 บรรทัด (จบด้วย '\n') → แสดงค่า แล้วส่ง "ACK\n" กลับ
void setup() {
  Serial.begin(9600, SERIAL_8E1);
  // Serial.println("Receiver Ready"); // debug (ถ้าอยากเปิด)
}

String line; // บัฟเฟอร์สะสม 1 บรรทัด

void processLine(const String& s) {
  String payload = s;
  payload.trim();  // ตัดช่องว่างหัว-ท้าย

  // แสดงค่าใน Serial Monitor ฝั่งผู้รับ (ไว้ดูเฉยๆ)
  Serial.print("Received: ");
  Serial.println(payload);

  // ตอบกลับยืนยัน
  Serial.println("ACK");
}

void loop() {
  // อ่านทีละตัวอักษรจนเจอ \n แล้วค่อยประมวลผล
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;   // ข้าม CR
    if (c == '\n') {           // จบบรรทัด
      if (line.length() > 0) {
        processLine(line);
        line = "";             // เคลียร์บัฟเฟอร์
      }
    } else {
      line += c;               // สะสม
      if (line.length() > 128) line.remove(0); // กันล้น
    }
  }
}
