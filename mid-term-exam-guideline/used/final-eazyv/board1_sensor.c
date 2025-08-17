#include <LiquidCrystal.h>
#include <Arduino.h>
#include <avr/wdt.h>

// กำหนดพิน
const int TRIG_PIN  = 8;   // ขาส่งคลื่นอัลตร้าโซนิก
const int ECHO_PIN  = 9;   // ขารับคลื่นสะท้อน
const int WATER_CH  = A1;  // ช่อง ADC อ่านระดับน้ำ
LiquidCrystal lcd(12, 11, 5, 4, 3, 2); // RS, E, D4, D5, D6, D7

// ตัวแปรสถานะ
volatile uint32_t ms = 0;      // ตัวนับมิลลิวินาที
unsigned int pill_count = 0;   // จำนวนเม็ดยาที่นับได้
unsigned int target_count = 0; // เป้าหมายเม็ดยา
unsigned int progress = 0;     // เปอร์เซ็นต์ความคืบหน้า
unsigned int water_percent = 0;// เปอร์เซ็นต์ระดับน้ำ

// สร้างอินเตอร์รัพต์ 1 ms
ISR(TIMER1_COMPA_vect){
  ms++;
}

// ตั้ง Timer1 ให้ข้ามอินเตอร์รัพต์ทุก 1 ms
void setup_timer1(){
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A  = 249;                     // ค่าที่ทำให้ข้ามอินเตอร์รัพต์ทุก 1ms (16MHz/64/1000 - 1)
  TCCR1B |= (1<<WGM12);             // CTC mode
  TCCR1B |= (1<<CS11) | (1<<CS10);  // prescaler = 64
  TIMSK1 |= (1<<OCIE1A);            // enable compare interrupt
  interrupts();
}

// อ่าน ADC ช่องที่กำหนด
uint16_t readADC(byte ch){
  return analogRead(ch);
}

// วัดระยะจาก HC‑SR04 (cm)
float measure_distance(){
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // timeout 30ms
  if (duration == 0) return 999.0; // ไม่มี echo
  return duration * 0.034f / 2.0f;
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  lcd.begin(16, 2);
  setup_timer1();
  wdt_enable(WDTO_2S);    // Watchdog 2s
  Serial.begin(9600);     // UART 9600 bps, 8N1 (อาจต้องปรับเป็น 8E1 ในโปรเจ็กต์จริง)
  lcd.print("Board1 Ready");
}

void loop() {
  // ป้อน watchdog
  wdt_reset();

  // อ่านระดับน้ำ
  uint16_t raw = readADC(WATER_CH);
  water_percent = (uint32_t)raw * 100UL / 1023UL;

  // วัดและนับเม็ดยาด้วย HC‑SR04 (ถ้าระยะ < 10 cm สามครั้งติด)
  float d1 = measure_distance();
  float d2 = measure_distance();
  float d3 = measure_distance();
  if (d1 < 10 && d2 < 10 && d3 < 10) {
    pill_count++;
  }

  // คำนวณความคืบหน้า
  if (target_count > 0) {
    progress = ((uint32_t)pill_count * 100UL) / target_count;
    if (progress > 100) progress = 100;
  } else {
    progress = 0;
  }

  // แสดงผลบน LCD
  lcd.setCursor(0,0);
  lcd.print("P:");
  lcd.print(pill_count);
  lcd.print(" T:");
  lcd.print(target_count);
  lcd.print(" ");

  lcd.setCursor(0,1);
  lcd.print("Prg:");
  lcd.print(progress);
  lcd.print("% W:");
  lcd.print(water_percent);
  lcd.print("   ");

  // ส่งค่าทาง serial ไปบอร์ด 3 (แสดงตัวอย่าง format)
  char buf[40];
  sprintf(buf, "PILLS:%03u\nPROGRESS:%03u\n", pill_count, progress);
  Serial.print(buf);

  delay(200);
}
