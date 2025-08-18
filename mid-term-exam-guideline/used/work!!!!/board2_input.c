/*
 * ========================= Board 2: Input Control (UPDATED) =========================
 * หน้าที่หลัก:
 *   - รับค่า “Target pills (0–999)” ผ่าน Keypad แบบ ADC ladder ที่ A5
 *   - เปลี่ยนวิธีอ่านคีย์แพดเป็น “แนวที่เรียน”:
 *       • อ่าน ADC แบบตรวจความนิ่ง: abs(diff) < 6, delay 3 ms, ต้องนิ่งครบ 20 ครั้ง
 *       • จากนั้นถอดรหัสช่วงแรงดัน → ปุ่ม '0'..'9', '*' (ล้าง), '#' (ส่งค่า)
 *       • รอจนปล่อยปุ่ม (ADC <= 33) เพื่อกัน repeat ตอนกดค้าง
 *   - แสดงสถานะบน LCD 16x2 (LiquidCrystal)
 *   - ส่ง "TARGET:xxx\n" ไปยังบอร์ด 1 เมื่อกด '#'
 *   - Idle > 30s → ตั้งค่าให้ “ปล่อย WDT รีบูต” เพื่อความปลอดภัย
 *
 * ฮาร์ดแวร์/การต่อสาย:
 *   - Keypad ADC ladder : A5 (PC5)
 *   - LEDs              : GREEN=D4, YELLOW=D5, RED=D6
 *   - LCD 16x2          : RS=D8, E=D9, D4=D10, D5=D11, D6=D12, D7=D13
 *
 * โมดูลเวลา/ความปลอดภัย:
 *   - Timer1 CTC 1ms tick → g_ms ใช้ตรวจ idle timeout
 *   - Watchdog 2s → ถ้า idle นานเราตั้ง wdt_timeout=true เพื่อ “หยุดป้อน WDT” ให้รีบูตเอง
 *   - UART 9600 8E1 → ส่งค่า TARGET อย่างง่าย
 */

#define F_CPU 16000000UL
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <LiquidCrystal.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

/* ---------------- LCD (Library) ---------------- */
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

/* ---------------- LEDs ---------------- */
#define LED_GREEN  4  // พร้อม (idle)
#define LED_YELLOW 5  // กำลังตั้งค่า
#define LED_RED    6  // แจ้ง error (เช่น เกิน 999)

/* ---------------- Globals ---------------- */
volatile unsigned long g_ms = 0;     // 1ms tick
static unsigned long     idle_start_ms = 0;
static volatile bool     wdt_timeout   = false;
static unsigned int      target_value  = 0; // 0..999

/* ---------------- Timer1 1ms ---------------- */
static void timer1_init_1ms(void){
  TCCR1A = 0;
  TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10); // CTC, /64
  OCR1A  = 249;                                 // 1kHz → 1 ms
  TIMSK1 = (1<<OCIE1A);
}
ISR(TIMER1_COMPA_vect){ g_ms++; }

/* ---------------- ADC ---------------- */
static void adc_init(void){
  ADMUX  = (1<<REFS0); // Vref = AVcc
  ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // enable, /128
}
static uint16_t adc_read(uint8_t ch){
  ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
  ADCSRA |= (1<<ADSC);
  while (ADCSRA & (1<<ADSC));
  return ADC;
}

/* ---------------- USART0 9600 8E1 ---------------- */
static void usart0_init(void){
  uint16_t ubrr = (F_CPU/(16UL*9600UL)) - 1;
  UBRR0H = (ubrr>>8)&0xFF; UBRR0L = ubrr&0xFF;
  UCSR0C = (1<<UPM01)|(1<<UCSZ01)|(1<<UCSZ00); // even parity, 8 data, 1 stop
  UCSR0B = (1<<TXEN0); // ใช้ส่งอย่างเดียว
}
static void usart0_send_byte(uint8_t d){ while(!(UCSR0A&(1<<UDRE0))); UDR0=d; }
static void usart0_send_string(const char*s){ while(*s) usart0_send_byte((uint8_t)*s++); }

/* ---------------- LED helpers ---------------- */
static void leds_set(bool g, bool y, bool r){
  digitalWrite(LED_GREEN,  g?HIGH:LOW);
  digitalWrite(LED_YELLOW, y?HIGH:LOW);
  digitalWrite(LED_RED,    r?HIGH:LOW);
}

/* ---------------- Decode ปุ่มตามช่วง ADC (โพยที่เรียน) ---------------- */
static char decode_key_from_adc(uint16_t adc){
  if (adc>= 50 && adc<= 80)  return '1';
  if (adc>= 90 && adc<=120)  return '2';
  if (adc>=130 && adc<=160)  return '3';
  if (adc>=170 && adc<=200)  return '4';
  if (adc>=210 && adc<=240)  return '5';
  if (adc>=250 && adc<=280)  return '6';
  if (adc>=290 && adc<=320)  return '7';
  if (adc>=330 && adc<=360)  return '8';
  if (adc>=370 && adc<=400)  return '9';
  if (adc>=410 && adc<=440)  return '0';
  if (adc>=450 && adc<=480)  return '*';
  if (adc>=490 && adc<=520)  return '#';
  return 0;
}

/* ----------- Keypad แบบ “แนวที่เรียน”: ตรวจความนิ่งก่อนยอมรับ + รอปล่อยปุ่ม ----------- */
/*
 * ขั้นตอน:
 *   1) ถ้าอ่านเร็วครั้งแรกค่า ≤33 → ถือว่า “ไม่มีการกด” return 0 ทันที
 *   2) ถ้าค่า >33 → เข้าเฟส “ตรวจความนิ่ง”: อ่านซ้ำทุก 3 ms
 *      - ถ้า |a2-a1| < 6 → counter++  (นับความนิ่ง)
 *      - ถ้าไม่ → counter=0
 *      - ต้องนิ่งครบ 20 ครั้งถึงจะผ่าน
 *   3) ผ่านแล้ว → decode ช่วง ADC เป็นปุ่ม ('0'..'9','*','#')
 *   4) รอจนปล่อยปุ่ม (ADC <= 33) เพื่อกัน repeat เมื่อกดค้าง
 */
static char keypad_getkey_learned(void){
  uint16_t quick = adc_read(5);
  if (quick <= 33) return 0; // ยังไม่กด

  int counter = 0;
  int prev = quick;
  uint16_t val;

  do {
    val = adc_read(5);
    if ( (abs((int)val - (int)prev) < 6) && (val > 33) ) counter++;
    else counter = 0;
    _delay_ms(3);
    prev = val;
    if (!wdt_timeout) wdt_reset(); // กัน WDT ระหว่างหน่วง
  } while (counter < 20); // ต้องนิ่งครบ 20 ครั้ง

  char k = decode_key_from_adc(val);

  // รอจนปล่อยปุ่ม (ADC <= 33) เพื่อกัน auto-repeat
  do {
    val = adc_read(5);
    _delay_ms(2);
    if (!wdt_timeout) wdt_reset();
  } while (val > 33);

  return k; // ถอดรหัสสำเร็จ → คืนปุ่ม
}

/* ---------------- UI helpers ---------------- */
static void lcd_update(void){
  lcd.setCursor(0,0);
  char l0[17]; snprintf(l0,sizeof(l0),"Set Target:%3u",target_value);
  lcd.print(l0);
  lcd.setCursor(0,1);
  lcd.print("Press # to send ");
}
static void send_target(unsigned v){
  char buf[16]; snprintf(buf,sizeof(buf),"TARGET:%03u\n",v);
  usart0_send_string(buf);
}

/* ---------------- main() ---------------- */
int main(void){
  // LED
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW,OUTPUT);
  pinMode(LED_RED,   OUTPUT);
  leds_set(true,false,false); // เริ่มต้น: เขียว (พร้อม)

  // LCD + Timer + ADC + UART
  lcd.begin(16,2); lcd.clear(); lcd.print("Board2 Ready"); _delay_ms(600); lcd.clear();
  timer1_init_1ms();
  adc_init();
  usart0_init();

  // Watchdog 2s
  MCUSR &= ~(1<<WDRF);
  wdt_enable(WDTO_2S);

  sei();

  target_value=0; lcd_update();
  idle_start_ms=g_ms;

  for(;;){
    if (!wdt_timeout) wdt_reset();

    // ไม่มี interaction > 30s → ปล่อยให้ WDT รีบูตเอง (~2s)
    if ((g_ms - idle_start_ms) > 30000UL){
      leds_set(true,false,false);
      wdt_timeout = true;
    }

    // ใช้ตัวอ่านแบบ “แนวที่เรียน”
    char k = keypad_getkey_learned();
    if (k){
      idle_start_ms = g_ms;       // มีการกด → รีเซ็ต idle timer
      leds_set(false,true,false); // เหลือง: กำลังตั้งค่า

      if (k>='0' && k<='9'){
        unsigned newv = target_value*10 + (k-'0');
        if (newv<=999) target_value=newv;
        else { leds_set(false,false,true); _delay_ms(150); leds_set(false,true,false); }
        lcd_update();
      } else if (k=='*'){
        target_value=0; lcd_update(); // ล้างค่า
      } else if (k=='#'){
        send_target(target_value);    // ส่งไปบอร์ด 1
        leds_set(true,false,false);   // กลับสภาพพร้อม
        lcd_update();
      }
    }
  }
  return 0;
}
