/*
 * ========================= Board 2: Input Control =========================
 * HW wiring:
 *   Keypad (ADC ladder) : A5 (PC5)
 *   LEDs                 : GREEN=D4, YELLOW=D5, RED=D6
 *   LCD 16x2             : RS=D8, E=D9, D4=D10, D5=D11, D6=D12, D7=D13
 * Serial link (USART0, 9600 8E1):
 *   ส่ง "TARGET:xxx\n" ไปยังบอร์ด 1 เมื่อกด '#'
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <LiquidCrystal.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

/* ---------------- LCD ---------------- */
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);

/* ---------------- LEDs ---------------- */
#define LED_GREEN  4
#define LED_YELLOW 5
#define LED_RED    6

/* ---------------- Globals ---------------- */
volatile unsigned long g_ms = 0;     // 1ms tick
static unsigned long idle_start_ms = 0;
static volatile bool wdt_timeout = false;

static unsigned int target_value = 0; // 0..999

/* ---------------- Timer1 1ms ---------------- */
static void timer1_init_1ms(void){
  TCCR1A=0; TCCR1B=(1<<WGM12)|(1<<CS11)|(1<<CS10); OCR1A=249; TIMSK1=(1<<OCIE1A);
}
ISR(TIMER1_COMPA_vect){ g_ms++; }

/* ---------------- ADC ---------------- */
static void adc_init(void){
  ADMUX = (1<<REFS0); // AVcc
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
  uint16_t ubrr=(F_CPU/(16UL*9600UL))-1;
  UBRR0H=(ubrr>>8)&0xFF; UBRR0L=ubrr&0xFF;
  UCSR0C=(1<<UPM01)|(1<<UCSZ01)|(1<<UCSZ00);
  UCSR0B=(1<<TXEN0);
}
static void usart0_send_byte(uint8_t d){ while(!(UCSR0A&(1<<UDRE0))); UDR0=d; }
static void usart0_send_string(const char*s){ while(*s) usart0_send_byte((uint8_t)*s++); }

/* ---------------- LED helpers ---------------- */
static void leds_set(bool g, bool y, bool r){
  digitalWrite(LED_GREEN,  g?HIGH:LOW);
  digitalWrite(LED_YELLOW, y?HIGH:LOW);
  digitalWrite(LED_RED,    r?HIGH:LOW);
}

/* ---------------- Keypad decode (ตามช่วงที่เรียน) ---------------- */
static char decode_key(uint16_t adc){
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
/* อ่านปุ่มพร้อม debounce 50ms + ตรวจค่านิ่ง */
static char keypad_getkey(void){
  static char last=0;
  uint16_t a1=adc_read(5); _delay_ms(50); uint16_t a2=adc_read(5);
  if ( (a1>33) && (abs((int)a2-(int)a1)<=10) ){
    char k=decode_key(a2);
    if (k!=0 && k!=last){ last=k; return k; }
    if (k==0) last=0;
  } else {
    last=0;
  }
  return 0;
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

/* ---------------- Main ---------------- */
int main(void){
  // IO
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW,OUTPUT);
  pinMode(LED_RED,   OUTPUT);
  leds_set(true,false,false);

  // modules
  lcd.begin(16,2); lcd.clear(); lcd.print("Board2 Ready");
  _delay_ms(600); lcd.clear();
  timer1_init_1ms();
  adc_init();
  usart0_init();

  // WDT 2s
  MCUSR &= ~(1<<WDRF);
  wdt_enable(WDTO_2S);

  sei();

  target_value=0; lcd_update();
  idle_start_ms=g_ms;

  for(;;){
    if (!wdt_timeout) wdt_reset();

    // 30s ไม่มีการกด → กลับ idle และปล่อยให้ WDT รีบูต
    if ((g_ms - idle_start_ms) > 30000UL){
      leds_set(true,false,false);
      wdt_timeout = true; // จะไม่รีเซ็ต WDT อีก → รีบูตภายใน ~2s
    }

    char k = keypad_getkey();
    if (k){
      idle_start_ms = g_ms; // มีการกด
      leds_set(false,true,false);

      if (k>='0' && k<='9'){
        unsigned newv = target_value*10 + (k-'0');
        if (newv<=999) target_value=newv;
        else { // เกิน 999 → blink แดงสั้น ๆ
          leds_set(false,false,true); _delay_ms(150); leds_set(false,true,false);
        }
        lcd_update();
      } else if (k=='*'){
        target_value=0; lcd_update();
      } else if (k=='#'){
        send_target(target_value);
        leds_set(true,false,false);   // ส่งแล้วกลับพร้อมรับใหม่
        lcd_update();
      }
    }
  }
  return 0;
}
