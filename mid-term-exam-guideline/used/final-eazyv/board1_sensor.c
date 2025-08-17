/*
 * ========================= Board 1: Sensor & Display =========================
 * HW wiring:
 *   Ultrasonic HC-SR04 : TRIG=D2 (PD2), ECHO=D3 (PD3)
 *   Water level (ADC)  : A1 (PC1)
 *   LCD 16x2 (LiquidCrystal): RS=D8, E=D9, D4=D4, D5=D5, D6=D6, D7=D7
 *   LED Timeout        : D13 (PB5)
 *
 * Serial link (USART0, 9600 8E1):
 *   RX/TX = D0/D1 (อย่าใช้กับเซ็นเซอร์)
 *   Receive  : "TARGET:xxx\n"
 *   Transmit : "PILLS:xxx\n" and "PROGRESS:yyy\n"
 *   (Added)  : "WATER:HI\n" เมื่อระดับน้ำสูงต่อเนื่อง (กันเด้ง)
 *
 * Safety:
 *   - น้ำสูง ≥ 80% → หยุดนับเม็ดยาชั่วคราว และส่ง WATER:HI ไปบอร์ด 3
 *   - comm timeout > 30s → แสดง Timeout และปล่อย WDT รีบูต (2s)
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>      // atoi
#include <LiquidCrystal.h>  // ใช้ LCD แบบที่เรียน

/* ---------------- Pins ---------------- */
#define TRIG_PIN   2      // D2
#define ECHO_PIN   3      // D3
#define LED_TIMEOUT 13    // D13 (ไฟสถานะ timeout)

#define WATER_ADC_CH 1    // A1

/* ---------------- LCD (ง่ายแบบที่เรียน) ---------------- */
LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // RS=8, E=9, D4=4, D5=5, D6=6, D7=7

/* ---------------- Globals ---------------- */
volatile unsigned long g_ms = 0;        // millis แบบง่ายจาก Timer1 (1ms)
static unsigned long last_serial_ms = 0;
static unsigned long last_send_ms   = 0;
static unsigned long lockout_until_ms = 0;

static volatile unsigned int pill_count = 0;      // 0..999
static volatile unsigned int target_count = 0;    // 0..999
static volatile unsigned int progress_percent = 0; // 0..100

static bool have_target = false;
static bool wdt_timeout = false;

/* น้ำสูง-ต่ำ (มี hysteresis กันเด้ง) */
#define WATER_HIGH_PCT 80
#define WATER_LOW_PCT  75
static volatile unsigned int water_percent = 0;
static bool  water_lock = false;   // true = หยุดนับเม็ดยาชั่วคราว
static bool  water_hi_sent = false;// ส่ง WATER:HI ไปแล้วหนึ่งครั้ง/เหตุการณ์
static uint8_t water_hi_cnt = 0;   // ฟิลเตอร์ค่าคงที่เล็กน้อย

/* RX buffer */
#define RX_BUF_SIZE 32
static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_len = 0;

/* ---------------- Timer1 → 1ms tick ---------------- */
static void timer1_init_1ms(void) {
  // CTC mode, prescale 64: 16MHz/64 = 250kHz → OCR1A=249 → 1kHz = 1ms
  TCCR1A = 0;
  TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10);
  OCR1A  = 249;
  TIMSK1 = (1<<OCIE1A);
}
ISR(TIMER1_COMPA_vect) { g_ms++; }

/* ---------------- USART0 9600 8E1 ---------------- */
static void usart0_init(void) {
  uint16_t ubrr = (F_CPU/(16UL*9600UL)) - 1; // U2X=0
  UBRR0H = (ubrr>>8)&0xFF; UBRR0L = ubrr&0xFF;
  UCSR0C = (1<<UPM01) | (1<<UCSZ01) | (1<<UCSZ00); // even parity, 8 data, 1 stop
  UCSR0B = (1<<RXEN0) | (1<<TXEN0);
}
static void usart0_send_byte(uint8_t d){ while(!(UCSR0A&(1<<UDRE0))); UDR0=d; }
static void usart0_send_string(const char *s){ while(*s) usart0_send_byte((uint8_t)*s++); }

/* ---------------- Ultrasonic (แบบ pulseIn ที่เรียน) ---------------- */
static float measure_distance_cm() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // timeout 30ms
  if (duration == 0) return 999.0f; // no echo
  return (duration * 0.034f) / 2.0f;
}

/* ---------------- ADC water (A1) → % ---------------- */
static void adc_init(void){
  ADMUX = (1<<REFS0) | (WATER_ADC_CH & 0x0F); // Vref=AVcc, ch=A1
  ADCSRA = (1<<ADEN) | (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // presc=128
}
static uint16_t adc_read(uint8_t ch){
  ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
  ADCSRA |= (1<<ADSC);
  while (ADCSRA & (1<<ADSC));
  return ADC;
}

/* ---------------- Progress calc ---------------- */
static void update_progress(void){
  if (!have_target || target_count==0){ progress_percent=0; return; }
  unsigned long p = (unsigned long)pill_count * 100UL / target_count;
  if (p>100) p=100;
  progress_percent = (unsigned int)p;
}

/* ---------------- LCD update (แบบง่าย) ---------------- */
static void lcd_show_status(void){
  lcd.setCursor(0,0);
  // "P:xxx T:yyy"
  char line0[17];
  snprintf(line0, sizeof(line0),"P:%3u T:%3u   ", pill_count, target_count);
  lcd.print(line0);
  lcd.setCursor(0,1);
  // ปกติ: "Prg:ppp% W:ww%"
  char line1[17];
  snprintf(line1, sizeof(line1),"Prg:%3u%% W:%2u%%", progress_percent, water_percent);
  lcd.print(line1);
}

/* ---------------- Send status to Board3 ---------------- */
static void send_status_to_board3(void){
  char buf[20];
  snprintf(buf,sizeof(buf),"PILLS:%03u\n", pill_count);
  usart0_send_string(buf);
  snprintf(buf,sizeof(buf),"PROGRESS:%03u\n", progress_percent);
  usart0_send_string(buf);
  last_send_ms = g_ms;
}

/* ---------------- RX line handling ---------------- */
static void handle_received_line(void){
  rx_buf[rx_len] = '\0';
  if (strncmp(rx_buf,"TARGET:",7)==0){
    int v = atoi(rx_buf+7);
    if (v>=0 && v<=999){ target_count=(unsigned)v; have_target=true; }
  } else if (strcmp(rx_buf,"RESET")==0){
    pill_count=0; progress_percent=0;
  }
  rx_len=0;
}
static void poll_usart_rx(void){
  while (UCSR0A&(1<<RXC0)) {
    char c = UDR0; last_serial_ms = g_ms;
    if (c=='\r') continue;
    if (c=='\n') handle_received_line();
    else {
      if (rx_len < RX_BUF_SIZE-1) rx_buf[rx_len++]=c; else rx_len=0;
    }
  }
}

/* ---------------- Main ---------------- */
int main(void){
  // GPIO
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_TIMEOUT, OUTPUT);
  digitalWrite(LED_TIMEOUT, LOW);

  // modules
  timer1_init_1ms();
  usart0_init();
  adc_init();
  lcd.begin(16,2);
  lcd.clear();
  lcd.print("Board1 Ready");
  _delay_ms(800); lcd.clear();

  // Watchdog 2s
  MCUSR &= ~(1<<WDRF);
  wdt_enable(WDTO_2S);

  sei();

  last_serial_ms = last_send_ms = g_ms;

  for(;;){
    // WDT keepalive (ถ้ายังไม่ตั้งให้ปล่อยรีเซ็ต)
    if (!wdt_timeout) wdt_reset();

    // รับคำสั่ง
    poll_usart_rx();

    // อ่านน้ำ → % (0..100)
    uint16_t raw = adc_read(WATER_ADC_CH);
    water_percent = (unsigned int)((unsigned long)raw * 100UL / 1023UL);

    // น้ำสูง → ฟิลเตอร์กันเด้ง + ตั้ง water_lock + แจ้งบอร์ด3 หนึ่งครั้ง
    if (water_percent >= WATER_HIGH_PCT) {
      if (water_hi_cnt < 10) water_hi_cnt++;     // ~10 × loop สั้น ๆ
      if (water_hi_cnt == 10) {
        water_lock = true;                       // หยุดนับเม็ดยาชั่วคราว
        if (!water_hi_sent) {                    // แจ้งบอร์ด 3 ครั้งเดียว/เหตุการณ์
          usart0_send_string("WATER:HI\n");
          water_hi_sent = true;
        }
      }
    } else if (water_percent <= WATER_LOW_PCT) {
      water_hi_cnt = 0;
      water_lock   = false;                      // กลับมานับได้
      water_hi_sent= false;                      // เคลียร์ trigger
    }

    // Timeout สื่อสาร >30s → แจ้งบน LCD + กระพริบ LED13 + ไม่ป้อน WDT
    if ((g_ms - last_serial_ms) > 30000UL) {
      lcd.setCursor(0,1); lcd.print("Timeout!        ");
      digitalWrite(LED_TIMEOUT, ((g_ms/500)%2)? HIGH:LOW);
      wdt_timeout = true;   // หยุด wdt_reset() → MCU จะรีเซ็ตเองภายใน ~2s
    } else {
      digitalWrite(LED_TIMEOUT, LOW);
    }

    // นับเม็ดยา (อ่าน HC-SR04 3 ครั้งติดกัน) — ยกเว้นช่วง water_lock
    if (!water_lock && have_target){
      float d1 = measure_distance_cm();
      float d2 = measure_distance_cm();
      float d3 = measure_distance_cm();
      if (d1<10 && d2<10 && d3<10){
        if (g_ms > lockout_until_ms){
          if (pill_count<999) pill_count++;
          lockout_until_ms = g_ms + 150; // กันหนึ่งเม็ดโดนนับซ้ำ
          update_progress();
        }
      }
    }

    // ส่งสถานะไปบอร์ด 3 ทุก ~200ms (เมื่อมี target)
    if (have_target && (g_ms - last_send_ms) >= 200UL){
      update_progress();
      send_status_to_board3();
    }

    // อัปเดต LCD
    lcd_show_status();

    _delay_ms(50);
  }
  return 0;
}
