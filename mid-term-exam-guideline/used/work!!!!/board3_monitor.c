/*
 * ========================= Board 3: Monitor & Control =========================
 * หน้าที่หลัก:
 *   - รับสถานะจากบอร์ด 1: "PILLS:xxx", "PROGRESS:yyy", (เสริม) "WATER:HI"
 *   - แสดงผลบน 7-segment 4 หลัก (multiplex 1kHz): ซ้าย = pills%100, ขวา = progress%100
 *   - แสดงสถานะด้วย LED 4 สี PB2..PB5 (G,Y,R,B):
 *       * G (<50%), Y (50–89%), R (90–99%), B กระพริบ (โค้ดนี้ตั้งให้กระพริบคงที่ ~4Hz)
 *   - โหมดล้าง (เมื่อ progress≥100): เปิด solenoid 3s, packaging 5s, แสดง "CLEn" แล้วส่ง "RESET\n" ไปบอร์ด 1
 *   - ถ้าไม่ได้รับข้อมูลใหม่ >30s → โหมด TIMEOUT: แสดง "----" + ไฟแดงค้าง จนกว่าจะมีข้อมูลอีกครั้ง
 *
 * ฮาร์ดแวร์/การต่อสาย:
 *   - 7-seg (active-high) : a..f = PC0..PC5, g = PB0
 *   - Digit select        : PD4..PD7 (เลือกหลัก 0..3)
 *   - LEDs (status)       : PB2=G, PB3=Y, PB4=R, PB5=B
 *   - Packaging (LED/Relay): PD2, Solenoid valve: PD3
 *
 * โมดูลเวลา/สื่อสาร:
 *   - Timer0 CTC 1ms → g_ms ใช้จับเวลา/timeout
 *   - Timer2 CTC 1kHz → multiplex 7-seg + กระพริบไฟฟ้า (blue) ~4Hz
 *   - UART 9600 8E1 → รับ/ส่งระหว่างบอร์ด 1
 */

#define F_CPU 16000000UL
#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

// -------- segment & digit mapping --------
#define SEG_PORT_C  PORTC
#define SEG_DDR_C   DDRC
#define SEG_MASK_CF 0x3F           // PC0..PC5 → segment a..f
#define SEG_PORT_G  PORTB
#define SEG_DDR_G   DDRB
#define SEG_BIT_G   PB0            // PB0 → segment g

#define DIGIT_PORT  PORTD
#define DIGIT_DDR   DDRD
#define DIGIT_MASK  ((1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7)) // 4 หลัก

// LEDs: PB2..PB5
#define LED_PORT  PORTB
#define LED_DDR   DDRB
#define LED_G     PB2
#define LED_Y     PB3
#define LED_R     PB4
#define LED_B     PB5

// Outputs (การล้าง/บรรจุ)
#define PACK_PORT PORTD
#define PACK_DDR  DDRD
#define PACK_PIN  PD2   // packaging 5s
#define SOL_PORT  PORTD
#define SOL_DDR   DDRD
#define SOL_PIN   PD3   // solenoid 3s

// -------- UART (9600 8E1) --------
static void usart0_init(void)
{
    uint16_t ubrr = (F_CPU / (16UL * 9600UL)) - 1;
    UBRR0H = (ubrr >> 8) & 0xFF;
    UBRR0L =  ubrr & 0xFF;
    UCSR0C = (1 << UPM01) | (1 << UCSZ01) | (1 << UCSZ00);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
}
static void usart0_send_byte(uint8_t d){ while (!(UCSR0A & (1 << UDRE0))); UDR0 = d; }
static void usart0_send_string(const char *s){ while (*s) usart0_send_byte((uint8_t)*s++); }

// -------- Timer0 → 1ms (g_ms) --------
static volatile unsigned long g_ms = 0;
static void timer0_init_1ms(void)
{
    TCCR0A = (1 << WGM01);                 // CTC
    TCCR0B = (1 << CS01) | (1 << CS00);    // /64
    OCR0A  = 249;                          // 1kHz
    TIMSK0 = (1 << OCIE0A);
}
ISR(TIMER0_COMPA_vect) { g_ms++; }

// -------- Timer2 → 1kHz (multiplex + blink) --------
static volatile uint8_t seg_frame[4] = {0, 0, 0, 0}; // โค้ด segment สำหรับ 4 หลัก
static volatile uint8_t cur_digit = 0;               // หลักปัจจุบัน (0..3)
static void timer2_init_1kHz(void)
{
    TCCR2A = (1 << WGM21);  // CTC
    TCCR2B = (1 << CS22);   // /64
    OCR2A  = 249;           // 1kHz
    TIMSK2 = (1 << OCIE2A);
}
ISR(TIMER2_COMPA_vect)
{
    // ปิดทุกหลักก่อน
    DIGIT_PORT &= ~DIGIT_MASK;

    // set segments สำหรับหลักปัจจุบัน
    uint8_t code = seg_frame[cur_digit & 3];
    SEG_PORT_C = (SEG_PORT_C & ~SEG_MASK_CF) | (code & 0x3F); // a..f
    if (code & 0x40) SEG_PORT_G |=  (1 << SEG_BIT_G);         // g
    else              SEG_PORT_G &= ~(1 << SEG_BIT_G);

    // เปิดหลักที่เลือก
    switch (cur_digit & 3){
      case 0: DIGIT_PORT |= (1 << PD4); break;
      case 1: DIGIT_PORT |= (1 << PD5); break;
      case 2: DIGIT_PORT |= (1 << PD6); break;
      case 3: DIGIT_PORT |= (1 << PD7); break;
    }
    cur_digit++;

    // กระพริบไฟน้ำเงิน ~4Hz (ในโค้ดนี้ “กระพริบตลอด”) — ถ้าต้องการให้กระพริบเฉพาะเมื่อ progress>=100
    // ให้เพิ่มเงื่อนไขตรวจ progress ก่อน toggle
    static uint16_t cnt = 0;
    if (cnt++ >= 125){ LED_PORT ^= (1 << LED_B); cnt = 0; }
}

// -------- 7-segment encode --------
static const uint8_t dt[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66,
    0x6D, 0x7D, 0x07, 0x7F, 0x6F
};
static uint8_t encC(char c){
    switch (c){
      case 'C': return 0x39;
      case 'L': return 0x38;
      case 'E': return 0x79;
      case 'n': return 0x54;
      case '-': return 0x40;
      default:  return 0x00;
    }
}
static void compose_run(uint16_t pills, uint8_t prog){
    // ซ้าย = pills%100, ขวา = prog%100
    uint8_t L = pills % 100, R = prog % 100;
    seg_frame[0] = dt[L / 10];
    seg_frame[1] = dt[L % 10];
    seg_frame[2] = dt[R / 10];
    seg_frame[3] = dt[R % 10];
}
static void compose_clean(void){ seg_frame[0]=encC('C'); seg_frame[1]=encC('L'); seg_frame[2]=encC('E'); seg_frame[3]=encC('n'); }
static void compose_timeout(void){ seg_frame[0]=seg_frame[1]=seg_frame[2]=seg_frame[3]=encC('-'); }

// -------- สถานะ/โหมด --------
static volatile uint16_t pills    = 0;
static volatile uint8_t  progress = 0;
static unsigned long last_rx_ms   = 0;

typedef enum { M_RUN, M_CLEAN, M_TIMEOUT } mode_t;
static mode_t       mode      = M_RUN;
static unsigned long clean_t0 = 0;

// -------- รับ Serial line (parse เมื่อจบ '\n') --------
#define RXN 32
static char rxbuf[RXN];
static uint8_t rxi = 0;
static void poll_serial(void)
{
    while (UCSR0A & (1 << RXC0))
    {
        char c = UDR0;
        last_rx_ms = g_ms;
        if (c == '\r') continue;
        if (c == '\n'){
            rxbuf[rxi] = '\0';
            if (!strncmp(rxbuf, "PILLS:", 6)){
                int v = atoi(rxbuf + 6);
                if (v >= 0 && v <= 999) pills = (uint16_t)v;
            } else if (!strncmp(rxbuf, "PROGRESS:", 9)){
                int v = atoi(rxbuf + 9);
                if (v >= 0 && v <= 100) progress = (uint8_t)v;
            } else if (!strcmp(rxbuf, "WATER:HI")){
                // "น้ำสูง" จากบอร์ด 1 → เปิดโซลินอยด์ 3s (แสดงการตอบสนองด้านความปลอดภัย)
                SOL_PORT |= (1 << SOL_PIN);
                unsigned long t0 = g_ms;
                while ((g_ms - t0) < 3000UL){ _delay_ms(5); }
                SOL_PORT &= ~(1 << SOL_PIN);
            }
            rxi = 0; // clear buffer
        } else {
            //ถ้ายังไม่เต็ม (rxi < RXN-1) ให้เก็บอักขระ c ลง rxbuf[rxi] แล้วเพิ่ม rxi++
            if (rxi < RXN - 1) rxbuf[rxi++] = c;
            else rxi = 0;
        }
    }
}

// -------- LED helper (G/Y/R คุมตรง ๆ) --------
static void leds(bool g, bool y, bool r){
    if (g) LED_PORT |=  (1 << LED_G); else LED_PORT &= ~(1 << LED_G);
    if (y) LED_PORT |=  (1 << LED_Y); else LED_PORT &= ~(1 << LED_Y);
    if (r) LED_PORT |=  (1 << LED_R); else LED_PORT &= ~(1 << LED_R);
    // Blue LED คุมโดย Timer2 ISR (กระพริบ)
}

// -------- main() --------
int main(void)
{
    // ตั้งทิศทางขา I/O
    SEG_DDR_C  |= SEG_MASK_CF;
    SEG_DDR_G  |= (1 << SEG_BIT_G);
    DIGIT_DDR  |= DIGIT_MASK;
    LED_DDR    |= (1 << LED_G) | (1 << LED_Y) | (1 << LED_R) | (1 << LED_B);
    PACK_DDR   |= (1 << PACK_PIN);
    SOL_DDR    |= (1 << SOL_PIN);

    // เคลียร์เอาต์พุตเริ่มต้น
    SEG_PORT_C &= ~SEG_MASK_CF;
    SEG_PORT_G &= ~(1 << SEG_BIT_G);
    DIGIT_PORT &= ~DIGIT_MASK;
    LED_PORT   &= ~((1 << LED_G) | (1 << LED_Y) | (1 << LED_R) | (1 << LED_B));
    PACK_PORT  &= ~(1 << PACK_PIN);
    SOL_PORT   &= ~(1 << SOL_PIN);

    // โมดูลเวลา + UART
    timer0_init_1ms();
    timer2_init_1kHz();
    usart0_init();
    sei();

    last_rx_ms = g_ms;
    mode = M_RUN;
    compose_run(pills, progress);

    for (;;)
    {
        // อ่าน serial + parse
        poll_serial();

        // ถ้าไม่มีข้อมูล >30s → TIMEOUT
        if ((g_ms - last_rx_ms) > 30000UL) mode = M_TIMEOUT;

        if (mode == M_TIMEOUT)
        {
            compose_timeout();
            leds(false, false, true); // แดงค้าง
            // ถ้าเริ่มได้รับข้อมูลกลับมา → กลับไป RUN
            if ((g_ms - last_rx_ms) <= 30000UL) mode = M_RUN;
        }
        else if (mode == M_CLEAN)
        {
            unsigned long dt = g_ms - clean_t0;

            // ลำดับล้าง: 0..3s เปิด solenoid, 0..5s เปิด packaging
            if (dt < 3000UL) SOL_PORT |= (1 << SOL_PIN);
            else             SOL_PORT &= ~(1 << SOL_PIN);

            if (dt < 5000UL) PACK_PORT |= (1 << PACK_PIN);
            else             PACK_PORT &= ~(1 << PACK_PIN);

            compose_clean();
            leds(false, false, false);

            // จบล้าง → รีเซ็ตค่าหน้าจอ, กลับ RUN, ไฟเขียว
            if (dt >= 5000UL)
            {
                mode = M_RUN;
                pills = 0;
                progress = 0;
                PACK_PORT &= ~(1 << PACK_PIN);
                SOL_PORT  &= ~(1 << SOL_PIN);
                leds(true, false, false);
                compose_run(pills, progress);
            }
        }
        else // M_RUN
        {
            compose_run(pills, progress);

            // ไฟสถานะตามระดับ progress
            if      (progress < 50)  leds(true,  false, false);
            else if (progress < 90)  leds(false, true,  false);
            else if (progress < 100) leds(false, false, true);

            // ถึง 100% → เริ่มโหมดล้าง + แจ้งรีเซ็ตบอร์ด 1
            if (progress >= 100)
            {
                mode = M_CLEAN;
                clean_t0 = g_ms;
                usart0_send_string("RESET\n");
            }
        }

        _delay_ms(10); // พัก CPU เล็กน้อย (งานแสดงผลหลักทำใน ISR แล้ว)
    }
    return 0;
}
