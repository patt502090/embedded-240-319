/*
 * ========================= Board 3: Monitor & Control =========================
 * HW wiring:
 *   7-seg (active-high): a..f = PC0..PC5, g = PB0
 *   Digit select        : PD4..PD7
 *   LEDs (status)       : PB2=G, PB3=Y, PB4=R, PB5=B
 *   Packaging (LED/Relay): PD2
 *   Solenoid valve      : PD3
 * Serial link (USART0, 9600 8E1):
 *   RX: "PILLS:xxx", "PROGRESS:yyy", "WATER:HI"
 *   TX: "RESET\n" เมื่อจบการล้างหลังครบ 100%
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* -------- segment & digit mapping -------- */
#define SEG_PORT_C PORTC
#define SEG_DDR_C DDRC
#define SEG_MASK_CF 0x3F // PC0..PC5 : a..f
#define SEG_PORT_G PORTB
#define SEG_DDR_G DDRB
#define SEG_BIT_G PB0

#define DIGIT_PORT PORTD
#define DIGIT_DDR DDRD
#define DIGIT_MASK ((1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7))

#define LED_PORT PORTB
#define LED_DDR DDRB
#define LED_G PB2
#define LED_Y PB3
#define LED_R PB4
#define LED_B PB5

#define PACK_PORT PORTD
#define PACK_DDR DDRD
#define PACK_PIN PD2
#define SOL_PORT PORTD
#define SOL_DDR DDRD
#define SOL_PIN PD3

/* -------- UART -------- */
static void usart0_init(void)
{
    uint16_t ubrr = (F_CPU / (16UL * 9600UL)) - 1;
    UBRR0H = (ubrr >> 8) & 0xFF;
    UBRR0L = ubrr & 0xFF;
    UCSR0C = (1 << UPM01) | (1 << UCSZ01) | (1 << UCSZ00); // 8E1
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
}
static void usart0_send_byte(uint8_t d)
{
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    UDR0 = d;
}
static void usart0_send_string(const char *s)
{
    while (*s)
        usart0_send_byte((uint8_t)*s++);
}

/* -------- time base -------- */
static volatile unsigned long g_ms = 0;
static void timer0_init_1ms(void)
{
    // CTC, /64, OCR0A=249 → 1kHz
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 249;
    TIMSK0 = (1 << OCIE0A);
}
ISR(TIMER0_COMPA_vect) { g_ms++; }

/* -------- multiplex 7-seg @1kHz -------- */
static volatile uint8_t seg_frame[4] = {0, 0, 0, 0};
static volatile uint8_t cur_digit = 0;
static void timer2_init_1kHz(void)
{
    // CTC, /64, OCR2A=249 → 1kHz
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22);
    OCR2A = 249;
    TIMSK2 = (1 << OCIE2A);
}
ISR(TIMER2_COMPA_vect)
{
    // ปิดทุกหลัก
    DIGIT_PORT &= ~DIGIT_MASK;

    // set segments
    uint8_t code = seg_frame[cur_digit & 3];
    SEG_PORT_C = (SEG_PORT_C & ~SEG_MASK_CF) | (code & 0x3F);
    if (code & 0x40)
        SEG_PORT_G |= (1 << SEG_BIT_G);
    else
        SEG_PORT_G &= ~(1 << SEG_BIT_G);

    // เปิดหลัก
    switch (cur_digit & 3)
    {
    case 0:
        DIGIT_PORT |= (1 << PD4);
        break;
    case 1:
        DIGIT_PORT |= (1 << PD5);
        break;
    case 2:
        DIGIT_PORT |= (1 << PD6);
        break;
    case 3:
        DIGIT_PORT |= (1 << PD7);
        break;
    }
    cur_digit++;

    // กระพริบไฟน้ำเงินเมื่อ progress>=100 (toggle ~4Hz)
    static uint16_t cnt = 0;
    if (cnt++ >= 125)
    {
        LED_PORT ^= (1 << LED_B);
        cnt = 0;
    }
}

/* -------- encode tables -------- */
static const uint8_t dt[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};
static uint8_t encC(char c)
{
    switch (c)
    {
    case 'C':
        return 0x39;
    case 'L':
        return 0x38;
    case 'E':
        return 0x79;
    case 'n':
        return 0x54;
    case '-':
        return 0x40;
    default:
        return 0x00;
    }
}
static void compose_run(uint16_t pills, uint8_t prog)
{
    uint8_t L = pills % 100, R = prog % 100;
    seg_frame[0] = dt[L / 10];
    seg_frame[1] = dt[L % 10];
    seg_frame[2] = dt[R / 10];
    seg_frame[3] = dt[R % 10];
}
static void compose_clean(void)
{
    seg_frame[0] = encC('C');
    seg_frame[1] = encC('L');
    seg_frame[2] = encC('E');
    seg_frame[3] = encC('n');
}
static void compose_timeout(void)
{
    seg_frame[0] = seg_frame[1] = seg_frame[2] = seg_frame[3] = encC('-');
}

/* -------- state -------- */
static volatile uint16_t pills = 0;
static volatile uint8_t progress = 0;
static unsigned long last_rx_ms = 0;

typedef enum
{
    M_RUN,
    M_CLEAN,
    M_TIMEOUT
} mode_t;
static mode_t mode = M_RUN;
static unsigned long clean_t0 = 0;

/* RX parse */
#define RXN 32
static char rxbuf[RXN];
static uint8_t rxi = 0;
static void poll_serial(void)
{
    while (UCSR0A & (1 << RXC0))
    {
        char c = UDR0;
        last_rx_ms = g_ms;
        if (c == '\r')
            continue;
        if (c == '\n')
        {
            rxbuf[rxi] = '\0';
            if (!strncmp(rxbuf, "PILLS:", 6))
            {
                int v = atoi(rxbuf + 6);
                if (v >= 0 && v <= 999)
                    pills = (uint16_t)v;
            }
            else if (!strncmp(rxbuf, "PROGRESS:", 9))
            {
                int v = atoi(rxbuf + 9);
                if (v >= 0 && v <= 100)
                    progress = (uint8_t)v;
            }
            else if (!strcmp(rxbuf, "WATER:HI"))
            {
                // น้ำสูง → เปิดโซลินอยด์ 3 วินาที (ง่ายและชัด)
                SOL_PORT |= (1 << SOL_PIN);
                unsigned long t0 = g_ms;
                while ((g_ms - t0) < 3000UL)
                {
                    _delay_ms(5);
                }
                SOL_PORT &= ~(1 << SOL_PIN);
            }
            rxi = 0;
        }
        else
        {
            if (rxi < RXN - 1)
                rxbuf[rxi++] = c;
            else
                rxi = 0;
        }
    }
}

/* LEDs helper */
static void leds(bool g, bool y, bool r)
{
    if (g)
        LED_PORT |= (1 << LED_G);
    else
        LED_PORT &= ~(1 << LED_G);
    if (y)
        LED_PORT |= (1 << LED_Y);
    else
        LED_PORT &= ~(1 << LED_Y);
    if (r)
        LED_PORT |= (1 << LED_R);
    else
        LED_PORT &= ~(1 << LED_R);
    // Blue handled by timer2 blink
}

/* ---------------- Main ---------------- */
int main(void)
{
    // IO directions
    SEG_DDR_C |= SEG_MASK_CF;
    SEG_DDR_G |= (1 << SEG_BIT_G);
    DIGIT_DDR |= DIGIT_MASK;
    LED_DDR |= (1 << LED_G) | (1 << LED_Y) | (1 << LED_R) | (1 << LED_B);
    PACK_DDR |= (1 << PACK_PIN);
    SOL_DDR |= (1 << SOL_PIN);
    // init outputs
    SEG_PORT_C &= ~SEG_MASK_CF;
    SEG_PORT_G &= ~(1 << SEG_BIT_G);
    DIGIT_PORT &= ~DIGIT_MASK;
    LED_PORT &= ~((1 << LED_G) | (1 << LED_Y) | (1 << LED_R) | (1 << LED_B));
    PACK_PORT &= ~(1 << PACK_PIN);
    SOL_PORT &= ~(1 << SOL_PIN);

    // timers & uart
    timer0_init_1ms();
    timer2_init_1kHz();
    usart0_init();
    sei();

    last_rx_ms = g_ms;
    mode = M_RUN;
    compose_run(pills, progress);

    for (;;)
    {
        poll_serial();

        // Timeout comm > 30s → M_TIMEOUT
        if ((g_ms - last_rx_ms) > 30000UL)
            mode = M_TIMEOUT;

        if (mode == M_TIMEOUT)
        {
            compose_timeout();
            leds(false, false, true); // แดงค้าง
            if ((g_ms - last_rx_ms) <= 30000UL)
                mode = M_RUN; // มีข้อมูลใหม่ → กลับ RUN
        }
        else if (mode == M_CLEAN)
        {
            unsigned long dt = g_ms - clean_t0;
            // 0..3s เปิดโซลินอยด์ / 0..5s เปิด packaging
            if (dt < 3000UL)
                SOL_PORT |= (1 << SOL_PIN);
            else
                SOL_PORT &= ~(1 << SOL_PIN);
            if (dt < 5000UL)
                PACK_PORT |= (1 << PACK_PIN);
            else
                PACK_PORT &= ~(1 << PACK_PIN);
            compose_clean();
            leds(false, false, false);
            if (dt >= 5000UL)
            {
                mode = M_RUN;
                pills = 0;
                progress = 0;
                PACK_PORT &= ~(1 << PACK_PIN);
                SOL_PORT &= ~(1 << SOL_PIN);
                leds(true, false, false);
                compose_run(pills, progress);
            }
        }
        else
        { // M_RUN
            compose_run(pills, progress);
            if (progress < 50)
                leds(true, false, false);
            else if (progress < 90)
                leds(false, true, false);
            else if (progress < 100)
                leds(false, false, true);

            if (progress >= 100)
            {
                mode = M_CLEAN;
                clean_t0 = g_ms;
                usart0_send_string("RESET\n");
            }
        }

        _delay_ms(10);
    }
    return 0;
}
