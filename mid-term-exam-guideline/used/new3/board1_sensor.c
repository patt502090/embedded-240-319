/*
 * Board 1: Sensor & Display (Pill Counter + Communication Hub)
 *
 * ระบบนี้ทำหน้าที่นับเม็ดยาที่ไหลผ่านด้วยเซ็นเซอร์ ultrasonic (HC‑SR04)
 * แสดงผลจำนวนเม็ดยาและเปอร์เซ็นต์ความคืบหน้าบน LCD 16×2 และสื่อสารกับบอร์ดอื่น
 *
 * การสื่อสารใช้ USART0 ที่ความเร็ว 9600 bps, 8 data bits, Even parity, 1 stop bit (8E1)
 * บอร์ดนี้จะรับคำสั่งตั้งเป้าหมายจากบอร์ด 2 ในรูปแบบ "TARGET:xxx\n"
 * ส่งสถานะออกไปบอร์ด 3 เป็นข้อความสองบรรทัด: "PILLS:xxx\n" และ "PROGRESS:yyy\n"
 * เมื่อได้รับ "RESET\n" จากบอร์ด 3 จะรีเซ็ตตัวนับเม็ดยาและเริ่มรอบใหม่
 *
 * โจทย์กำหนดให้ต้องกรองสัญญาณ ultrasonic ด้วยการอ่านระยะ 3 ครั้งติดกัน
 * และมีช่วง lockout 150 ms กันนับซ้ำในกรณีเม็ดเดียวคร่อมลำแสง
 * หากไม่สื่อสารอะไรเลยเกิน 30 วินาที จะขึ้นสถานะ Timeout บน LCD และกระพริบ LED13
 */

#define F_CPU 16000000UL               /* กำหนดความถี่ CPU 16 MHz */
#include <avr/io.h>                    /* ไฟล์รีจิสเตอร์ของ AVR */
#include <avr/interrupt.h>             /* สำหรับ ISR */
#include <util/delay.h>                /* สำหรับ _delay_ms/us */
#include <string.h>                    /* สำหรับ strcmp(), strncmp() */
#include <stdbool.h>                   /* สำหรับประเภท bool */
#include <avr/wdt.h>                   /* สำหรับ Watchdog Timer */

/*------------------------------------------------------------------
 * พอร์ตและพินสำหรับอุปกรณ์ต่อพ่วง
 *----------------------------------------------------------------*/
#define TRIG_PIN  PD2                  /* ขา PD2 (digital pin 2) ต่อ TRIG ของ HC‑SR04 */
#define ECHO_PIN  PD3                  /* ขา PD3 (digital pin 3) ต่อ ECHO ของ HC‑SR04 */

#define LCD_RS    PB1                  /* RS ของ LCD ต่อ PB1 */
#define LCD_EN    PB2                  /* EN ของ LCD ต่อ PB2 */
#define LCD_D4    PC4                  /* D4 ของ LCD ต่อ PC4 */
#define LCD_D5    PC5                  /* D5 ของ LCD ต่อ PC5 */
#define LCD_D6    PC6                  /* D6 ของ LCD ต่อ PC6 */
#define LCD_D7    PC7                  /* D7 ของ LCD ต่อ PC7 */

#define LED_TIMEOUT PB5                /* ใช้ LED บนบอร์ด (D13) แสดง Timeout */

/*------------------------------------------------------------------
 * ตัวแปรสถานะร่วม
 *----------------------------------------------------------------*/
volatile unsigned long g_ms = 0;        /* ตัวนับเวลาเพิ่มทุก 1 ms (ใช้นับ timeout) */
volatile unsigned int pill_count = 0;    /* จำนวนเม็ดยาที่นับได้ (0–999) */
volatile unsigned int target_count = 0;  /* เป้าหมายเม็ดยาที่ต้องผลิต (0–999) */
volatile unsigned int progress_percent = 0; /* เปอร์เซ็นต์ความคืบหน้า (0–100) */

/* ตัวแปรสำหรับการจัดการสื่อสารและ timeout */
static unsigned long last_serial_ms = 0;    /* เวลาล่าสุดที่ได้รับหรือส่งข้อมูล USART */
static unsigned long last_send_ms = 0;      /* เวลาล่าสุดที่ส่งสถานะไปยังบอร์ด 3 */
static unsigned long lockout_until_ms = 0;  /* หมดเวลาช่วงกันนับซ้ำเม็ด (millis) */
static bool have_target = false;            /* มีการตั้งเป้าหมายแล้วหรือไม่ */

/*
 * ตัวแปรสำหรับ Watchdog Timer
 * หากเกิด timeout นานเกิน 30 วินาที จะตั้งค่า flag นี้ให้ true
 * แล้วหยุดป้อนอาหารให้ WDT (ไม่เรียก wdt_reset()) เพื่อให้ MCU รีเซ็ตตัวเองภายใน ~2 วินาที
 */
static bool wdt_timeout = false;

/* บัฟเฟอร์รับข้อความจาก USART (หนึ่งบรรทัด) */
#define RX_BUF_SIZE 32
static char rx_buf[RX_BUF_SIZE];
static uint8_t rx_len = 0;

/*------------------------------------------------------------------
 * ส่วน Timer1: สร้าง interrupt ทุก 1 ms ด้วย CTC mode
 * ใช้สำหรับนับเวลา (timeout, debounce)
 *----------------------------------------------------------------*/
static void timer1_init_1ms(void) {
    /* CTC mode: WGM12=1, prescaler=64 → f = F_CPU/64 = 250 kHz
     * OCR1A = (250 kHz / 1000 Hz) - 1 = 249 → interrupt ทุก 1 ms */
    TCCR1A = 0x00;
    TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10);
    OCR1A  = 249;
    TIMSK1 = (1<<OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
    g_ms++;
}

/*------------------------------------------------------------------
 * ส่วน USART0: ตั้งค่า 9600 bps 8E1 (Even parity)
 *----------------------------------------------------------------*/
static void usart0_init(void) {
    /* ตั้งบอดเรต 9600 (U2X0=0) → UBRR = 103 */
    uint16_t ubrr = (F_CPU / (16UL * 9600UL)) - 1;
    UBRR0H = (ubrr >> 8) & 0xFF;
    UBRR0L = ubrr & 0xFF;
    /* 8 data bits, even parity, 1 stop bit: UPM01=1, UPM00=0, USBS0=0, UCSZ01=1, UCSZ00=1 */
    UCSR0C = (1<<UPM01) | (1<<UCSZ01) | (1<<UCSZ00);
    /* เปิดภาครับ/ส่ง */
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
}

/* ส่งข้อมูล 1 ไบต์ออกทาง USART */
static void usart0_send_byte(uint8_t data) {
    while (!(UCSR0A & (1<<UDRE0))) ;
    UDR0 = data;
}

/* ส่งสตริง (จบด้วย '\0') ออกทาง USART */
static void usart0_send_string(const char *s) {
    while (*s) {
        usart0_send_byte((uint8_t)*s++);
    }
}

/*------------------------------------------------------------------
 * ส่วน LCD 16×2 โหมด 4 บิต
 * ใช้งานด้วย RS=PB1, EN=PB2, D4..D7=PC4..PC7
 *----------------------------------------------------------------*/
static void lcd_pulse_enable(void) {
    PORTB |= (1<<LCD_EN);
    _delay_us(1);
    PORTB &= ~(1<<LCD_EN);
    _delay_us(50);
}

static void lcd_write4(uint8_t nib) {
    PORTC = (PORTC & 0x0F) | ((nib & 0x0F) << 4);
    lcd_pulse_enable();
}

static void lcd_cmd(uint8_t cmd) {
    PORTB &= ~(1<<LCD_RS);
    lcd_write4(cmd >> 4);
    lcd_write4(cmd & 0x0F);
    if (cmd == 0x01 || cmd == 0x02) {
        _delay_ms(2);
    }
}

static void lcd_data(uint8_t data) {
    PORTB |= (1<<LCD_RS);
    lcd_write4(data >> 4);
    lcd_write4(data & 0x0F);
}

static void lcd_init(void) {
    /* ตั้งขาเป็น output */
    DDRB |= (1<<LCD_RS) | (1<<LCD_EN);
    DDRC |= (1<<LCD_D4) | (1<<LCD_D5) | (1<<LCD_D6) | (1<<LCD_D7);
    _delay_ms(40);
    /* ชุดเริ่มต้น */
    lcd_write4(0x03);
    _delay_ms(5);
    lcd_write4(0x03);
    _delay_us(150);
    lcd_write4(0x03);
    _delay_us(150);
    lcd_write4(0x02);
    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
}

static void lcd_gotoxy(uint8_t x, uint8_t y) {
    uint8_t addr = (y ? 0x40 : 0) + x;
    lcd_cmd(0x80 | addr);
}

static void lcd_print(const char *s) {
    while (*s) {
        lcd_data((uint8_t)*s++);
    }
}

static void lcd_clear(void) {
    lcd_cmd(0x01);
}

/*------------------------------------------------------------------
 * วัดระยะ (cm) ด้วย HC-SR04 – polling simplification
 *----------------------------------------------------------------*/
static unsigned int measure_distance_cm(void) {
    /* ทริกเกอร์ pulse 10 µs */
    PORTD &= ~(1<<TRIG_PIN);
    _delay_us(2);
    PORTD |= (1<<TRIG_PIN);
    _delay_us(10);
    PORTD &= ~(1<<TRIG_PIN);
    /* รอขอบขึ้นของ echo (timeout 30 ms) */
    unsigned long start_ms = g_ms;
    while (!(PIND & (1<<ECHO_PIN))) {
        if ((g_ms - start_ms) > 30) return 999; /* ไม่มี echo */
    }
    /* วัดความกว้างของสัญญาณ HIGH */
    unsigned long count_us = 0;
    while (PIND & (1<<ECHO_PIN)) {
        _delay_us(1);
        count_us++;
        if (count_us > 30000UL) break;
    }
    return (unsigned int)(count_us / 58UL);
}

/*------------------------------------------------------------------
 * คำนวณ progress = pill_count * 100 / target_count
 *----------------------------------------------------------------*/
static void update_progress(void) {
    if (!have_target || target_count == 0) {
        progress_percent = 0;
        return;
    }
    unsigned long p = (unsigned long)pill_count * 100UL / target_count;
    if (p > 100) p = 100;
    progress_percent = (unsigned int)p;
}

/*------------------------------------------------------------------
 * อัปเดตข้อความบน LCD
 *----------------------------------------------------------------*/
static void update_lcd_display(void) {
    char buf[17];
    lcd_gotoxy(0, 0);
    snprintf(buf, sizeof(buf), "Pills:%3u Target:%3u", pill_count, target_count);
    lcd_print(buf);
    lcd_gotoxy(0, 1);
    snprintf(buf, sizeof(buf), "Progress:%3u%%", progress_percent);
    lcd_print(buf);
}

/*------------------------------------------------------------------
 * ส่งสถานะไปยังบอร์ด 3
 *----------------------------------------------------------------*/
static void send_status_to_board3(void) {
    char buf[20];
    snprintf(buf, sizeof(buf), "PILLS:%03u\n", pill_count);
    usart0_send_string(buf);
    snprintf(buf, sizeof(buf), "PROGRESS:%03u\n", progress_percent);
    usart0_send_string(buf);
    last_send_ms = g_ms;
}

/*------------------------------------------------------------------
 * จัดการข้อความที่รับครบหนึ่งบรรทัด
 *----------------------------------------------------------------*/
static void handle_received_line(void) {
    rx_buf[rx_len] = '\0';
    if (strncmp(rx_buf, "TARGET:", 7) == 0) {
        int v = atoi(rx_buf + 7);
        if (v >= 0 && v <= 999) {
            target_count = (unsigned int)v;
            have_target = true;
        }
    } else if (strcmp(rx_buf, "RESET") == 0) {
        pill_count = 0;
        progress_percent = 0;
    }
    rx_len = 0;
}

/*------------------------------------------------------------------
 * Poll USART เพื่ออ่านข้อมูลทีละตัวอักษรและสะสมจนจบบรรทัด
 *----------------------------------------------------------------*/
static void poll_usart_rx(void) {
    while (UCSR0A & (1<<RXC0)) {
        char c = UDR0;
        last_serial_ms = g_ms;
        if (c == '\r') continue;
        if (c == '\n') {
            handle_received_line();
        } else {
            if (rx_len < (RX_BUF_SIZE - 1)) {
                rx_buf[rx_len++] = c;
            } else {
                rx_len = 0;
            }
        }
    }
}

/*------------------------------------------------------------------
 * ฟังก์ชันหลัก
 *----------------------------------------------------------------*/
int main(void) {
    /* ตั้งทิศทางขา TRIG เป็น output, ECHO เป็น input */
    DDRD |= (1<<TRIG_PIN);
    DDRD &= ~(1<<ECHO_PIN);
    /* ปิด TRIG */
    PORTD &= ~(1<<TRIG_PIN);
    /* LED timeout เป็น output */
    DDRB |= (1<<LED_TIMEOUT);
    PORTB &= ~(1<<LED_TIMEOUT);

    /* เริ่มต้นโมดูล */
    timer1_init_1ms();
    usart0_init();
    lcd_init();
    /* เปิดใช้งาน Watchdog Timer ประมาณ 2 วินาที 
     * หากลูปหลักไม่เรียก wdt_reset() ภายในเวลานี้ MCU จะรีเซ็ตตัวเอง 
     * การเปิดใช้งาน WDT หลังปิด global interrupt (คล้ายตามคู่มือ) เพื่อไม่ให้รีเซ็ตทันที */
    MCUSR &= ~(1<<WDRF);           /* ล้างแฟล็กรีเซ็ตจาก WDT ครั้งก่อน */
    wdt_enable(WDTO_2S);           /* ตั้ง WDT timeout ≈ 2s */

    sei();

    lcd_print("Board1 Ready");
    _delay_ms(1000);
    lcd_clear();

    last_serial_ms = g_ms;
    last_send_ms   = g_ms;

    /* วนลูปหลักตลอดเวลา */
    for (;;) {
        /* ป้อนอาหารให้ Watchdog ถ้ายังไม่เกิด timeout 
         * ในกรณีปกติจะต้องเรียก wdt_reset() ในทุก ๆ รอบ
         * หากเคยเกิด timeout (wdt_timeout=true) จะไม่รีเซ็ต WDT เพื่อให้รีบูต */
        if (!wdt_timeout) {
            wdt_reset();
        }
        poll_usart_rx();
        /* ตรวจ timeout ของสื่อสาร หากเกิน 30 วินาทีให้แสดงข้อความและหยุดป้อน WDT */
        if ((g_ms - last_serial_ms) > 30000UL) {
            lcd_gotoxy(0, 1);
            lcd_print("Timeout!       ");
            /* กระพริบ LED13 เพื่อบอกว่ายังมีชีวิตแต่รอรีเซ็ต */
            if (((g_ms / 500) % 2) == 0) PORTB |= (1<<LED_TIMEOUT);
            else PORTB &= ~(1<<LED_TIMEOUT);
            /* ถ้ายังไม่ตั้ง wdt_timeout ให้ตั้งและหยุดป้อน WDT */
            wdt_timeout = true;
        } else {
            PORTB &= ~(1<<LED_TIMEOUT);
        }
        /* นับเม็ดยาเมื่อมีเป้าหมายแล้วเท่านั้น */
        unsigned int d1 = measure_distance_cm();
        unsigned int d2 = measure_distance_cm();
        unsigned int d3 = measure_distance_cm();
        if (d1 < 10 && d2 < 10 && d3 < 10) {
            if (g_ms > lockout_until_ms) {
                if (pill_count < 999) pill_count++;
                lockout_until_ms = g_ms + 150;
                update_progress();
                update_lcd_display();
            }
        }
        /* ส่งสถานะไปบอร์ด 3 ทุก 200 ms เมื่อมีเป้าหมายแล้ว */
        if (have_target && ((g_ms - last_send_ms) >= 200)) {
            update_progress();
            send_status_to_board3();
            update_lcd_display();
        }
    }
    return 0;
}