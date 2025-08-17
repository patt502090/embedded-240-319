/*
 * Board 2: Input Control (Keypad ADC + LCD + LEDs)
 *
 * บอร์ดนี้ทำหน้าที่รับจำนวนเม็ดยาเป้าหมายจากผู้ใช้ผ่าน Keypad แบบ ADC 4×4
 * ค่า ADC channel A5 (PC5) ถูกอ่านเพื่อระบุปุ่มตามช่วงแรงดันที่กำหนดในโจทย์
 * เมื่อกดตัวเลข 0–9 จะสะสมเป็นตัวเลข 3 หลัก (0–999)
 * กด * ล้างค่า, กด # ยืนยันและส่งค่าไปบอร์ด 1 ในรูปแบบ "TARGET:xxx\n"
 * มี debouncing 50 ms และ timeout ถ้าไม่มีการกดเกิน 30 วินาทีจะกลับสู่สถานะพร้อม
 * แสดงค่าป้อนและคำแนะนำบน LCD 16×2 และแสดงสถานะด้วย LED 3 สี (เขียว/เหลือง/แดง)
 */

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>       /* สำหรับ atoi, itoa */
#include <stdbool.h>
#include <string.h>

/* เพิ่มไลบรารีสำหรับ Watchdog Timer */
#include <avr/wdt.h>

/*---------------- Pin definitions ----------------*/
#define LCD_RS    PB1    /* RS ของ LCD */
#define LCD_EN    PB2    /* EN ของ LCD */
#define LCD_D4    PC4    /* D4 ของ LCD */
#define LCD_D5    PC5    /* D5 ของ LCD */
#define LCD_D6    PC6    /* D6 ของ LCD */
#define LCD_D7    PC7    /* D7 ของ LCD */

#define PIN_KEY_ADC PC5  /* อ่าน ADC channel 5 (PC5 = A5) สำหรับ keypad */

/* LED แสดงสถานะ (board2 ใช้พอร์ต PD4..PD6) */
#define LED_GREEN  PD4
#define LED_YELLOW PD5
#define LED_RED    PD6

/*---------------- Global variables ----------------*/
volatile unsigned long g_ms = 0;   /* 1 ms tick จาก Timer1 */

static unsigned int target_value = 0;    /* ค่าที่ผู้ใช้ป้อน (0–999) */
static bool editing = false;              /* กำลังป้อนค่าอยู่หรือไม่ */
static unsigned long idle_start_ms = 0;   /* เวลานิ่งล่าสุด ใช้นับ timeout */

/* ตัวแปรสถานะสำหรับ WDT: ถ้าเป็น true จะหยุดรีเซ็ต watchdog เพื่อให้ MCU รีเซ็ตตัวเอง
 * เราใช้ volatile เพื่อให้มั่นใจว่าถูกอ่าน/เขียนอย่างถูกต้องใน ISR หรือส่วนอื่น ๆ
 */
static volatile bool wdt_timeout = false;

/*---------------- Timer1: 1 ms tick ----------------*/
static void timer1_init_1ms(void) {
    TCCR1A = 0x00;
    TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10); /* CTC, prescale 64 */
    OCR1A  = 249;  /* 16MHz/64/(1+249) = 1 kHz → 1 ms */
    TIMSK1 = (1<<OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
    g_ms++;
}

/*---------------- ADC init/read ----------------*/
static void adc_init(void) {
    ADMUX  = (1<<REFS0); /* Vref = AVcc, input channel later set in read */
    ADCSRA = (1<<ADEN) | (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); /* enable, prescale=128 */
}

/* อ่านค่า ADC ช่อง ch (0..7) คืนค่า 0..1023 */
static uint16_t adc_read(uint8_t ch) {
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
    ADCSRA |= (1<<ADSC);
    while (ADCSRA & (1<<ADSC));
    return ADC;
}

/*---------------- LCD 16×2 4-bit ----------------*/
static void lcd_pulse(void) {
    PORTB |= (1<<LCD_EN);
    _delay_us(1);
    PORTB &= ~(1<<LCD_EN);
    _delay_us(50);
}

static void lcd_write4(uint8_t nib) {
    PORTC = (PORTC & 0x0F) | ((nib & 0x0F) << 4);
    lcd_pulse();
}

static void lcd_cmd(uint8_t cmd) {
    PORTB &= ~(1<<LCD_RS);
    lcd_write4(cmd >> 4);
    lcd_write4(cmd & 0x0F);
    if (cmd == 0x01 || cmd == 0x02) _delay_ms(2);
}

static void lcd_data(uint8_t d) {
    PORTB |= (1<<LCD_RS);
    lcd_write4(d >> 4);
    lcd_write4(d & 0x0F);
}

static void lcd_init(void) {
    DDRB |= (1<<LCD_RS) | (1<<LCD_EN);
    DDRC |= (1<<LCD_D4) | (1<<LCD_D5) | (1<<LCD_D6) | (1<<LCD_D7);
    _delay_ms(40);
    lcd_write4(0x03); _delay_ms(5);
    lcd_write4(0x03); _delay_us(150);
    lcd_write4(0x03); _delay_us(150);
    lcd_write4(0x02);
    lcd_cmd(0x28); /* 4-bit, 2-line */
    lcd_cmd(0x0C); /* display ON */
    lcd_cmd(0x06); /* entry mode */
    lcd_cmd(0x01); /* clear */
}

static void lcd_gotoxy(uint8_t x, uint8_t y) {
    uint8_t addr = (y ? 0x40 : 0) + x;
    lcd_cmd(0x80 | addr);
}

static void lcd_print(const char *s) {
    while (*s) lcd_data((uint8_t)*s++);
}

static void lcd_clear(void) {
    lcd_cmd(0x01);
}

/*---------------- USART0 9600 8E1 ----------------*/
static void usart0_init(void) {
    uint16_t ubrr = (F_CPU / (16UL * 9600UL)) - 1;
    UBRR0H = (ubrr >> 8) & 0xFF;
    UBRR0L = ubrr & 0xFF;
    UCSR0C = (1<<UPM01) | (1<<UCSZ01) | (1<<UCSZ00); /* even parity, 8 data bits */
    UCSR0B = (1<<TXEN0); /* เราใช้ส่งอย่างเดียว */
}

static void usart0_send_byte(uint8_t data) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

static void usart0_send_string(const char *s) {
    while (*s) usart0_send_byte((uint8_t)*s++);
}

/*---------------- LED control ----------------*/
static void leds_set(bool g, bool y, bool r) {
    if (g) PORTD |= (1<<LED_GREEN); else PORTD &= ~(1<<LED_GREEN);
    if (y) PORTD |= (1<<LED_YELLOW); else PORTD &= ~(1<<LED_YELLOW);
    if (r) PORTD |= (1<<LED_RED); else PORTD &= ~(1<<LED_RED);
}

/*---------------- Keypad ADC decoding ----------------*/
/* แม็พช่วงแรงดัน ADC → อักขระคีย์ */
static char decode_key(uint16_t adc) {
    if (adc >=  50 && adc <=  80) return '1';
    if (adc >=  90 && adc <= 120) return '2';
    if (adc >= 130 && adc <= 160) return '3';
    if (adc >= 170 && adc <= 200) return '4';
    if (adc >= 210 && adc <= 240) return '5';
    if (adc >= 250 && adc <= 280) return '6';
    if (adc >= 290 && adc <= 320) return '7';
    if (adc >= 330 && adc <= 360) return '8';
    if (adc >= 370 && adc <= 400) return '9';
    if (adc >= 410 && adc <= 440) return '0';
    if (adc >= 450 && adc <= 480) return '*';
    if (adc >= 490 && adc <= 520) return '#';
    return 0;
}

/* อ่านปุ่มพร้อม debouncing 50 ms
 * คืนค่าอักขระ (0 ถ้าไม่พบปุ่มหรือยังไม่เสถียร)
 */
static char keypad_getkey(void) {
    static char last = 0;
    uint16_t a1 = adc_read(5);
    _delay_ms(50);
    uint16_t a2 = adc_read(5);
    if (abs((int)a2 - (int)a1) > 10) return 0; /* ค่าไม่นิ่ง */
    char k = decode_key(a2);
    if (k != 0 && k != last) {
        last = k;
        return k;
    }
    if (k == 0) {
        last = 0;
    }
    return 0;
}

/* ส่งค่า target ไปยังบอร์ด 1 */
static void send_target(unsigned int v) {
    char buf[16];
    /* ส่งข้อความในรูป TARGET:xxx\n */
    snprintf(buf, sizeof(buf), "TARGET:%03u\n", v);
    usart0_send_string(buf);
}

/* อัปเดต LCD ให้ตรงกับสถานะป้อน */
static void update_lcd(void) {
    char line[17];
    lcd_gotoxy(0, 0);
    snprintf(line, sizeof(line), "Set Target:%3u", target_value);
    lcd_print(line);
    lcd_gotoxy(0, 1);
    lcd_print("Press # to confirm");
}

int main(void) {
    /* ตั้งขา LED เป็น output */
    DDRD |= (1<<LED_GREEN) | (1<<LED_YELLOW) | (1<<LED_RED);
    leds_set(true, false, false); /* เริ่มต้น: เขียว = พร้อมรับคำสั่ง */

    timer1_init_1ms();
    adc_init();
    lcd_init();
    usart0_init();
    /*
     * ตรวจสอบว่า MCU ถูกรีเซ็ตจาก WDT หรือไม่
     * หากต้องการแจ้งเตือนผู้ใช้ สามารถเพิ่มข้อความในส่วนนี้
     */
    if (MCUSR & (1<<WDRF)) {
        /* ตัวอย่าง: เขียนข้อความหรือกระพริบไฟให้ผู้ใช้ทราบว่าเกิด Watchdog reset */
        /* ในที่นี้เราเพียงล้างแฟล็กโดยไม่มีการแจ้ง */
    }
    /* ล้างเหตุการณ์รีเซ็ตทั้งหมด เพื่อป้องกันการทับทวนในครั้งถัดไป */
    MCUSR = 0;
    /* เปิดใช้งาน Watchdog timer ที่ช่วงประมาณ 2 วินาที (WDTO_2S)
     * หากโปรแกรมไม่เรียก wdt_reset() ภายในช่วงนี้ MCU จะรีเซ็ตเอง
     */
    wdt_enable(WDTO_2S);

    sei();

    idle_start_ms = g_ms;
    target_value = 0;
    update_lcd();

    for(;;) {
        /* Timeout: 30 s ไม่มีการกด → กลับสู่พร้อม (ไฟเขียว)
         * พร้อมทั้งตั้ง wdt_timeout = true เพื่อหยุดรีเซ็ต watchdog
         * ทำให้ MCU รีเซ็ตตัวเองหลัง ~2 วินาที
         */
        if ((g_ms - idle_start_ms) > 30000UL) {
            editing = false;
            leds_set(true, false, false);
            idle_start_ms = g_ms;
            wdt_timeout = true;
        }
        /* อ่านคีย์จาก keypad (คืน 0 หากไม่มีการกด) */
        char k = keypad_getkey();

        if (k) {
            /* มีการกดปุ่ม → รีเซ็ต idle timer และเข้าสู่สถานะกำลังป้อน */
            idle_start_ms = g_ms;
            leds_set(false, true, false); /* เหลือง: กำลังตั้งค่า */
            editing = true;

            if (k >= '0' && k <= '9') {
                /* ต่อท้ายตัวเลข แต่ไม่เกิน 999 */
                unsigned int newv = target_value * 10 + (k - '0');
                if (newv <= 999) {
                    target_value = newv;
                    update_lcd();
                } else {
                    /* ถ้าค่าเกิน 999 ให้แจ้ง error ด้วยไฟแดงกระพริบสั้น ๆ */
                    leds_set(false, false, true);
                    _delay_ms(200);
                    leds_set(false, true, false);
                }
            } else if (k == '*') {
                /* ปุ่มล้าง */
                target_value = 0;
                update_lcd();
            } else if (k == '#') {
                /* ยืนยัน → ส่งไปบอร์ด 1 */
                send_target(target_value);
                leds_set(true, false, false); /* เขียว: พร้อมรับคำสั่งใหม่ */
                editing = false;
                update_lcd();
            }
        }

        /* รีเซ็ต watchdog ถ้าไม่อยู่ในสถานะ timeout
         * การเรียกนี้จะเกิดในทุกลูป ไม่ว่าจะมีการกดคีย์หรือไม่
         */
        if (!wdt_timeout) {
            wdt_reset();
        }
    }
    return 0;
}
