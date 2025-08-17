/*
 * Board 3: Monitor & Control (7‑segment display + LEDs + Solenoid)
 *
 * บอร์ดนี้ทำหน้าที่ตรวจสอบจำนวนเม็ดยาและความคืบหน้าที่ส่งมาจากบอร์ด 1
 * และควบคุมการแสดงผลบนจอ 7‑segment 4 หลัก, แสดงสถานะด้วย LED 4 ดวง
 * เมื่อถึงเป้าหมาย 100% จะเริ่มกระบวนการบรรจุและล้าง และส่งคำสั่ง RESET
 * กลับไปยังบอร์ด 1 หลังจากทำความสะอาดเสร็จ
 *
 * การสื่อสาร: USART0 ใช้ความเร็ว 9600 bps, 8 data bits, Even parity, 1 stop bit (8E1)
 * รูปแบบข้อความ:
 *   รับจากบอร์ด 1: "PILLS:xxx\n" (จำนวนเม็ด) และ "PROGRESS:yyy\n" (เปอร์เซ็นต์)
 *   ส่งกลับไปบอร์ด 1: "RESET\n" เมื่อครบเป้าหมายและล้างเสร็จ
 * ข้อความต้องจบด้วย '\n' เสมอ และต้องมี timeout 30 s หากไม่ได้รับข้อมูลใหม่
 *
 * ฮาร์ดแวร์:
 *   - 7‑segment 4 หลัก, multiplexed ด้วย Timer2 interrupt
 *     โดยขาเลือกหลัก (digit select) เชื่อมกับ PD4..PD7 (bit 4‑7) และ
 *     ขา segment a..f เชื่อมกับ PC0..PC5 (bit 0‑5) ส่วน segment g เชื่อมกับ PB0 (bit 0)
 *     โค้ด segment แบบ active‑high: bit0=a, bit1=b, bit2=c, bit3=d, bit4=e, bit5=f, bit6=g
 *   - LED สถานะ 4 สีต่อที่พอร์ต B: PB2=เขียว, PB3=เหลือง, PB4=แดง, PB5=น้ำเงิน
 *   - LED บรรจุ (Packaging) ต่อที่ PD2, Solenoid Valve ต่อที่ PD3
 *
 * การทำงาน:
 *   - ขณะโหมดปกติ (M_RUN) จะแสดงจำนวนเม็ดยา (mod 100) ในสองหลักซ้าย
 *     และเปอร์เซ็นต์ความคืบหน้า (mod 100) ในสองหลักขวา
 *     ไฟเขียวเมื่อ progress < 50%, เหลืองเมื่อ 50–89%, แดงเมื่อ 90–99%
 *   - เมื่อ progress ≥ 100% จะเข้าสู่โหมดล้าง (M_CLEAN)
 *       • เปิด packaging LED 5 s (ไฟ/รีเลย์ที่ PD2)
 *       • เปิด solenoid valve 3 s (PD3)
 *       • แสดง “CLEn” บน 7‑segment
 *       • ส่ง "RESET\n" กลับบอร์ด 1 และรอจบการล้าง
 *       • เมื่อครบเวลา 5 s จะปิดอุปกรณ์ทั้งหมด เพิ่มตัวนับ batch และกลับสู่ M_RUN
 *   - ถ้าไม่ได้รับข้อมูลใหม่จากบอร์ด 1 เกิน 30 s จะเข้าสู่โหมด Timeout (M_TIMEOUT)
 *       • แสดง "----" บน 7‑segment และเปิดไฟแดงค้างไว้
 *       • เมื่อได้รับข้อมูลอีกครั้งจะกลับสู่ M_RUN
 *   - LED น้ำเงินจะกระพริบที่ความถี่ประมาณ 4 Hz ในช่วง progress ≥ 100%
 *
 * หมายเหตุ: โค้ดนี้ออกแบบให้เข้าใจง่าย ไม่จำเป็นต้องแม่นยำระดับโปรดักชัน
 * แต่พยายามครอบคลุมหัวข้อในบทเรียน (Timer, Interrupt, ADC, Serial, etc.)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define F_CPU 16000000UL

/*---------------- Pin mapping ----------------*/
/* Segment pins (active‑high)
 *  bit0=a → PC0
 *  bit1=b → PC1
 *  bit2=c → PC2
 *  bit3=d → PC3
 *  bit4=e → PC4
 *  bit5=f → PC5
 *  bit6=g → PB0
 */
#define SEG_A_PORT    PORTC
#define SEG_A_DDR     DDRC
// a..f on PC0..PC5
#define SEG_PORT_PC_MASK  0x3F     /* mask PC0‑PC5 */
#define SEG_PIN_G_PORT PORTB
#define SEG_PIN_G_DDR  DDRB
#define SEG_PIN_G_BIT  PB0

/* Digit select pins (active‑high)
 *  digit0 (พัน)  → PD4
 *  digit1 (ร้อย) → PD5
 *  digit2 (สิบ)  → PD6
 *  digit3 (หน่วย)→ PD7
 */
#define DIGIT_PORT    PORTD
#define DIGIT_DDR     DDRD
#define DIGIT_MASK    ((1<<PD4)|(1<<PD5)|(1<<PD6)|(1<<PD7))

/* LED pins (PB2..PB5)
 *  Green  = PB2
 *  Yellow = PB3
 *  Red    = PB4
 *  Blue   = PB5
 */
#define LED_PORT      PORTB
#define LED_DDR       DDRB
#define LED_GREEN_BIT PB2
#define LED_YELL_BIT  PB3
#define LED_RED_BIT   PB4
#define LED_BLUE_BIT  PB5

/* Output control pins for packaging LED and solenoid valve */
#define PACK_PORT     PORTD
#define PACK_DDR      DDRD
#define PACK_PIN      PD2      /* เปิดบรรจุขวด (Packaging LED) */
#define SOL_PORT      PORTD
#define SOL_DDR       DDRD
#define SOL_PIN       PD3      /* เปิดวาล์วน้ำล้าง (Solenoid valve) */

/* USART settings: 9600 bps, 8 data bits, even parity, 1 stop bit (8E1) */
#define BAUD_RATE 9600UL

/* Timeout and cleaning durations (ms) */
#define TIMEOUT_MS      30000UL /* 30 s timeout สำหรับสื่อสาร */
#define PACK_TIME_MS     5000UL /* 5 s สำหรับ packaging LED */
#define WASH_TIME_MS     3000UL /* 3 s สำหรับ solenoid valve */

/*---------------- State variables ----------------*/
static volatile unsigned long g_ms = 0;   /* ตัวนับมิลลิวินาที (จาก Timer0) */
static volatile uint8_t cur_digit_idx = 0;/* ตัวเลือกหลักปัจจุบันของ 7‑seg สำหรับ multiplex */
static volatile uint16_t pills = 0;        /* จำนวนเม็ดยาปัจจุบัน (0–999) */
static volatile uint8_t progress = 0;      /* ความคืบหน้า (% 0–100) */
static volatile uint8_t seg_codes[4] = {0,0,0,0}; /* รหัส segment สำหรับแต่ละหลัก */

static unsigned long last_rx_ms = 0;      /* เวลาล่าสุดที่ได้รับข้อมูล */
static unsigned long clean_start_ms = 0;  /* เวลาเริ่มล้าง */
static uint16_t batch_count = 0;          /* นับจำนวนรอบผลิตที่เสร็จ */

/* โหมดการทำงานของบอร์ด 3 */
typedef enum { M_RUN, M_CLEAN, M_TIMEOUT } run_mode_t;
static run_mode_t mode = M_RUN;

/*---------------- Segment encode table ----------------*/
/* encode digit 0–9 ตามลำดับ gfedcba (bit0=a) */
static const uint8_t digit_table[10] = {
    0x3F, /* 0: a b c d e f */
    0x06, /* 1: b c */
    0x5B, /* 2: a b d e g */
    0x4F, /* 3: a b c d g */
    0x66, /* 4: b c f g */
    0x6D, /* 5: a c d f g */
    0x7D, /* 6: a c d e f g */
    0x07, /* 7: a b c */
    0x7F, /* 8: a b c d e f g */
    0x6F  /* 9: a b c d f g */
};
/* encode ตัวอักษรและสัญลักษณ์อื่น ๆ */
static uint8_t encode_char(char ch) {
    switch (ch) {
        case 'C': return 0x39; /* a, d, e, f */
        case 'L': return 0x38; /* d, e, f */
        case 'E': return 0x79; /* a, d, e, f, g */
        case 'n': return 0x54; /* g, e, c (ประมาณรูป n เล็ก) */
        case '-': return 0x40; /* g */
        case ' ': return 0x00; /* blank */
        default:
            return 0x00;
    }
}

/*---------------- Timer2: Multiplex 7‑segment + blink blue ----------------*/
/* 
 * Timer2 ใช้ CTC mode เพื่อสร้าง interrupt ความถี่ 1 kHz
 * ช่วยสลับหลัก (digit) ของ 7‑segment อย่างเร็วพอที่ตาคนเห็นเป็นภาพต่อเนื่อง
 * และกระพริบ LED น้ำเงินเมื่อ progress ≥ 100%
 */
static void timer2_init_1kHz(void) {
    /* CTC: WGM21=1, prescale=64 → f = F_CPU/64 = 250 kHz
     * ต้องการ 1 kHz → OCR2A = 250 kHz / 1000 - 1 = 249 */
    TCCR2A = (1<<WGM21);
    TCCR2B = (1<<CS22);    /* prescale 64 */
    OCR2A  = 249;
    TIMSK2 = (1<<OCIE2A);
}

ISR(TIMER2_COMPA_vect) {
    /* สลับ digit: ปิดทุกหลักก่อน */
    DIGIT_PORT &= ~DIGIT_MASK;
    /* ตั้ง segment ตามข้อมูลของหลักปัจจุบัน */
    uint8_t code = seg_codes[cur_digit_idx & 0x03];
    /* อัพเดต PC0‑PC5 (a..f) */
    SEG_A_PORT = (SEG_A_PORT & ~SEG_PORT_PC_MASK) | (code & 0x3F);
    /* อัพเดต PB0 (g) */
    if (code & 0x40) SEG_PIN_G_PORT |= (1<<SEG_PIN_G_BIT);
    else             SEG_PIN_G_PORT &= ~(1<<SEG_PIN_G_BIT);
    /* เปิด digit ปัจจุบัน (PD4‑PD7) */
    switch (cur_digit_idx & 0x03) {
        case 0: DIGIT_PORT |= (1<<PD4); break;
        case 1: DIGIT_PORT |= (1<<PD5); break;
        case 2: DIGIT_PORT |= (1<<PD6); break;
        case 3: DIGIT_PORT |= (1<<PD7); break;
    }
    /* เตรียมหลักถัดไป */
    cur_digit_idx++;
    /* กระพริบ LED น้ำเงินที่ ~4 Hz เมื่อ progress >= 100 */
    static uint16_t blink_cnt = 0;
    if (progress >= 100) {
        blink_cnt++;
        /* Interrupt 1 kHz → 250 counts = ~250 ms → ครึ่งรอบ = 125 ms (4 Hz) */
        if (blink_cnt >= 125) {
            /* toggle blue LED */
            LED_PORT ^= (1<<LED_BLUE_BIT);
            blink_cnt = 0;
        }
    } else {
        /* ดับน้ำเงินถ้าไม่ครบ 100% */
        LED_PORT &= ~(1<<LED_BLUE_BIT);
        blink_cnt = 0;
    }
}

/*---------------- Timer0: 1 ms tick ----------------*/
/* ใช้ Timer0 ใน CTC mode สร้าง interrupt ทุก 1 ms เพื่อเพิ่มตัวนับ g_ms
 * ช่วยใช้ในการ timeout และจับเวลาการล้าง
 */
static void timer0_init_1ms(void) {
    /* CTC: WGM01=1, prescaler=64 → f = F_CPU/64 = 250 kHz
     * ต้องการ 1 kHz → OCR0A = 250 kHz / 1000 - 1 = 249 */
    TCCR0A = (1<<WGM01);
    TCCR0B = (1<<CS01) | (1<<CS00); /* prescaler = 64 */
    OCR0A  = 249;
    TIMSK0 = (1<<OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
    g_ms++;
}

/*---------------- USART0 (9600 8E1) ----------------*/
static void usart0_init(void) {
    /* คำนวณ UBRR สำหรับ 9600 bps (U2X=0) */
    uint16_t ubrr = (F_CPU / (16UL * BAUD_RATE)) - 1;
    UBRR0H = (ubrr >> 8) & 0xFF;
    UBRR0L = ubrr & 0xFF;
    /* เซต 8 data bits, even parity, 1 stop bit */
    UCSR0C = (1<<UPM01) | (1<<UCSZ01) | (1<<UCSZ00);
    /* เปิดภาครับ/ส่ง */
    UCSR0B = (1<<RXEN0) | (1<<TXEN0);
}

/* ส่ง 1 ไบต์ */
static void usart0_send_byte(uint8_t data) {
    while (!(UCSR0A & (1<<UDRE0))) ;
    UDR0 = data;
}
/* ส่งสตริง (จบด้วย '\0') */
static void usart0_send_string(const char *s) {
    while (*s) {
        usart0_send_byte((uint8_t)*s++);
    }
}

/*---------------- LED helper ----------------*/
static void led_set(bool g, bool y, bool r) {
    if (g) LED_PORT |=  (1<<LED_GREEN_BIT); else LED_PORT &= ~(1<<LED_GREEN_BIT);
    if (y) LED_PORT |=  (1<<LED_YELL_BIT);  else LED_PORT &= ~(1<<LED_YELL_BIT);
    if (r) LED_PORT |=  (1<<LED_RED_BIT);   else LED_PORT &= ~(1<<LED_RED_BIT);
    /* LED Blue handled in timer ISR */
}

/*---------------- Display composition ----------------*/
/* สร้างรหัส 7‑segment สำหรับโหมดปกติ: ซ้าย = pills mod 100, ขวา = progress mod 100 */
static void compose_run_frame(void) {
    uint8_t left  = (uint8_t)(pills % 100);
    uint8_t right = (uint8_t)(progress % 100);
    seg_codes[0] = digit_table[left / 10];
    seg_codes[1] = digit_table[left % 10];
    seg_codes[2] = digit_table[right / 10];
    seg_codes[3] = digit_table[right % 10];
}

/* โหมดล้าง: แสดง "CLEn" */
static void compose_clean_frame(void) {
    seg_codes[0] = encode_char('C');
    seg_codes[1] = encode_char('L');
    seg_codes[2] = encode_char('E');
    seg_codes[3] = encode_char('n');
}

/* โหมด timeout: แสดง "----" */
static void compose_timeout_frame(void) {
    seg_codes[0] = encode_char('-');
    seg_codes[1] = encode_char('-');
    seg_codes[2] = encode_char('-');
    seg_codes[3] = encode_char('-');
}

/*---------------- Serial line parser ----------------*/
/* บัฟเฟอร์รับข้อความ (สูงสุด 31 ตัวอักษร + NUL) */
#define RX_BUFFER_SIZE 32
static char rx_buffer[RX_BUFFER_SIZE];
static uint8_t rx_index = 0;

/* เรียกในลูปหลักเพื่ออ่านข้อมูลจาก UART0 แล้ว parse เมื่อถึง '\n' */
static void poll_serial(void) {
    while (UCSR0A & (1<<RXC0)) {
        char c = UDR0;
        last_rx_ms = g_ms;
        if (c == '\r') continue;
        if (c == '\n') {
            rx_buffer[rx_index] = '\0';
            /* parse: "PILLS:xxx" หรือ "PROGRESS:yyy" */
            if (strncmp(rx_buffer, "PILLS:", 6) == 0) {
                int v = atoi(rx_buffer + 6);
                if (v >= 0 && v <= 999) pills = (uint16_t)v;
            } else if (strncmp(rx_buffer, "PROGRESS:", 9) == 0) {
                int v = atoi(rx_buffer + 9);
                if (v >= 0 && v <= 100) progress = (uint8_t)v;
            }
            rx_index = 0;
        } else {
            if (rx_index < (RX_BUFFER_SIZE-1)) {
                rx_buffer[rx_index++] = c;
            } else {
                rx_index = 0; /* overflow → reset */
            }
        }
    }
}

/*---------------- Main ----------------*/
int main(void) {
    /* ตั้งพอร์ตของ segment (PC0-PC5) และ segment g (PB0) เป็น output */
    SEG_A_DDR |= SEG_PORT_PC_MASK;
    SEG_PIN_G_DDR |= (1<<SEG_PIN_G_BIT);
    /* ตั้งพอร์ต digit select (PD4-PD7) เป็น output */
    DIGIT_DDR |= DIGIT_MASK;
    /* ตั้ง LED พอร์ต B (PB2-PB5) เป็น output */
    LED_DDR |= (1<<LED_GREEN_BIT) | (1<<LED_YELL_BIT) | (1<<LED_RED_BIT) | (1<<LED_BLUE_BIT);
    /* ตั้ง packaging และ solenoid เป็น output */
    PACK_DDR |= (1<<PACK_PIN);
    SOL_DDR  |= (1<<SOL_PIN);
    /* ปิด packaging & solenoid ในตอนเริ่มต้น */
    PACK_PORT &= ~(1<<PACK_PIN);
    SOL_PORT  &= ~(1<<SOL_PIN);
    /* Clear segments and digits */
    SEG_A_PORT &= ~SEG_PORT_PC_MASK;
    SEG_PIN_G_PORT &= ~(1<<SEG_PIN_G_BIT);
    DIGIT_PORT &= ~DIGIT_MASK;
    /* เริ่มต้น LED: ปิดหมด */
    LED_PORT &= ~((1<<LED_GREEN_BIT)|(1<<LED_YELL_BIT)|(1<<LED_RED_BIT)|(1<<LED_BLUE_BIT));

    /* เริ่มต้นตัวนับเวลา */
    timer0_init_1ms();
    timer2_init_1kHz();
    /* เริ่มต้น USART0 */
    usart0_init();
    /* เปิด interrupt ทั่วไป */
    sei();
    /* ค่าเริ่มต้น */
    last_rx_ms = g_ms;
    clean_start_ms = 0;
    pills = 0;
    progress = 0;
    mode = M_RUN;
    compose_run_frame();

    for (;;) {
        /* Poll serial เพื่อรับข้อความจากบอร์ด 1 */
        poll_serial();
        /* Timeout: ถ้าไม่มีข้อมูลเกิน TIMEOUT_MS → M_TIMEOUT */
        if ((g_ms - last_rx_ms) > TIMEOUT_MS) {
            mode = M_TIMEOUT;
        }
        /* โหมด timeout: แสดง "----" และแดงค้าง */
        if (mode == M_TIMEOUT) {
            compose_timeout_frame();
            led_set(false, false, true);
            /* เมื่อได้รับข้อมูลใหม่ (last_rx_ms ถูกอัปเดตใน poll_serial) จะออกจาก timeout */
            if ((g_ms - last_rx_ms) <= TIMEOUT_MS) {
                mode = M_RUN;
            }
        }
        /* โหมดล้าง */
        else if (mode == M_CLEAN) {
            /* ตรวจจับเวลาผ่านไปนานเท่าไรตั้งแต่เริ่มล้าง */
            unsigned long dt = g_ms - clean_start_ms;
            if (dt < WASH_TIME_MS) {
                /* เปิด solenoid แรก 3 s */
                SOL_PORT |= (1<<SOL_PIN);
            } else {
                SOL_PORT &= ~(1<<SOL_PIN);
            }
            if (dt < PACK_TIME_MS) {
                /* เปิด packaging 5 s */
                PACK_PORT |= (1<<PACK_PIN);
            } else {
                PACK_PORT &= ~(1<<PACK_PIN);
            }
            /* แสดง CLEn โดยไม่เปลี่ยน LED (ใช้ตัวอื่นทำ) */
            compose_clean_frame();
            led_set(false, false, false);
            /* เมื่อครบเวลาทั้งสอง → จบโหมดล้าง */
            if (dt >= ((PACK_TIME_MS > WASH_TIME_MS) ? PACK_TIME_MS : WASH_TIME_MS)) {
                mode = M_RUN;
                pills = 0;
                progress = 0;
                batch_count++;
                /* ปิด output */
                PACK_PORT &= ~(1<<PACK_PIN);
                SOL_PORT  &= ~(1<<SOL_PIN);
                /* หลังล้างให้ไฟเขียว (พร้อมรับข้อมูล) */
                led_set(true, false, false);
                compose_run_frame();
            }
        }
        /* โหมดปกติ */
        else if (mode == M_RUN) {
            /* อัปเดตรหัสแสดงผล */
            compose_run_frame();
            /* อัปเดต LED สถานะตาม progress */
            if (progress < 50) {
                led_set(true, false, false);
            } else if (progress < 90) {
                led_set(false, true, false);
            } else if (progress < 100) {
                led_set(false, false, true);
            }
            /* ถ้า progress >= 100 → เริ่มล้าง */
            if (progress >= 100) {
                mode = M_CLEAN;
                clean_start_ms = g_ms;
                /* ส่งคำสั่ง RESET ไปบอร์ด 1 (แจ้งรีเซ็ต) */
                usart0_send_string("RESET\n");
            }
        }
        /* หน่วงสั้น ๆ ให้ CPU พัก (ลูปนี้ไม่บล็อกเพราะมี interrupt ทำงานหลัก) */
        _delay_ms(10);
    }
    return 0;
}
