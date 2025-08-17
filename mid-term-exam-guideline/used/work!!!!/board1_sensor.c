/*
 * ========================= Board 1: Sensor & Display =========================
 * หน้าที่หลัก:
 *   - นับเม็ดยาด้วย HC-SR04 (อ่าน 3 ครั้งติดกัน + lockout 150ms กันนับซ้ำ)
 *   - อ่านระดับน้ำ (ADC A1) → แปลงเป็นเปอร์เซ็นต์เพื่อความปลอดภัย
 *   - เมื่อ "น้ำสูง" (≥ 80% ต่อเนื่อง) → หยุดนับเม็ดยาชั่วคราว (water_lock) และแจ้งบอร์ด 3 ด้วย "WATER:HI"
 *   - แสดงผลบน LCD 16x2: P (pills), T (target), Prg (%), W (%)
 *   - ส่งสถานะให้บอร์ด 3 ทาง UART: "PILLS:xxx\n", "PROGRESS:yyy\n"
 *   - รับค่าตั้งเป้าจากบอร์ด 2: "TARGET:xxx\n"
 *   - หากไม่มีการสื่อสาร >30s → แสดง Timeout + ปล่อยให้ Watchdog Reset ~2s
 *
 * การสื่อสาร Serial (USART0, 9600 8E1):
 *   - 8 data bits, Even parity, 1 stop bit (8E1) เพื่อความทนทานมากขึ้น
 *   - RX/TX ใช้ D0/D1 (ห้ามต่อเซ็นเซอร์ร่วมขา)
 *
 * ฮาร์ดแวร์/การต่อสาย:
 *   - Ultrasonic HC-SR04 : TRIG=D2 (PD2), ECHO=D3 (PD3)
 *   - Water level (ADC)  : A1 (PC1) → อ่านค่า 0..1023 แล้ว map เป็นเปอร์เซ็นต์
 *   - LCD 16x2 (LiquidCrystal): RS=D8, E=D9, D4=D4, D5=D5, D6=D6, D7=D7
 *   - LED Timeout        : D13 (PB5) → กระพริบเมื่อ timeout
 *
 * โมดูลเวลา/ความปลอดภัย:
 *   - Timer1 CTC 1ms tick → ตัวนับ g_ms ใช้จับเวลา/timeout โดยไม่ block
 *   - Watchdog 2s → ถ้าหลัก ๆ ค้าง/timeout ยาว เรา “ไม่ wdt_reset()” ให้รีบูตเอง
 */

#define F_CPU 16000000UL
#include <Arduino.h>        // สำหรับ pinMode/digitalWrite/pulseIn/delay* (Arduino core)
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>        // atoi
#include <LiquidCrystal.h> // ใช้ LCD แบบที่เรียน (ใช้ง่าย)


// ---------------------- Pin mapping ----------------------
#define TRIG_PIN 2     // D2 → TRIG ของ HC-SR04 (ขา Output)
#define ECHO_PIN 3     // D3 → ECHO ของ HC-SR04 (ขา Input)
#define LED_TIMEOUT 13 // D13 → LED แจ้ง Timeout/สถานะ

#define WATER_ADC_CH 1 // A1 → อ่านแรงดันระดับน้ำ

// ---------------------- LCD (แบบ Library ที่เรียน) ----------------------
// LiquidCrystal(rs, enable, d4, d5, d6, d7)
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ---------------------- Globals (สถานะระบบ) ----------------------
volatile unsigned long g_ms = 0;   // ตัวจับเวลาแบบมิลลิวินาที (increment ใน ISR ทุก 1ms)
static unsigned long last_serial_ms = 0;   // เวลาล่าสุดที่มีการ รับ/ส่ง UART
static unsigned long last_send_ms = 0;     // เวลาล่าสุดที่ส่งสถานะไปบอร์ด 3
static unsigned long lockout_until_ms = 0; // เวลากันนับซ้ำเม็ด (150 ms ต่อเม็ด)

static volatile unsigned int pill_count = 0;       // จำนวนเม็ดยาที่นับได้ (0..999)
static volatile unsigned int target_count = 0;     // เป้าหมายจากบอร์ด 2 (0..999)
static volatile unsigned int progress_percent = 0; // ความคืบหน้า 0..100%

static bool have_target = false;    // ได้รับ target แล้วหรือยัง
static bool wdt_timeout = false;    // ถ้า true → จะเลิกป้อน WDT เพื่อให้ MCU รีบูต

// เกณฑ์น้ำ (มี hysteresis กันเด้ง: ขึ้นที่ 80%, ลงที่ 75%)
#define WATER_HIGH_PCT 80
#define WATER_LOW_PCT  75
static volatile unsigned int water_percent = 0; // ค่าน้ำปัจจุบันเป็น %
static bool   water_lock   = false; // true = หยุดนับเม็ดยาชั่วคราว (น้ำสูง)
static bool   water_hi_sent= false; // ส่ง "WATER:HI" ไปบอร์ด 3 ครบแล้วยัง (หนึ่งครั้งต่อเหตุการณ์)
static uint8_t water_hi_cnt= 0;     // ตัวช่วย filter ให้ “สูงต่อเนื่อง” ก่อนตัดสินใจ

// ---------------------- RX buffer สำหรับ parse คำสั่งจากบอร์ดอื่น ----------------------
#define RX_BUF_SIZE 32
static char   rx_buf[RX_BUF_SIZE];
static uint8_t rx_len = 0;


// ---------------------- Timer1 → 1ms tick ----------------------
// ใช้ CTC mode, prescaler 64 → 16MHz/64=250kHz, ต้องการ 1kHz → OCR1A=249
static void timer1_init_1ms(void)
{
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    OCR1A  = 249;
    TIMSK1 = (1 << OCIE1A);
}
ISR(TIMER1_COMPA_vect) { g_ms++; } // เพิ่มตัวนับทุก 1ms


// ---------------------- USART0 9600 8E1 (ทน noise กว่า 8N1) ----------------------
static void usart0_init(void)
{
    uint16_t ubrr = (F_CPU / (16UL * 9600UL)) - 1; // U2X=0 → 103
    UBRR0H = (ubrr >> 8) & 0xFF;
    UBRR0L =  ubrr & 0xFF;
    UCSR0C = (1 << UPM01) | (1 << UCSZ01) | (1 << UCSZ00); // Even parity + 8 data + 1 stop
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
}
static void usart0_send_byte(uint8_t d) { while (!(UCSR0A & (1 << UDRE0))); UDR0 = d; }
static void usart0_send_string(const char *s) { while (*s) usart0_send_byte((uint8_t)*s++); }


// ---------------------- Ultrasonic แบบ pulseIn (ตามที่เรียน) ----------------------
// ส่งทริกเกอร์ 10µs → อ่านช่วงเวลาที่ echo=HIGH ด้วย pulseIn (timeout 30ms)
// ถ้าไม่เจอ echo (duration=0) → คืน 999.0f เป็น “ไกลเกิน/อ่านไม่ได้”
static float measure_distance_cm()
{
    digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // timeout ~30ms
    if (duration == 0) return 999.0f; // no echo
    // ระยะทาง (cm) = (เวลา(µs) × ความเร็วเสียง 0.034 cm/µs) / 2 (ไป-กลับ)
    return (duration * 0.034f) / 2.0f;
}


// ---------------------- ADC (อ่านระดับน้ำจาก A1) ----------------------
// ADMUX:   REFS0=1 → Vref=AVcc, เลือกช่องด้วย (ch & 0x0F)
// ADCSRA:  เปิด ADC + prescaler 128 → ให้การแปลงนิ่ง
static void     adc_init(void){ ADMUX = (1<<REFS0) | (WATER_ADC_CH & 0x0F);
                                ADCSRA= (1<<ADEN) | (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); }
static uint16_t adc_read(uint8_t ch){
    ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
    ADCSRA |= (1<<ADSC);
    while (ADCSRA & (1<<ADSC));  // wait conversion
    return ADC;
}


// ---------------------- คำนวณ progress (%) ----------------------
static void update_progress(void)
{
    if (!have_target || target_count == 0) { progress_percent = 0; return; }
    unsigned long p = (unsigned long)pill_count * 100UL / target_count;
    if (p > 100) p = 100;
    progress_percent = (unsigned int)p;
}


// ---------------------- อัปเดต LCD (ข้อความสั้น อ่านง่าย) ----------------------
static void lcd_show_status(void)
{
    lcd.setCursor(0, 0);
    char line0[17]; // 16 ตัว + NUL
    snprintf(line0, sizeof(line0), "P:%3u T:%3u   ", pill_count, target_count);
    lcd.print(line0);

    lcd.setCursor(0, 1);
    char line1[17];
    snprintf(line1, sizeof(line1), "Prg:%3u%% W:%2u%%", progress_percent, water_percent);
    lcd.print(line1);
}


// ---------------------- ส่งสถานะไปบอร์ด 3 ----------------------
static void send_status_to_board3(void)
{
    char buf[20];
    snprintf(buf, sizeof(buf), "PILLS:%03u\n",     pill_count);       usart0_send_string(buf);
    snprintf(buf, sizeof(buf), "PROGRESS:%03u\n",  progress_percent); usart0_send_string(buf);
    last_send_ms = g_ms;
}


// ---------------------- จัดการบรรทัดคำสั่งที่อ่านจบแล้ว (จากบอร์ด 2/3) ----------------------
// รูปแบบที่รองรับ:
//   - "TARGET:xxx" → ตั้ง target (0..999)
//   - "RESET"      → รีเซ็ตตัวนับและ progress
static void handle_received_line(void)
{
    rx_buf[rx_len] = '\0'; // ปิดสตริงให้ปลอดภัย
    if (strncmp(rx_buf, "TARGET:", 7) == 0)
    {
        int v = atoi(rx_buf + 7); //แปลง ส่วนตัวเลขที่ตามหลัง “TARGET:” เป็นจำนวนเต็ม
        if (v >= 0 && v <= 999){ target_count = (unsigned)v; have_target = true; }
    }
    else if (strcmp(rx_buf, "RESET") == 0) //ถ้าไม่ใช่ TARGET: ก็เช็คว่าเป็น RESET พอดีทั้งสตริงไหม
    {
        pill_count = 0;
        progress_percent = 0;
    }
    rx_len = 0; // เคลียร์บัฟเฟอร์เตรียมรอบใหม่
}


// ---------------------- Poll UART (อ่านทีละตัวอักษร จนจบบรรทัด '\n') ----------------------
static void poll_usart_rx(void)
{   
    while (UCSR0A & (1 << RXC0)) // มีข้อมูลในบัฟเฟอร์รับของ UART ไหม?
    //วนอ่าน “ตราบใดที่” UART มีตัวอักษรค้างอยู่ในบัฟเฟอร์รับ
    {
        char c = UDR0;           // อ่าน 1 ตัว
        last_serial_ms = g_ms;   // อัปเดตเวลา “มีการสื่อสารล่าสุด”
        if (c == '\r') continue; // ข้าม CR (รองรับ CRLF) "\r\n"
        if (c == '\n')           // เจอจบบรรทัด → ไป parse คำสั่ง
            handle_received_line();
        else {                   // สะสมลงบัฟเฟอร์
            if (rx_len < RX_BUF_SIZE - 1) rx_buf[rx_len++] = c;
            else rx_len = 0;     // ล้น → รีเซ็ต (กันค้าง)
        }
    }
}


// ---------------------- main() ----------------------
int main(void)
{
    // ตั้ง I/O พื้นฐาน
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(LED_TIMEOUT, OUTPUT);
    digitalWrite(LED_TIMEOUT, LOW);

    // โมดูลต่าง ๆ
    timer1_init_1ms();
    usart0_init();
    adc_init();

    // LCD (แบบ Library)
    lcd.begin(16, 2);
    lcd.clear();
    lcd.print("Board1 Ready");
    _delay_ms(800);
    lcd.clear();

    // Watchdog 2s (เปิดทิ้งไว้เพื่อความปลอดภัย — ถ้าค้าง/timeout ยาวให้รีบูต)
    MCUSR &= ~(1 << WDRF);   // ล้างสถานะรีเซ็ตจาก WDT ครั้งก่อน
    wdt_enable(WDTO_2S);

    sei(); // เปิด interrupt ทั้งระบบ

    last_serial_ms = last_send_ms = g_ms;

    for (;;)
    {
        // ป้อนอาหารให้ WDT เสมอ (ยกเว้นกรณีตัดสินใจ "ปล่อยให้รีบูต")
        if (!wdt_timeout) wdt_reset();

        // อ่าน/แปลคำสั่งจาก UART
        poll_usart_rx();

        // อ่านระดับน้ำ → map เป็น %
        uint16_t raw = adc_read(WATER_ADC_CH);
        water_percent = (unsigned int)((unsigned long)raw * 100UL / 1023UL);

        // น้ำสูงต่อเนื่อง → lock การนับ + แจ้งบอร์ด 3 (ครั้งเดียว/เหตุการณ์)
        if (water_percent >= WATER_HIGH_PCT)
        {
            if (water_hi_cnt < 10) water_hi_cnt++;     // ต้องสูงติดกันให้ครบก่อน
            if (water_hi_cnt == 10){
                water_lock = true;
                if (!water_hi_sent) { usart0_send_string("WATER:HI\n"); water_hi_sent = true; }
            }
        }
        else if (water_percent <= WATER_LOW_PCT)
        {
            // ลดต่ำกว่า hysteresis → กลับมาทำงานตามปกติ
            water_hi_cnt = 0;
            water_lock   = false;
            water_hi_sent= false;
        }

        // สื่อสารหาย > 30s → แสดง Timeout + กระพริบ D13 + ปล่อยให้ WDT รีบูต
        if ((g_ms - last_serial_ms) > 30000UL)
        {
            lcd.setCursor(0, 1);
            lcd.print("Timeout!        ");
            digitalWrite(LED_TIMEOUT, ((g_ms / 500) % 2) ? HIGH : LOW);
            wdt_timeout = true; // หยุด wdt_reset() → MCU รีบูตภายใน ~2s
        }
        else
        {
            digitalWrite(LED_TIMEOUT, LOW);
        }

        // นับเม็ดยา (อ่าน 3 ครั้งติดกัน < 10cm) + กันนับซ้ำ 150ms/เม็ด — แต่ “หยุดนับ” ถ้า water_lock
        if (!water_lock && have_target)
        {
            float d1 = measure_distance_cm();
            float d2 = measure_distance_cm();
            float d3 = measure_distance_cm();
            if (d1 < 10 && d2 < 10 && d3 < 10)
            {
                if (g_ms > lockout_until_ms)
                {
                    if (pill_count < 999) pill_count++;
                    lockout_until_ms = g_ms + 150; // กันเม็ดเดียวโดนนับซ้ำ
                    update_progress();
                }
            }
        }

        // ส่งสถานะไปบอร์ด 3 ทุก ~200ms เมื่อมี target แล้ว
        if (have_target && (g_ms - last_send_ms) >= 200UL)
        {
            update_progress();
            send_status_to_board3();
        }

        // อัปเดต LCD
        lcd_show_status();

        _delay_ms(50);
    }
    return 0;
}
