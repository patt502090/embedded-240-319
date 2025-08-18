#define F_CPU 16000000UL            // บอกความถี่ MCU ให้ไลบรารี delay รู้
#include <avr/io.h>                 // เข้าถึงรีจิสเตอร์ AVR
#include <util/delay.h>             // ใช้ _delay_ms()
#include <stdint.h>                 // ชนิดข้อมูลมาตรฐาน
#include <stdlib.h>                 // itoa()

#define THRESHOLD 75                // เกณฑ์แจ้งเตือน 75°C

// ----------------- แปลง ADC → °C -----------------
static inline uint8_t convertToC(uint16_t adc_value) {
  // สมมติเซ็นเซอร์ 0–5V = 0–100°C → scale linear
  float voltage = (adc_value * 5.0f) / 1023.0f;  // แปลง ADC เป็นโวลต์
  float tempC   = voltage * 100.0f;              // แปลงเป็นองศา
  if (tempC < 0)   tempC = 0;                    // กันค่าผิดปกติ
  if (tempC > 255) tempC = 255;                  // บีบให้อยู่ใน 8-bit
  return (uint8_t)tempC;                         // คืนค่าเป็น 0..255
}

// ----------------- UART -----------------
static void UART_init(void) {
  // 16 MHz, 9600 bps, U2X=0 → UBRR = 103
  UBRR0H = 0;
  UBRR0L = 103;
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);               // เปิด RX, TX
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);             // 8 data bits, no parity, 1 stop (8N1)
}
static void UART_sendChar(char c) {
  while (!(UCSR0A & (1 << UDRE0))) ;                  // รอ TX buffer ว่าง
  UDR0 = c;                                           // ส่ง 1 ตัวอักษร
}
static void UART_sendString(const char* s) {
  while (*s) UART_sendChar(*s++);                     // ส่งจนเจอ '\0'
}

// ----------------- ADC -----------------
static void ADC_init(void) {
  ADMUX  = (1 << REFS0);                               // Vref = AVcc, เลือกช่องจะตั้งระหว่างอ่าน
  ADCSRA = (1 << ADEN) | (1 << ADPS2)                  // เปิด ADC + prescaler = /128
         | (1 << ADPS1) | (1 << ADPS0);                // (16MHz/128 = 125kHz ตามสเปค)
}
static uint16_t ADC_read(uint8_t ch) {
  ADMUX  = (ADMUX & 0xF0) | (ch & 0x0F);               // เลือกช่อง 0..7 (ที่นี่ใช้ ADC0 = ch=0)
  ADCSRA |= (1 << ADSC);                               // เริ่มแปลง
  while (ADCSRA & (1 << ADSC)) ;                       // รอจนจบ (ADSC=0)
  return ADC;                                          // อ่านผล 10-bit (ADCL ต้องอ่านก่อน ADCH)
}

// ----------------- main -----------------
int main(void) {
  DDRB = 0xFF;                  // PORTB เป็น output (เอาไว้โชว์สถานะด้วย LED)
  PORTB = 0x00;                 // เริ่มต้นดับ

  UART_init();                  // เปิด UART
  ADC_init();                   // เปิด ADC

  char buf[12];                 // บัฟเฟอร์ข้อความ "Temp: xxx"

  while (1) {
    uint16_t adc   = ADC_read(0);               // อ่านจาก ADC0
    uint8_t  tempC = convertToC(adc);           // แปลงเป็น °C (0..100 เรียบง่าย)

    // แสดงสถานะแบบง่าย: เกินเกณฑ์ → จุดทุกดวง, ไม่เกิน → โชว์เป็นบิตแพทเทิร์น
    if (tempC > THRESHOLD) PORTB = 0xFF;        // แจ้งเตือน
    else                   PORTB = tempC;       // ลองแมปค่าไปเป็นลวดลาย LED

    // ส่งข้อความผ่าน UART เช่น "Temp: 27 C\r\n"
    itoa((int)tempC, buf, 10);                  // int → “สตริง” ฐานสิบใน buf
    UART_sendString("Temp: ");
    UART_sendString(buf);
    UART_sendString(" C\r\n");

    _delay_ms(1000);                            // ส่งทุก 1 วินาที
  }
}
