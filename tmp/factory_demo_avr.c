// factory_demo_avr.c
// ATmega328P @ 16 MHz (e.g., Arduino UNO hardware), build with avr-gcc
// Features: ADC (water level), Timer1 CTC + ISR (100 Hz scheduler),
// USART 9600 8N1 (status), PWM on OC0A (PB7 on some parts / on UNO OC0A = D6 / OC0B = D5),
// watchdog, simple 3x4 keypad scan (rows PD2..PD5, cols PC0..PC2).
// Adjust pinout to your board.

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <stdlib.h>

// -------- Pins --------
#define KP_ROW_PORT  PORTD
#define KP_ROW_DDR   DDRD
#define KP_ROW_PINR  PIND
#define KP_R0        PD2
#define KP_R1        PD3
#define KP_R2        PD4
#define KP_R3        PD5

#define KP_COL_PORT  PORTC
#define KP_COL_DDR   DDRC
#define KP_COL_PINR  PINC
#define KP_C0        PC0
#define KP_C1        PC1
#define KP_C2        PC2

// PWM output on OC0A -> PD6 on ATmega328P (Arduino D6)
#define PUMP_PWM_DDR DDRD
#define PUMP_PWM_PIN PD6

// LED heartbeat on PB5 (Arduino D13)
#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_PIN  PB5

// Water sensor on ADC0 (PC0) - NOTE: shares with KP_C0 if you use the same pin.
// For clarity keep ADC on ADC1 (PC1) to avoid clash.
#define ADC_CH  1   // use ADC1 (PC1)

// -------- Globals --------
volatile uint32_t ticks = 0;
volatile uint8_t  tickFlag = 0;

static const char keymap[4][3] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

// -------- USART --------
static void usart_init(uint32_t baud){
  // UBRR = F_CPU/(16*baud) - 1
  uint16_t ubrr = (F_CPU/(16UL*baud)) - 1;
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)(ubrr & 0xFF);
  UCSR0B = (1<<TXEN0);                 // enable TX (RX optional)
  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);  // 8N1
}
static void usart_tx(char c){
  while(!(UCSR0A & (1<<UDRE0)));
  UDR0 = c;
}
static void usart_print(const char* s){
  while(*s) usart_tx(*s++);
}
static void usart_print_u16(uint16_t v){
  char buf[8]; itoa(v, buf, 10); usart_print(buf);
}

// -------- ADC (single conversion) --------
static void adc_init(void){
  ADMUX = (1<<REFS0) | (ADC_CH & 0x0F);         // AVcc ref, select channel
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // prescale 128 -> 125 kHz
}
static uint16_t adc_read_blocking(uint8_t ch){
  ADMUX = (ADMUX & 0xF0) | (ch & 0x0F);
  ADCSRA |= (1<<ADSC);
  while (ADCSRA & (1<<ADSC));
  return ADC;
}

// -------- Timer1 CTC 100 Hz scheduler --------
static void timer1_init_ctc_100hz(void){
  // f = F_CPU / (N * (1 + OCR1A))
  // choose N=64 => OCR1A = F_CPU/(64*100) - 1 = 2499
  TCCR1A = 0;
  TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10);  // CTC, prescale 64
  OCR1A  = 2499;
  TIMSK1 = (1<<OCIE1A); // enable compare A interrupt
}

// -------- PWM on OC0A (fast PWM) --------
static void pwm0_init(void){
  // OC0A (PD6) as output
  PUMP_PWM_DDR |= (1<<PUMP_PWM_PIN);
  // Fast PWM, non-inverting on OC0A, prescale 64
  TCCR0A = (1<<COM0A1) | (1<<WGM01) | (1<<WGM00);
  TCCR0B = (1<<CS01) | (1<<CS00); // prescale 64
  OCR0A  = 0; // duty 0%
}

// -------- Keypad scan (rows out low one-by-one, columns pull-up) --------
static void keypad_init(void){
  // Rows as outputs (start high), Cols as inputs with pull-ups
  KP_ROW_DDR  |= (1<<KP_R0)|(1<<KP_R1)|(1<<KP_R2)|(1<<KP_R3);
  KP_ROW_PORT |= (1<<KP_R0)|(1<<KP_R1)|(1<<KP_R2)|(1<<KP_R3);
  KP_COL_DDR  &= ~((1<<KP_C0)|(1<<KP_C1)|(1<<KP_C2));
  KP_COL_PORT |=  ((1<<KP_C0)|(1<<KP_C1)|(1<<KP_C2)); // enable pull-ups
}
static char keypad_getkey(void){
  for (uint8_t r=0; r<4; r++){
    // drive one row low, others high
    KP_ROW_PORT |= (1<<KP_R0)|(1<<KP_R1)|(1<<KP_R2)|(1<<KP_R3);
    KP_ROW_PORT &= ~(1<<(KP_R0 + r)); // assumes consecutive bits PD2..PD5
    _delay_us(5);
    uint8_t cols = KP_COL_PINR;
    if (!(cols & (1<<KP_C0))) return keymap[r][0];
    if (!(cols & (1<<KP_C1))) return keymap[r][1];
    if (!(cols & (1<<KP_C2))) return keymap[r][2];
  }
  return 0;
}

// -------- ISR --------
ISR(TIMER1_COMPA_vect){
  ticks++;
  if ((ticks % 100) == 0) { // every 1s at 100Hz
    tickFlag = 1;
  }
}

// -------- Main --------
int main(void){
  // IO
  LED_DDR |= (1<<LED_PIN);

  // subsystems
  usart_init(9600);
  adc_init();
  pwm0_init();
  keypad_init();
  timer1_init_ctc_100hz();
  wdt_enable(WDTO_2S);

  sei();

  uint16_t setpoint = 600;
  uint16_t adcVal   = 0;
  uint8_t  lastKey  = 0;
  char     entry[6]; uint8_t idx=0;

  usart_print("Factory demo (AVR C)\r\nEnter setpoint (0..1023) then '#'\r\n");

  for(;;){
    // keypad (poll every loop; debouncing kept simple)
    char k = keypad_getkey();
    if (k && k != lastKey){ // edge
      lastKey = k;
      if (k>='0' && k<='9'){
        if (idx<5){ entry[idx++] = k; entry[idx]=0; }
        usart_tx(k);
      } else if (k=='#'){
        if (idx){
          int v = atoi(entry);
          if (v<0) v=0; if (v>1023) v=1023;
          setpoint = (uint16_t)v;
          idx=0; entry[0]=0;
          usart_print(" -> setpoint="); usart_print_u16(setpoint); usart_print("\r\n");
        }
      } else if (k=='*'){
        idx=0; entry[0]=0;
        usart_print("\r\nCleared\r\n");
      }
    } else if (!k) {
      lastKey = 0;
    }

    // sample ADC channel
    adcVal = adc_read_blocking(ADC_CH);

    // simple control to compute PWM
    int16_t error = (int16_t)setpoint - (int16_t)adcVal;
    uint8_t duty = 0;
    if (error > 0){
      if (error > 512) error = 512;
      duty = (uint8_t)((uint16_t)error * 255U / 512U);
    } else {
      duty = 0;
    }
    OCR0A = duty;

    // periodic status each second
    if (tickFlag){
      tickFlag = 0;
      LED_PORT ^= (1<<LED_PIN);
      usart_print("ADC="); usart_print_u16(adcVal);
      usart_print(" SP=");  usart_print_u16(setpoint);
      usart_print(" PWM="); usart_print_u16(duty);
      usart_print("\r\n");
    }

    // pet watchdog
    wdt_reset();
  }
}
