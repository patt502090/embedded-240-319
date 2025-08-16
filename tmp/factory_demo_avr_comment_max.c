/*
================================================================================
 Project  : Pharma Factory Demo (7 Blocks) — AVR C (Register-level, MAX Comments)
 MCU      : ATmega328P @ 16 MHz  (e.g., Arduino UNO hardware but using pure C)
 Author   : You (exam-ready skeleton)
 Language : C (avr-gcc), direct register access (NO Arduino framework)

 HOW TO READ THIS FILE
 ---------------------
 This file is intentionally "over-commented" for open-book exams. It explains:
   • WHAT each block does (Timer, ADC, Keypad, LCD, USART, INT0 counter, WDT)
   • WHY we set specific bits in registers
   • HOW signals/pins are wired (Board role + signal direction)
   • WHERE to change parameters (thresholds, baudrate, etc.)
   • STATE MACHINE that ties everything together

 BOARD ROLES (single-board version)
 ----------------------------------
 • This code is meant to run on the **Controller/HMI + Dispenser Board**.
   - Responsibilities:
       [ADC] Read water level sensor on PC0 (ADC0), convert to %.
       [Keypad] Read target pill count via matrix keypad.
       [LCD] Display current water %, status (LOW/OK/HIGH), pill count, etc.
       [INT0] Count pills passing an IR slot sensor (one pulse per pill).
       [MOTOR] Control a motor/solenoid via PB0 (relay or driver input).
       [USART] Periodically log status to PC (9600 8N1).
       [Timer1] Provide a 1 ms "system tick" (non-blocking timing style).
       [WDT] Keep the system from hanging (watchdog reset if main loop stalls).
 • In a multi-board exam variant:
   - A "Sensor Node" board could do only ADC and send values via I²C/UART.
   - This board (HMI) would then read that value and still display/control.
   - The code here stays valid; you would disable ADC here and read from comms.

 PIN MAPPING (Controller/HMI + Dispenser Board)
 ----------------------------------------------
 * Water level analog sensor (0–5 V) -> PC0 (ADC0). (Ensure Vref=AVcc=5 V)
 * Keypad 3x4:
     ROW = PD3..PD0 (output, drive LOW one-by-one when scanning)
     COL = PC3..PC0 (input with internal pull-ups; key press -> COL reads LOW)
 * LCD 16x2 (HD44780-compatible), 4-bit mode:
     RS = PB1, EN = PB2, D4..D7 = PC4..PC7
 * Pill counter sensor (IR slot) -> PD2 (INT0). Output HIGH/LOW pulses per pill.
 * Motor/Relay control -> PB0 (digital output). HIGH=ON, LOW=OFF (you may invert).
 * USART 9600 8N1 -> PD1 (TX) to PC/USB-UART adapter, PD0 (RX) unused in this demo.
 * System clock: external 16 MHz (Arduino UNO style).

 CHAPTER-TO-BLOCK MAPPING (matches your course topics)
 -----------------------------------------------------
  [1] Timer/Counter (Normal & CTC mode)            -> Timer1 CTC 1ms tick (non-blocking)
  [2] ADC                                           -> ADC0 (PC0), 10-bit, Vref=AVcc
  [3] Interfacing technique (Keypad matrix)         -> 3x4 keypad scan with pull-ups
  [4] Interrupt of AVR                              -> ISR(TIMER1_COMPA_vect), ISR(INT0_vect)
  [5] Asynchronous Serial Communication (USART)     -> 9600 8N1 TX logs
  [6] Watchdog Timer                                -> WDTO_1S, wdt_reset() in main loop
  [7] LCD (HD44780, 4-bit)                          -> Minimal 4-bit driver

 BUILD (avr-gcc)
 ----------------
   avr-gcc -mmcu=atmega328p -Os main.c -o main.elf
   avr-objcopy -O ihex main.elf main.hex
   (flash with avrdude as usual)

================================================================================
*/

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdbool.h>
#include <stdlib.h>

/* ============================================================================
   [GLOBAL STATE] — shared between ISRs and main loop
   ----------------------------------------------------------------------------
   g_ms      : 1 ms system tick (incremented by Timer1 CTC ISR)
   g_pill    : pill count (incremented by INT0 ISR on each valid pulse)
   g_last_ms : last tick when a valid INT0 pulse was accepted (for debouncing)
============================================================================ */
volatile unsigned long g_ms      = 0;
volatile unsigned int  g_pill    = 0;
volatile unsigned long g_last_ms = 0;

/* ============================================================================
   [1] TIMER/COUNTER — Timer1 in CTC mode to create a 1 ms "system tick"
   ----------------------------------------------------------------------------
   Why Timer1 CTC?
     - We need a periodic timebase to do non-blocking tasks (poll ADC/LCD/USART).
     - CTC (Clear Timer on Compare Match) automatically resets the counter to 0
       when it matches OCR1A, giving precise intervals without manual reset.
   Formula (from datasheet):
     f_compare = F_CPU / (N * (1 + OCR1A))
   Choose:
     F_CPU = 16 MHz, want 1 kHz (1 ms), choose prescaler N = 64
       => OCR1A = 16e6 / (64 * 1000) - 1 = 249
   Registers/bits:
     TCCR1A: [unused here] mode bits WGM11:0=0 for CTC via TCCR1B.WGM12=1
     TCCR1B: WGM12=1 (CTC), CS11=1 & CS10=1 (prescaler = 64)
     OCR1A : 249 (compare value for 1 ms)
     TIMSK1: OCIE1A=1 enable compare A interrupt
============================================================================ */
static void timer1_init_1ms(void){
  TCCR1A = 0;                                   // ensure normal port operation
  TCCR1B = (1<<WGM12) | (1<<CS11) | (1<<CS10);  // WGM12=1: CTC, CS11:CS10=1:1 => N=64
  OCR1A  = 249;                                 // compare value for 1 ms tick
  TIMSK1 = (1<<OCIE1A);                         // enable interrupt on compare A
}
ISR(TIMER1_COMPA_vect){
  g_ms++;  // increment global millisecond tick
}

/* ============================================================================
   [2] ADC — Read water level at ADC0 (PC0) with Vref=AVcc, 10-bit
   ----------------------------------------------------------------------------
   Why prescaler=128?
     - ADC clock must be ~50–200 kHz (datasheet). With F_CPU=16 MHz:
       16 MHz / 128 = 125 kHz -> inside spec -> good trade-off.
   Registers/bits:
     ADMUX  : REFS0=1 (Vref=AVcc), MUX3..0 selects channel (ADC0..ADC7)
     ADCSRA : ADEN=1 enable ADC, ADPS2..0 set prescaler (128), ADSC start conv.
     ADC    : 16-bit register (ADCL must be read first, then ADCH)
============================================================================ */
static void adc_init(void){
  ADMUX  = (1<<REFS0);                                        // Vref=AVcc, MUX=0000 (ADC0)
  ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);  // enable + prescaler 128
}
static uint16_t adc_read(uint8_t ch){
  ADMUX = (ADMUX & 0xF0) | (ch & 0x0F); // set channel 0..7 (keep REFS bits)
  ADCSRA |= (1<<ADSC);                  // start single conversion
  while (ADCSRA & (1<<ADSC));           // wait until conversion completes
  return ADC;                            // read 10-bit result
}

/* ============================================================================
   [3] KEYPAD 3x4 — Matrix scan with internal pull-ups
   ----------------------------------------------------------------------------
   Wiring (Controller Board):
     ROW = PD3..PD0 (outputs) — we drive one row LOW at a time
     COL = PC3..PC0 (inputs)  — enable pull-ups; pressed key pulls the line LOW
   Scan algorithm:
     for each row r:
       - set that row LOW, others HIGH
       - small delay to settle
       - read COL pins (inverted); if any LOW -> that key is pressed
   Debounce:
     - For exam brevity we do a simple edge check in app code. You can add a
       20–30 ms guard in main loop or use the systick to time it.
============================================================================ */
static const char KP_MAP[4][4] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
static void keypad_init(void){
  // ROW (PD3..PD0) as outputs, idle HIGH
  DDRD  |= 0x0F;   // set PD0..PD3 to outputs
  PORTD |= 0x0F;   // drive all HIGH as default
  // COL (PC3..PC0) as inputs with internal pull-ups
  DDRC  &= ~0x0F;  // clear PC0..PC3 direction (input)
  PORTC |=  0x0F;  // enable pull-ups (so idle reads HIGH)
}
static char keypad_getkey(void){
  for(uint8_t r=0; r<4; r++){
    // drive one row LOW, others HIGH
    PORTD |= 0x0F;            // set all HIGH
    PORTD &= ~(1<<r);         // clear the bit for this row (LOW)
    _delay_us(5);             // settle (RC effects)
    uint8_t col = ~PINC & 0x0F; // read cols, invert because active-low
    if (col){
      for(uint8_t c=0; c<4; c++){
        if (col & (1<<c)) return KP_MAP[r][c];
      }
    }
  }
  return 0; // no key
}

/* ============================================================================
   [4] LCD 16x2 (HD44780) — 4-bit interface
   ----------------------------------------------------------------------------
   Wiring (Controller Board):
     RS=PB1, EN=PB2
     D4..D7 = PC4..PC7
   Protocol:
     - To send a command or data byte, split into high nibble then low nibble.
     - Put nibble on PC4..PC7, toggle EN (falling edge latches data).
     - RS=0 -> command, RS=1 -> data.
   Initialization sequence follows datasheet for 4-bit mode.
============================================================================ */
#define LCD_RS  PB1
#define LCD_EN  PB2
static void lcd_gpio_init(void){
  DDRB |= (1<<LCD_RS) | (1<<LCD_EN) | (1<<PB0); // PB0 also used for motor output
  DDRC |= 0xF0;                                  // PC4..PC7 as outputs for data
  PORTB &= ~((1<<LCD_RS)|(1<<LCD_EN));           // ensure RS=0, EN=0
}
static void lcd_pulse(void){
  PORTB |=  (1<<LCD_EN);     // EN=1
  _delay_us(1);
  PORTB &= ~(1<<LCD_EN);     // EN=0 -> latch on falling edge
  _delay_us(50);             // command settling
}
static void lcd_write4(uint8_t nib){
  // Put 4-bit data on PC4..PC7
  PORTC = (PORTC & 0x0F) | (nib << 4);
  lcd_pulse();
}
static void lcd_cmd(uint8_t c){
  PORTB &= ~(1<<LCD_RS);       // RS=0 (command)
  lcd_write4(c >> 4);
  lcd_write4(c & 0x0F);
  if (c == 0x01 || c == 0x02)  // clear/home need >1.5 ms
    _delay_ms(2);
}
static void lcd_data(uint8_t d){
  PORTB |= (1<<LCD_RS);        // RS=1 (data)
  lcd_write4(d >> 4);
  lcd_write4(d & 0x0F);
}
static void lcd_init(void){
  lcd_gpio_init();
  _delay_ms(40);        // power-up wait
  // 8-bit init pulses (sent as 4-bit high nibbles) per datasheet
  lcd_write4(0x03); _delay_ms(5);
  lcd_write4(0x03); _delay_us(150);
  lcd_write4(0x03); _delay_us(150);
  lcd_write4(0x02); // switch to 4-bit mode
  // Function set: 4-bit, 2 lines, 5x8 dots
  lcd_cmd(0x28);
  // Display on, cursor off, blink off
  lcd_cmd(0x0C);
  // Entry mode: increment, no shift
  lcd_cmd(0x06);
  // Clear display
  lcd_cmd(0x01);
}
static void lcd_gotoxy(uint8_t x,uint8_t y){
  uint8_t addr = (y ? 0x40 : 0x00) + x;
  lcd_cmd(0x80 | addr);
}
static void lcd_print(const char*s){ while(*s) lcd_data(*s++); }

/* ============================================================================
   [5] USART — Asynchronous 9600 8N1 (TX only for logging)
   ----------------------------------------------------------------------------
   Baudrate formula (normal mode, U2X=0):
     UBRR = F_CPU/(16*baud) - 1  ->  16e6/(16*9600) - 1 ≈ 103
   Registers/bits:
     UBRR0H/L : set baud rate
     UCSR0C   : UCSZ01:0=11 -> 8-bit, USBS0=0 -> 1 stop, UPM01:0=00 -> no parity
     UCSR0B   : TXEN0=1 enable transmitter (RXEN0 optional; not used here)
     UCSR0A   : UDRE0 indicates data register empty (ready for next byte)
============================================================================ */
static void usart_init_9600(void){
  UCSR0A = 0x00;             // normal speed (U2X0=0)
  UBRR0H = 0x00; UBRR0L = 103; // 9600 bps @16 MHz
  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); // 8N1
  UCSR0B = (1<<TXEN0);       // enable TX only
}
static void usart_tx(char c){
  while(!(UCSR0A & (1<<UDRE0))); // wait until transmit buffer is empty
  UDR0 = c;
}
static void usart_print(const char*s){ while(*s) usart_tx(*s++); }

/* ============================================================================
   [6] EXTERNAL INTERRUPT (INT0) — Count pills with debouncing
   ----------------------------------------------------------------------------
   Hardware:
     - IR slot sensor output connected to PD2 (INT0). Each pill produces a pulse.
     - Ensure sensor outputs clean edges; else add RC or comparator.
   Debounce policy:
     - Ignore pulses that occur within 5 ms from previous valid pulse.
       (prevents double-counting due to mechanical jitter/reflections)
   Registers/bits:
     EICRA: ISC01=1 & ISC00=1 -> trigger on RISING edge
     EIMSK: INT0=1 enable external interrupt 0
     EIFR : INTF0=1 can clear pending flag before enabling
============================================================================ */
static void int0_init_rising(void){
  DDRD  &= ~(1<<PD2);   // PD2 (INT0) as input
  PORTD |=  (1<<PD2);   // enable pull-up (optional depending on sensor)
  EICRA  =  (1<<ISC01) | (1<<ISC00);  // rising edge
  EIFR   =  (1<<INTF0);               // clear any pending flag
  EIMSK  =  (1<<INT0);                // enable INT0
}
ISR(INT0_vect){
  unsigned long now = g_ms;         // current time in ms
  if (now - g_last_ms > 5) {        // accept only if >5 ms since last pulse
    g_pill++;                       // count one pill
    g_last_ms = now;                // update last accepted time
  }
}

/* ============================================================================
   [7] WATCHDOG TIMER — ~1 s timeout (auto-reset if main loop stalls)
   ----------------------------------------------------------------------------
   Usage:
     - Call wdt_enable(WDTO_1S) once in init.
     - In the main loop, call wdt_reset() regularly (e.g., each iteration).
   Notes:
     - If code locks up and fails to reset WDT within ~1 s, MCU will reset.
============================================================================ */
static void wdt_init_1s(void){ wdt_enable(WDTO_1S); }

/* ============================================================================
   Helpers — non-blocking timer check using g_ms
============================================================================ */
static bool elapsed_ms(unsigned long *t0, unsigned long dt){
  if ((g_ms - *t0) >= dt) { *t0 = g_ms; return true; }
  return false;
}

/* ============================================================================
   APPLICATION STATE MACHINE (ties all blocks together)
   ----------------------------------------------------------------------------
   States:
     S_IDLE        : Wait for any key to begin.
     S_WAIT_WATER  : Read ADC -> compute percent & status; proceed if acceptable.
     S_INPUT       : Enter target pill count via keypad. '#' to confirm, '*' clear.
     S_DISPENSE    : Turn on motor; count pills via INT0; stop when target reached.
     S_DONE        : Display result; log via USART; wait for key to go back to IDLE.
     S_ERROR       : (optional) For timeouts/sensor error (not fully used here).
============================================================================ */
typedef enum { S_IDLE, S_WAIT_WATER, S_INPUT, S_DISPENSE, S_DONE, S_ERROR } state_t;

int main(void){
  /* -------------------- Global/Peripheral Initialization ------------------ */
  cli();                    // disable interrupts during setup
  timer1_init_1ms();        // [1] system tick
  adc_init();               // [2] water level input
  keypad_init();            // [3] keypad matrix
  lcd_init();               // [4] LCD interface
  usart_init_9600();        // [5] serial logs
  int0_init_rising();       // [6] pill counter
  wdt_init_1s();            // [7] watchdog
  sei();                    // enable global interrupts

  /* Motor/Relay output on PB0 (Controller Board) */
  // - HIGH = ON (assumed). If your relay board is active-low, invert logic.
  PORTB &= ~(1<<PB0);       // ensure OFF at startup

  /* Parameters (thresholds) */
  const uint8_t TH_LOW  = 30;   // percent considered "LOW"
  const uint8_t TH_HIGH = 70;   // percent considered "HIGH" (OK if between)

  /* Runtime variables */
  uint16_t target_pills = 0;    // target count entered by user
  unsigned long t_adc = 0;      // last ADC update time
  unsigned long t_log = 0;      // last log time
  state_t st = S_IDLE;

  /* Splash screen & first log */
  lcd_print("Ready ");          // show "Ready " on power-up
  usart_print("Start\r\n");

  /* ------------------------------- Main Loop ------------------------------ */
  for(;;){
    wdt_reset();  // IMPORTANT: keep petting the watchdog in the loop

    switch(st){
      case S_IDLE: {
        // Waiting for any keypress to start workflow
        char k = keypad_getkey();
        if (k){
          st = S_WAIT_WATER;
          lcd_cmd(0x01);                // clear display
          lcd_print("Check water...");
        }
      } break;

      case S_WAIT_WATER: {
        // Periodically read water level and display percentage & status.
        if (elapsed_ms(&t_adc, 100)){   // every 100 ms
          uint16_t raw = adc_read(0);   // ADC0 (PC0)
          // Scale 10-bit value 0..1023 to 0..100%
          uint8_t  p   = (uint32_t)raw * 100 / 1023;

          // LCD display: first line shows W=xx%
          lcd_gotoxy(0,1);
          lcd_print("W=");
          lcd_data('0' + (p/10));       // tens
          lcd_data('0' + (p%10));       // ones
          lcd_data('%');
          lcd_print("   ");

          // Status: LOW / OK / HIGH based on thresholds
          lcd_gotoxy(8,1);
          if (p < TH_LOW)        lcd_print("LOW ");
          else if (p > TH_HIGH)  { lcd_print("HIGH"); st = S_INPUT; lcd_cmd(0x01); }
          else                   { lcd_print("OK  "); st = S_INPUT; lcd_cmd(0x01); }
        }
        // If exam asks for timeout/error, add a timer and goto S_ERROR if too long.
      } break;

      case S_INPUT: {
        // Enter desired pill count with keypad: digits to append, '*' to clear, '#' to confirm.
        static uint16_t acc = 0;
        lcd_gotoxy(0,0); lcd_print("Target pills:");
        lcd_gotoxy(0,1);

        char k = keypad_getkey();
        if (k){
          if (k>='0' && k<='9'){
            acc = acc*10 + (k-'0');        // append digit
          } else if (k=='*'){
            acc = 0;                        // clear buffer
          } else if (k=='#'){
            target_pills = acc;             // confirm target
            acc = 0;
            g_pill = 0;                     // reset counter
            PORTB |= (1<<PB0);              // motor ON
            st = S_DISPENSE;
            lcd_cmd(0x01);                  // clear for next screen
          }
        }

        // Show current entry
        lcd_gotoxy(13,0); lcd_data(' ');    // small cleanup
        char buf[6]; itoa(acc, buf, 10);
        lcd_gotoxy(0,1); lcd_print("   ");  // clear line start
        lcd_gotoxy(0,1); lcd_print(buf);
      } break;

      case S_DISPENSE: {
        // Motor is ON, INT0 ISR counts pills. Stop when target reached.
        lcd_gotoxy(0,0); lcd_print("Dispensing...");
        lcd_gotoxy(0,1); lcd_print("CNT=");
        char cb[8]; itoa(g_pill, cb, 10); lcd_print(cb); lcd_print("    ");

        if (g_pill >= target_pills){
          PORTB &= ~(1<<PB0); // motor OFF
          st = S_DONE;
        }
        // Optional: add timeout using g_ms to detect jam -> st = S_ERROR
      } break;

      case S_DONE: {
        // Show final result and periodically log to USART
        lcd_cmd(0x01);
        lcd_print("DONE. Pills=");
        char b[8]; itoa(g_pill, b, 10); lcd_print(b);

        if (elapsed_ms(&t_log, 1000)){
          usart_print("DONE\r\n");
        }

        // Any key returns to IDLE
        if (keypad_getkey()){
          st = S_IDLE;
          lcd_cmd(0x01);
          lcd_print("Ready ");
        }
      } break;

      case S_ERROR: {
        // Error handling (not fully used in this demo)
        PORTB &= ~(1<<PB0);     // ensure motor OFF
        lcd_cmd(0x01); lcd_print("ERROR!");
        if (elapsed_ms(&t_log, 1000)) usart_print("ERROR\r\n");
        if (keypad_getkey()){ st = S_IDLE; lcd_cmd(0x01); lcd_print("Ready "); }
      } break;
    }

    // Periodic log to USART: water % and pill count every 1 s
    if (elapsed_ms(&t_log, 1000)){
      uint16_t raw = adc_read(0);
      uint8_t  p   = (uint32_t)raw * 100 / 1023;
      usart_print("W,");
      usart_tx('0' + (p/10));
      usart_tx('0' + (p%10));
      usart_print("%  CNT=");
      char cb[8]; itoa(g_pill, cb, 10); usart_print(cb);
      usart_print("\r\n");
    }
  }
}
