

#include "avr/io.h" library
#define F_CPU 16000000UL       // Define CPU frequency as 16 MHz
#include "util/delay.h"        //library
#include "Arduino.h"           //for Serial communication (if using Arduino IDE)
#define TURN_ALL_LED_OFF 0x00  // Define value to turn off all LEDs
#define ADC5 5                 // Read analog signal from ADC5 pin

#ifdef ARDUINO
#include "Arduino.h"  // Include  only if compiling in Arduino IDE
#endif

unsigned char TABLE7SEG[] =  // การถอดรหัสสำหรับแสดงผลแอลอีดีชนิด 7 ส่วน
  {
    0b00111111,  // รหัสของเลข 0
    0b00000110,  // รหัสของเลข 1
    0b01011011,  // รหัสของเลข 2
    0b01001111,  // รหัสของเลข 3
    0b01100110,  // รหัสของเลข 4
    0b01101101,  // รหัสของเลข 5
    0b01111101,  // รหัสของเลข 6
    0b00000111,  // รหัสของเลข 7
    0b01111111,  // รหัสของเลข 8
    0b01101111,  // รหัสของเลข 9
    0b01110111,  // รหัสของเลข A
    0b01111100,  // รหัสของเลข B
    0b00111001,  // รหัสของเลข C
    0b01011110,  // รหัสของเลข D
    0b01111001,  // รหัสของเลข E
    0b01110001,  // รหัสของเลข F
    0b00000000   // รหัสสำหรับใช้ในแอลอีดีดับทุกดวง
  };

int counter = 0;               // ตัวแปรนับจำนวนการเห็นค่าที่เปลี่ยนใหม่
int adcVal, prevADC, DECODED;  // ประกาศตัวแปร

void DISPLAY7segment(signed char a)  // ฟังก์ชันสำหรับแสดงผลเลขบน 7-segment
{
  PORTD = TABLE7SEG[a] | (1 << PD7);  // แสดงผลค่าที่ถอดรหัสแล้วที่พอร์ต D
}

unsigned int ADC_read(unsigned char a)  // ฟังก์ชันอ่านค่าจาก ADC
{
  ADMUX &= 0xF0;          // เคลียร์บิตช่องเลือก ADC
  ADMUX |= a;             // ตั้งค่าช่องอินพุต ADC ที่ต้องการอ่าน
  ADCSRA |= (1 << ADSC);  // เริ่มแปลงสัญญาณ
  while (!(ADCSRA & (1 << ADIF)))
    ;                   // รอให้การแปลงเสร็จ
  ADCSRA |= 1 << ADIF;  // เคลียร์ธง ADIF
  return ADC;           // คืนค่าผลลัพธ์จากการแปลง
}

int main(void) {
  //Serial.begin(9600);
  DDRC = 0x00;  // ตั้งพอร์ต C เป็น input (อ่านค่า ADC)
  DDRD = 0xFF;  // พอร์ต D ต่อกับ 7-segment LED (output)

  DISPLAY7segment(TURN_ALL_LED_OFF);  // แสดงเลข 0 (ดับ LED)
  ADMUX = 0b01000101;                 // ใช้ AVCC เป็นแรงดันอ้างอิง, อ่านจาก ADC5
  ADCSRA = 0x87;                      // เปิดใช้ ADC และตั้ง prescaler เป็น 128
  DECODED = 0;                        // ค่าเริ่มต้นของตัวถอดรหัส

  while (1)  // วนซ้ำไม่รู้จบ
  {
    counter = 0;
    prevADC = 0;

    do {
      adcVal = ADC_read(ADC5);
      if ((abs(adcVal - prevADC) < 6) && (adcVal > 33))
        counter++;
      else
        counter = 0;
      //Serial.println(adcVal);
      _delay_ms(3);
      prevADC = adcVal;
    } while (counter < 20);
    /*
if ((adcVal > 33) && (adcVal < 65)) DECODED = 1;
else if ((adcVal > 71) && (adcVal < 103)) DECODED = 2;
else if ((adcVal > 114) && (adcVal < 146)) DECODED = 3;
else if ((adcVal > 165) && (adcVal < 213)) DECODED = 4;
else if ((adcVal > 275) && (adcVal < 326)) DECODED = 5;
else if ((adcVal > 375) && (adcVal < 428)) DECODED = 6;
else if ((adcVal > 465) && (adcVal < 581)) DECODED = 7;
else if ((adcVal > 600) && (adcVal < 690)) DECODED = 8;
else if ((adcVal > 701) && (adcVal < 780)) DECODED = 9;
else if ((adcVal > 785) && (adcVal < 860)) DECODED = 10;
else if ((adcVal > 875) && (adcVal < 900)) DECODED = 0;
else if ((adcVal > 910) && (adcVal < 1023)) DECODED = 11;
*/

    if ((adcVal > 33) && (adcVal < 60))
      DECODED = 1;
    else if ((adcVal > 62) && (adcVal < 100))
      DECODED = 2;
    else if ((adcVal > 100) && (adcVal < 135))
      DECODED = 3;
    else if ((adcVal > 140) && (adcVal < 170))
      DECODED = 10;

    else if ((adcVal > 175) && (adcVal < 220))
      DECODED = 4;
    else if ((adcVal > 294) && (adcVal < 326))
      DECODED = 5;
    else if ((adcVal > 225) && (adcVal < 289))
      DECODED = 6;
    else if ((adcVal > 410) && (adcVal < 473))
      DECODED = 11;

    else if ((adcVal > 480) && (adcVal < 581))
      DECODED = 7;
    else if ((adcVal > 660) && (adcVal < 670))
      DECODED = 8;
    else if ((adcVal > 330) && (adcVal < 405))
      DECODED = 9;
    else if ((adcVal > 630) && (adcVal < 675))
      DECODED = 12;

    else if ((adcVal > 843) && (adcVal < 884))  //E
      DECODED = 14;
    else if ((adcVal > 901) && (adcVal < 936))
      DECODED = 0;
    else if ((adcVal > 946) && (adcVal < 971))  //F
      DECODED = 15;
    else if ((adcVal > 972) && (adcVal < 987))  //D
      DECODED = 13;
    //-----------

    do {
      adcVal = ADC_read(ADC5);
    } while (adcVal > 33);

    DISPLAY7segment(DECODED);
  }
}
