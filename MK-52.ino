/* сопряжение программируемого калькулятора Электроника МК 52 с компьютером
 * ниже структура кольцевой оперативной памяти калькулятора
 *   07    06    05    04    03    02    01    00     0E    0D    0C    0B    0A    09    08 
 * +-----+-----+-----+-----+-----+-----+-----+-----+------+-----+-----+-----+-----+-----+-----+
 * |  R7 |  R6 |  R5 |  R4 |  R3 |  R2 |  R1 |  R0 |  Re  |  Rd |  Rc |  Rb |  Ra |  R9 |  R8 |  M1 (регистры R0...Ra)
 * +-----+-----+-----+-----+-----+-----+-----+-----+------+-----+-----+-----+-----+-----+-----+
 * |     |     |     |     |     |     |     |     |  T   |  Z  |  Y  |  X  |  X1 |  КС  метка|  M2
 * +-----+-----+-----+-----+-----+-----+-----+-----+------+-----+-----+-----+-----+-----+-----+
 *  50-56 43-49 36-42 29-35 22-28 15-21 08-14 01-07 99-105 92-98 85-91 78-84 71-77 64-70 57-63|  M3 (память программ) в ячейках указаны шаги программы  
 * +-----+-----+-----+-----+-----+-----+-----+-----+------+-----+-----+-----+-----+-----+-----+
 *    
 * В кольцевой оперативной памяти 3 блока памяти М1, М2, М3. В каждом блоке 15 страниц памяти.
 * Всего 15 х 3 = 45 страниц памяти. В каждой странице памяти 14 тетрад. В каждой тетраде 4 бита.
 * Всего 45 х 14 х 4 = 2520 бит = 315 байт.
 * Данные по кольцу передаются так: 1-я тетрада блока М1, 1-я тетрада блока М2, 1-я тетрада блока М3,
 * 2-я тетрада блока М1, 2-я тетрада блока М2, 2-я тетрада блока М3 и т.д.
 * Сразу после включения 1-е страницы буфера будут выглядеть так: 
 *  M1  M2  M3   M1  M2  M3   M1  M2  M3   M1  M2  M3   M1  M2  M3   M1  M2  M3   M1  M2  M3
 * 000011110000 000011110000 000011110000 000011110000 000011110000 000011110000 000011110000
 * 000011110000 000011110000 000000000000 000000000000 000000000000 000000000000 000000000000
 * (пробелы поставлены для визуального разделения троек тетрад)
 * M1 | 00000000000000
 * M2 | FFFFFFFFF00000
 * M3 | 00000000000000
 * Код "метка" отображается на начальной странице блока М2 цифрами F в первых 9 тетрадах,
 * что и показано на последовательности битов вверху. 
 *
 * В памяти программ команды располагаются в такой последовательности (на примере страницы 0):
 * 02 03 04 05 06 07 01
 *    
 */

///#define DEBUG   //закомментировать, если не нужна отладочная информация
#include "MKCALC.h"
#include <avr/interrupt.h>
#include <SPI.h>
#include <Arduino.h>

//volatile boolean process_it;
MKCALC mk52 = MKCALC();

// SPI interrupt routine
ISR(SPI_STC_vect) {
  mk52.interruptSPI();
}  // end of interrupt routine SPI_STC_vect

void setup() {
  Serial.begin(115200, SERIAL_8N1);
  mk52.setSerial(&Serial);
  TIMSK0 = 0;

  pinMode( 8, OUTPUT);      // pin PB0
  pinMode( 9, OUTPUT);      // pin PB1
  //pinMode(12, OUTPUT);      // pin PB4

  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(CPOL);
  SPCR |= _BV(CPHA);
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  // turn on interrupts
  SPCR |= _BV(SPIE);

}

void loop() {
/*
  switch (*mk52.cmd_state) { 
    case WAIT_A_START:
      //Serial.print(".");
      break;
    case WAIT_A_MARK:
      //Serial.print("+");
      break;
    case READ_BITS:
      //Serial.println("READ_BITS");
      break;
  }
*/
  if (Serial.available() > 0) {
    switch (Serial.read()) {
      case '1':
        Serial.println("ArduinoNano");
        Serial.print("cmd_state ");
        Serial.println((mk52.getState() + 9)%3);
        Serial.println(mk52.getState());
        break;
      case '2':
        Serial.println("SPI //SS = 1");
        mk52.commandT = 1;
        break;
      case 'r':
        mk52.setState(WAIT_A_START);
        break;
      case 's':
        mk52.clrInterrupt();
        mk52.MemoryPagesPrint();
        break;
      case 'm':
        mk52.clrInterrupt();
        mk52.MemoryPagesPrint();
        mk52.setInterrupt();
        mk52.setState(WAIT_A_START);
        break;
      case 'd':
        Serial.println("ringToMem - start");
        mk52.ringToMem();
        mk52.MemoryPagesPrint();
        Serial.println("\r\nringToMem - stop");
        break;
      case 'f':
        Serial.println("ringToMemNoInterrupt - start");
        mk52.ringToMemNoInterrupt();
        mk52.MemoryPagesPrint();
        Serial.println("\r\nringToMemNoInterrupt - stop");
        break;
      case 'u':
        PORTD |= (1 << 2);
        PORTD |= (1 << 3);
        PORTD |= (1 << 4);
        Serial.println("\r\nON  pull-up");
        break;
      case 'i':
        PORTD &= ~(1 << 2);
        PORTD &= ~(1 << 3);
        PORTD &= ~(1 << 4);
        Serial.println("\r\nOFF pull-up");
        break;
    }
  }   
}
