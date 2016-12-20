/***************************************************
 * MKCALC.cpp - Core of MKCalk library for linking *
 *              MK-52/61 calculators and Arduino.  *
 *              Created by _______________________ *
 *              2016-_____________________________ *
 ***************************************************/

#include "Arduino.h"
#include "MKCALC.h"

volatile byte MKCALC::cmd_state = WAIT_A_START;
volatile uint8_t MKCALC::RAMdata[315];
volatile uint16_t MKCALC::posRAM = 0;
volatile uint8_t MKCALC::skipClockCycle = 0;
volatile uint8_t MKCALC::tempOut = 0;
volatile uint8_t MKCALC::byteWriteToMkStatus = 0;
volatile uint8_t MKCALC::byteToMk = 0xAA;
volatile uint16_t MKCALC::byteNumToMk = 13;
volatile uint8_t MKCALC::fixCounter = 0x00;

MKCALC::MKCALC() {
  //serial_ = NULL;
  cmd_state = WAIT_A_START;
}

void MKCALC::setSerial(HardwareSerial* serial) {
  serial_ = serial;
}

byte MKCALC::readTetrad(uint16_t ind, volatile uint8_t* memDump) {
  //See: Arduino/hardware/tools/avr/avr/include/avr/builtins.h
  return 0x0F & (((ind & 0x01) == 1) ? (memDump[ind >> 1]) : (__builtin_avr_swap(memDump[ind >> 1])));
}

#define TOREALIND(indRAM) ((indRAM >= 315) ? (indRAM - 315) : (indRAM))
void MKCALC::interruptSPI() {
  PORTD |= (1 << 6);
  //if (cmd_state == READ_BITS) SPDR = __builtin_avr_swap(RAMdata[(posRAM == 314) ? (0) : (posRAM + 1)]); //Set data to send by SPI
  if (cmd_state == READ_BITS) SPDR = __builtin_avr_swap(RAMdata[TOREALIND(posRAM + 1)]); //Set data to send by SPI
  byte c = __builtin_avr_swap(~SPDR); //read from SPI Data Register (double buffered)
  if (posRAM == 0) PORTB |= (1 << 0); //oscilloscope trigger start
  fixCounter++;

  PORTD |= (1 << 7); //write to MK OFF
  if ((byteWriteToMkStatus == 2) && (cmd_state == READ_BITS) && (posRAM == (byteNumToMk - 1))) {
    PORTD &= ~(1 << 7); //write to MK ON
    byteWriteToMkStatus = 0;
  }
  if ((byteWriteToMkStatus == 1) && (cmd_state == READ_BITS) && (posRAM == byteNumToMk)) {
    //RAMdata[57] = 0x55;
    c = byteToMk;
    byteWriteToMkStatus = 2;
  }
  
  RAMdata[posRAM] = c;
  posRAM = posRAM + 1;

  if ((cmd_state == WAIT_A_START) && (posRAM > 15)) {
    uint16_t t = posRAM - 14;
    if ( ((RAMdata[t+0] & RAMdata[t+3] & RAMdata[t+6] & RAMdata[t+9] & RAMdata[t+12] & 0x0F )
        | (RAMdata[t+2] & RAMdata[t+5] & RAMdata[t+8] & RAMdata[t+11]              & 0xF0 )) == 0xFF ) {
      posRAM = 15;
      cmd_state = WAIT_A_MARK;
    } else if (fixCounter == 0) {
      skipClockCycle = 1;
    }
  }

  if (posRAM == 315) {
    posRAM = 0;
    if ( ((RAMdata[0] & RAMdata[3] & RAMdata[6] & RAMdata[9] & RAMdata[12] & 0x0F )
        | (RAMdata[2] & RAMdata[5] & RAMdata[8] & RAMdata[11]              & 0xF0 )) == 0xFF ) {
      cmd_state = READ_BITS;
    } else {
      if (cmd_state == READ_BITS) cmd_state = WAIT_A_START;
      skipClockCycle = 1;
    }
  }

  if (skipClockCycle == 1) { //skip one clock cycle
    PORTB |= (1 << 1); 
    skipClockCycle = 0;
    NOP5; NOP5; NOP5; NOP5;
    PORTB &= ~(1 << 1);
  }

  PORTD &= ~(1 << 6);
  PORTB &= ~(1 << 0); //oscilloscope trigger end
}

void MKCALC::WriteRamToRom() {
  NOP5;
}

void MKCALC::WriteRamToMK() {
  NOP5; NOP5;
}

void MKCALC::MemoryPagesPrint() {
  serial_->print("cmd_state in MemoryPagesPrint - ");
  serial_->println(cmd_state);
  serial_->println("+----+--------------- + -------------- + -------------- + -------------- + -------------- +");
  serial_->println("|    | 0123456789ABCD | 0123456789ABCD | 0123456789ABCD | 0123456789ABCD | 0123456789ABCD |");
  serial_->println("+----+--------------- + -------------- + -------------- + -------------- + -------------- +");

  for (uint8_t j = 0; j < 3; j++) {
    serial_->print("  M");
    serial_->print(j + 1);
    serial_->print(" | ");
    for (uint16_t i = 0; i < 210; i++) {
      serial_->print(readTetrad(i * 3 + j, RAMdata), HEX);
      if (i > 0) {
        if (0 == (i + 1) % 14) serial_->print(" | ");
        if (0 == ((i + 1) % (14 * 5))) serial_->print("\r\n     | ");
      }
    }
    serial_->println("");
  }
  
  serial_->println("+----+--------------- + -------------- + -------------- + -------------- + -------------- +");
  serial_->println("");
}
